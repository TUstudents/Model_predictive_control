# run_ann_conversion_tests.py
import torch
import torch.nn as nn
import numpy as np
import onnx
import casadi as ca
import os
import shutil # For cleaning up ONNX files
import time
from onnx_casadi_converter import ONNXConversion, get_logger

# --- Setup ---
logger = get_logger(level="INFO") # Set to DEBUG for more verbosity
# logger = get_logger(level="DEBUG") 
TEMP_ONNX_DIR = "temp_onnx_models"
os.makedirs(TEMP_ONNX_DIR, exist_ok=True)

# --- Helper: PyTorch Model Definition ---
class GenericMLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dims, activation_name="tanh"):
        super().__init__()
        layers = []
        current_dim = input_dim
        
        activation_map = {
            "tanh": nn.Tanh,
            "relu": nn.ReLU,
            "sigmoid": nn.Sigmoid,
            "identity": nn.Identity
            # Add more if ONNXOperations supports them
        }
        if activation_name.lower() not in activation_map:
            raise ValueError(f"Unsupported activation: {activation_name}")
        ActModule = activation_map[activation_name.lower()]

        for h_dim in hidden_dims:
            layers.append(nn.Linear(current_dim, h_dim))
            layers.append(ActModule())
            current_dim = h_dim
        layers.append(nn.Linear(current_dim, output_dim))
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)

# --- Helper: Export to ONNX ---
def export_pytorch_to_onnx(pytorch_model, onnx_file_path, input_dim, batch_size=1, dynamic_batch=True):
    pytorch_model.eval()
    # For ONNX export, input shape should be (batch_size, input_dim)
    # If input_dim is already a tuple (e.g. for images (C,H,W)), then (batch_size, C, H, W)
    if isinstance(input_dim, int):
        dummy_input_shape = (batch_size, input_dim)
    else: # input_dim is a tuple like (C,H,W)
        dummy_input_shape = (batch_size, *input_dim)
        
    dummy_input = torch.randn(*dummy_input_shape, dtype=torch.float32)
    
    input_names = ["input_0"]
    output_names = ["output_0"]
    
    dynamic_axes_config = None
    if dynamic_batch:
        dynamic_axes_config = {'input_0': {0: 'batch_size'}, 'output_0': {0: 'batch_size'}}

    torch.onnx.export(
        pytorch_model,
        dummy_input,
        onnx_file_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=12, # Use a reasonably modern opset
        do_constant_folding=True,
        dynamic_axes=dynamic_axes_config
    )
    logger.info(f"Exported PyTorch model to {onnx_file_path}")
    return onnx_file_path

# --- Helper: Numerical Comparison ---
def compare_model_outputs(
    test_name, 
    pytorch_model, 
    casadi_func, 
    input_np, 
    atol=1e-6, 
    rtol=1e-5,
    casadi_input_is_vector=True # If CasADi func expects (N,1) vector
    ):
    # PyTorch inference
    pytorch_model.eval()
    with torch.no_grad():
        input_torch = torch.tensor(input_np.astype(np.float32))
        # Ensure batch dimension for PyTorch model, even if input_np is a single sample
        if input_torch.ndim == (len(pytorch_model.network[0].weight.shape) -1): # e.g. input_dim for MLP
            input_torch_batched = input_torch.unsqueeze(0)
        else:
            input_torch_batched = input_torch # Assume already batched if ndim matches

        pytorch_out_torch = pytorch_model(input_torch_batched)
        pytorch_out_np = pytorch_out_torch.numpy()
        if pytorch_out_np.shape[0] == 1: # If batch was 1, flatten it
            pytorch_out_np = pytorch_out_np.flatten()


    # CasADi inference
    casadi_input_val = input_np
    if casadi_input_is_vector:
        if input_np.ndim == 1: # if input_np is (features,)
            casadi_input_val = input_np.reshape(-1, 1) # CasADi func often expects (features, 1)
        elif input_np.ndim == 2 and input_np.shape[0] == 1: # if input_np is (1, features)
            casadi_input_val = input_np.T # transpose to (features, 1)
        # else assume input_np is already (features, batch_samples) for map or (features,1)
            
    casadi_out_dm = casadi_func(casadi_input_val)
    casadi_out_np = np.array(casadi_out_dm).flatten()
    
    logger.info(f"--- Comparison for: {test_name} ---")
    logger.info(f"Input (NumPy shape {input_np.shape}):\n{input_np.flatten()}")
    logger.info(f"PyTorch Output (NumPy shape {pytorch_out_np.shape}):\n{pytorch_out_np}")
    logger.info(f"CasADi Output (NumPy shape {casadi_out_np.shape}):\n{casadi_out_np}")
    
    is_close = np.allclose(pytorch_out_np, casadi_out_np, atol=atol, rtol=rtol)
    if is_close:
        logger.info(f"SUCCESS: Outputs are close for {test_name}.")
    else:
        diff = pytorch_out_np - casadi_out_np
        logger.error(f"FAILURE: Outputs differ significantly for {test_name}! Max diff: {np.max(np.abs(diff))}")
        logger.error(f"Difference array:\n{diff}")
    return is_close

# --- Helper: Gradient Comparison (Finite Differences vs CasADi AD) ---
def compare_gradients(
    test_name, 
    pytorch_model, # For reference, though not directly used for grad here
    casadi_func,   # The CasADi function to get AD gradients from
    input_np,      # Numerical input point to evaluate gradients
    atol=1e-5, 
    rtol=1e-4,
    casadi_input_is_vector=True,
    fd_epsilon = 1e-7 # Epsilon for finite differences
    ):
    
    logger.info(f"--- Gradient Comparison for: {test_name} ---")
    if casadi_input_is_vector:
        if input_np.ndim == 1:
            casadi_input_val = input_np.reshape(-1, 1)
        elif input_np.ndim == 2 and input_np.shape[0] == 1:
            casadi_input_val = input_np.T
        else:
            casadi_input_val = input_np # Assume correctly shaped
    else: # Input is not a simple vector, more complex
        casadi_input_val = input_np

    # CasADi AD Jacobian
    # Input of casadi_func is expected to be casadi_func.name_in()[0]
    # Output of casadi_func is expected to be casadi_func.name_out()[0]
    try:
        # jac_expr = casadi_func.jacobian() # This gives a function that computes Jacobian
        # This needs more careful handling if casadi_func has multiple inputs/outputs
        # For a single input vector, single output vector:
        # J_ad_func = casadi.Function(f'{test_name}_jac_ad', [casadi_func.sx_in()[0]], [ca.jacobian(casadi_func.sx_out()[0], casadi_func.sx_in()[0])])
        # J_ad = J_ad_func(casadi_input_val)

        # Simpler: use jacobian_old if inputs/outputs are single SX element
        if len(casadi_func.sx_in()) == 1 and len(casadi_func.sx_out()) == 1:
            J_ad_expr = ca.jacobian(casadi_func.sx_out()[0], casadi_func.sx_in()[0])
            J_ad_func = ca.Function(f'{test_name}_jac_ad', [casadi_func.sx_in()[0]], [J_ad_expr])
            J_ad = J_ad_func(casadi_input_val)
            J_ad_np = np.array(J_ad)
            logger.info(f"CasADi AD Jacobian shape: {J_ad_np.shape}")
            # logger.debug(f"CasADi AD Jacobian:\n{J_ad_np}")
        else:
            logger.warning(f"Cannot compute simple Jacobian for {test_name} due to multiple inputs/outputs in CasADi function. Skipping gradient check.")
            return True # Pass trivially
    except Exception as e:
        logger.error(f"ERROR computing CasADi AD Jacobian for {test_name}: {e}")
        return False

    # Finite Difference Jacobian (on CasADi function for consistency check)
    input_dim_fd = casadi_input_val.shape[0]
    output_dim_fd = J_ad_np.shape[0] # Inferred from AD Jacobian's output dimension
    
    J_fd_np = np.zeros((output_dim_fd, input_dim_fd))
    
    # Original output
    f_x = np.array(casadi_func(casadi_input_val)).flatten()

    for i in range(input_dim_fd):
        input_perturbed = np.array(casadi_input_val).copy()
        input_perturbed[i, 0] += fd_epsilon
        f_x_plus_eps = np.array(casadi_func(input_perturbed)).flatten()
        J_fd_np[:, i] = (f_x_plus_eps - f_x) / fd_epsilon
        
    logger.info(f"Finite Diff Jacobian shape: {J_fd_np.shape}")
    # logger.debug(f"Finite Diff Jacobian:\n{J_fd_np}")

    is_close = np.allclose(J_ad_np, J_fd_np, atol=atol, rtol=rtol)
    if is_close:
        logger.info(f"SUCCESS: AD and FD Jacobians are close for {test_name}.")
    else:
        diff = J_ad_np - J_fd_np
        logger.error(f"FAILURE: AD and FD Jacobians differ for {test_name}! Max diff: {np.max(np.abs(diff))}")
        logger.error(f"Difference Matrix (AD - FD):\n{diff}")
    return is_close


# --- Test Definitions ---
# List of test configurations: (name, input_dim, output_dim, hidden_dims, activation_name)
ann_test_configs = [
    ("Linear_3_2", 3, 2, [], "identity"), # No hidden layers, just linear
    ("MLP_1H_3_5_2_Tanh", 3, 2, [5], "tanh"),
    ("MLP_1H_5_8_3_ReLU", 5, 3, [8], "relu"),
    ("MLP_2H_4_10_5_1_Tanh", 4, 1, [10, 5], "tanh"),
    ("MLP_2H_2_4_4_2_Sigmoid", 2, 2, [4,4], "sigmoid"),
    ("MLP_3H_6_12_8_4_1_ReLU", 6, 1, [12,8,4], "relu"),
    # Add a case that might use Gemm (usually a single linear layer without bias, or if bias is C in Gemm)
    # PyTorch Linear layers are exported as MatMul + Add (if bias=True) or just MatMul
    # To test Gemm, the ONNX must have a Gemm node.
    # If a layer is y = alpha*A*X + beta*C, it would be Gemm.
    # Standard nn.Linear is usually not Gemm unless alpha/beta are not 1.0/1.0 or C is more complex.
]

# --- Main Test Loop ---
def run_tests():
    all_tests_passed = True
    for name, input_d, output_d, hidden_ds, act_name in ann_test_configs:
        logger.info(f"\n{'='*10} Running Test: {name} {'='*10}")
        
        # 1. Create and "train" PyTorch model (here, just initialize for structure)
        pytorch_model = GenericMLP(input_d, output_d, hidden_ds, act_name)
        # In a real scenario, load pre-trained weights or train here.
        # For testing the converter, initialized weights are fine.
        logger.info("PyTorch model created.")
        # print(pytorch_model) # For debugging architecture

        # 2. Export to ONNX
        onnx_file = os.path.join(TEMP_ONNX_DIR, f"{name}.onnx")
        try:
            export_pytorch_to_onnx(pytorch_model, onnx_file, input_d, batch_size=1, dynamic_batch=False) # Test with fixed batch 1 for simplicity
        except Exception as e:
            logger.error(f"Failed to export {name} to ONNX: {e}")
            all_tests_passed = False
            continue

        # 3. Convert ONNX to CasADi
        try:
            converter = ONNXConversion(onnx_file, model_name=name)
            logger.info(f"ONNX Converter initialized for {name}.")
            # print(converter) # For debugging converter's understanding of graph
            
            # CasADi symbolic input
            casadi_sym_in = ca.SX.sym("input_0_sx", input_d, 1) # Assume (features, 1) for CasADi
            
            # Perform conversion
            conversion_outputs = converter.convert(input_0=casadi_sym_in) # Match exported input name
            
            if "output_0" not in conversion_outputs:
                logger.error(f"ONNX output node 'output_0' not found in converted expressions for {name}.")
                all_tests_passed = False
                continue
                
            casadi_expr_out = conversion_outputs["output_0"]
            casadi_func = ca.Function(f"{name}_casadi", [casadi_sym_in], [casadi_expr_out], ["input_0"], ["output_0"])
            logger.info(f"CasADi function created for {name}.")

        except Exception as e:
            logger.error(f"Failed during ONNX to CasADi conversion for {name}: {e}")
            all_tests_passed = False
            continue

        # 4. Numerical Comparison (Forward Pass)
        test_input_np_single_sample = np.random.randn(input_d) # (features,)
        try:
            if not compare_model_outputs(f"{name}_ForwardPass", pytorch_model, casadi_func, test_input_np_single_sample):
                all_tests_passed = False
        except Exception as e:
            logger.error(f"Error during forward pass comparison for {name}: {e}")
            all_tests_passed = False
        
        # 5. Gradient Comparison
        try:
            if not compare_gradients(f"{name}_Gradients", pytorch_model, casadi_func, test_input_np_single_sample):
                all_tests_passed = False
        except Exception as e:
            logger.error(f"Error during gradient comparison for {name}: {e}")
            all_tests_passed = False

    logger.info(f"\n{'='*10} All Tests Summary {'='*10}")
    if all_tests_passed:
        logger.info("All defined ANN conversion tests passed successfully!")
    else:
        logger.error("One or more ANN conversion tests FAILED.")

    # Clean up temporary ONNX files
    # try:
    #     shutil.rmtree(TEMP_ONNX_DIR)
    #     logger.info(f"Cleaned up temporary directory: {TEMP_ONNX_DIR}")
    # except Exception as e:
    #     logger.warning(f"Could not clean up temp directory {TEMP_ONNX_DIR}: {e}")


if __name__ == "__main__":
    # Configure logging for the script run
    # logging.basicConfig(level=logging.DEBUG) # For very detailed output
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger_script = logging.getLogger() # Get root logger to affect all
    logger_script.setLevel(logging.INFO)
    
    run_tests()