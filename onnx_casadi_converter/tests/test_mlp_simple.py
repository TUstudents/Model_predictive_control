# tests/test_mlp_simple.py
import pytest
import torch
import torch.nn as nn
import os
import numpy as np
import onnx
import casadi as ca
from onnx_casadi_converter import ONNXConversion, ONNXConversionError, UnsupportedONNXOperationError

# Helper function to create a simple PyTorch MLP
def create_pytorch_mlp(input_dim, output_dim, hidden_dims, activation_module):
    layers = [nn.Linear(input_dim, hidden_dims[0]), activation_module()]
    for i in range(len(hidden_dims) - 1):
        layers.extend([nn.Linear(hidden_dims[i], hidden_dims[i+1]), activation_module()])
    layers.append(nn.Linear(hidden_dims[-1], output_dim))
    return nn.Sequential(*layers)

# Helper function to export PyTorch model to ONNX
def export_to_onnx(pytorch_model, onnx_file_path, input_dim, batch_size=1):
    pytorch_model.eval()
    dummy_input = torch.randn(batch_size, input_dim, dtype=torch.float32)
    input_names = ["input_0"]
    output_names = ["output_0"]
    
    # Ensure the directory for onnx_file_path exists
    os.makedirs(os.path.dirname(onnx_file_path), exist_ok=True)

    torch.onnx.export(
        pytorch_model,
        dummy_input,
        onnx_file_path,
        input_names=input_names,
        output_names=output_names,
        opset_version=11, # Or a version well-supported by your converter
        do_constant_folding=True,
        dynamic_axes={'input_0': {0: 'batch_size'}, 'output_0': {0: 'batch_size'}} if batch_size is None else None
    )
    return onnx_file_path

# Helper function to compare PyTorch and CasADi outputs
def compare_outputs(pytorch_model, casadi_func, input_np, atol=1e-6):
    # PyTorch inference
    pytorch_model.eval()
    with torch.no_grad():
        input_torch = torch.tensor(input_np.astype(np.float32))
        if input_torch.ndim == 1: # Ensure batch dimension for PyTorch model
             input_torch = input_torch.unsqueeze(0)
        pytorch_out_torch = pytorch_model(input_torch)
        pytorch_out_np = pytorch_out_torch.numpy().flatten()

    # CasADi inference
    # CasADi function often expects column vectors for inputs if not specified otherwise
    if input_np.ndim == 1:
        casadi_input_np = input_np.reshape(-1, 1)
    else: # if input_np is already (batch, features), take first sample
        casadi_input_np = input_np[0,:].reshape(-1,1)
        
    casadi_out_dm = casadi_func(casadi_input_np)
    casadi_out_np = np.array(casadi_out_dm).flatten()
    
    print(f"Input: {input_np.flatten()}")
    print(f"PyTorch output: {pytorch_out_np}")
    print(f"CasADi output:  {casadi_out_np}")
    assert np.allclose(pytorch_out_np, casadi_out_np, atol=atol), \
        f"Outputs differ significantly! Diff: {pytorch_out_np - casadi_out_np}"


# --- Test Cases ---

def test_single_layer_linear_mlp(temp_onnx_file):
    """Test a simple MLP: Input -> Linear -> Output"""
    input_dim, output_dim = 3, 2
    pytorch_model = nn.Linear(input_dim, output_dim)
    
    onnx_path = export_to_onnx(pytorch_model, temp_onnx_file, input_dim)
    
    converter = ONNXConversion(onnx_path)
    
    # Define CasADi symbolic input
    # Assume batch_size=1 for CasADi function, so input shape (input_dim, 1)
    # or (1, input_dim) depending on convention, let's try (input_dim,1)
    casadi_sym_in = ca.SX.sym("input_0_sx", input_dim, 1) 
    
    conversion_outputs = converter.convert(input_0=casadi_sym_in) # Match ONNX input name
    assert "output_0" in conversion_outputs 
    casadi_expr_out = conversion_outputs["output_0"]
    
    casadi_func = ca.Function("mlp_func", [casadi_sym_in], [casadi_expr_out])
    
    # Test with numerical input
    test_input_np = np.random.randn(input_dim)
    compare_outputs(pytorch_model, casadi_func, test_input_np)

    # Test gradient computation (simple check)
    try:
        J = casadi_func.jacobian_old(0,0) # Jacobian of output wrt input
        J_eval = J(test_input_np.reshape(-1,1))
        assert J_eval.shape == (output_dim, input_dim)
        print(f"Jacobian computation successful, shape: {J_eval.shape}")
    except Exception as e:
        pytest.fail(f"CasADi Jacobian computation failed: {e}")


def test_one_hidden_layer_tanh_mlp(temp_onnx_file):
    """Test MLP: Input -> Linear -> Tanh -> Linear -> Output"""
    input_dim, hidden_dim, output_dim = 4, 5, 2
    pytorch_model = create_pytorch_mlp(input_dim, output_dim, [hidden_dim], nn.Tanh)
    
    onnx_path = export_to_onnx(pytorch_model, temp_onnx_file, input_dim)
    converter = ONNXConversion(onnx_path)
    
    casadi_sym_in = ca.SX.sym("input_0_sx", input_dim, 1)
    conversion_outputs = converter.convert(input_0=casadi_sym_in)
    casadi_expr_out = conversion_outputs["output_0"]
    casadi_func = ca.Function("mlp_func_tanh", [casadi_sym_in], [casadi_expr_out])
    
    test_input_np = np.random.randn(input_dim)
    compare_outputs(pytorch_model, casadi_func, test_input_np)

    try:
        J = casadi_func.jacobian_old(0,0)
        J_eval = J(test_input_np.reshape(-1,1))
        assert J_eval.shape == (output_dim, input_dim)
    except Exception as e:
        pytest.fail(f"CasADi Jacobian computation failed for Tanh MLP: {e}")

def test_two_hidden_layers_relu_mlp(temp_onnx_file):
    """Test MLP: Input -> L -> ReLU -> L -> ReLU -> L -> Output"""
    input_dim, h1, h2, output_dim = 3, 8, 4, 1
    pytorch_model = create_pytorch_mlp(input_dim, output_dim, [h1, h2], nn.ReLU)

    onnx_path = export_to_onnx(pytorch_model, temp_onnx_file, input_dim)
    converter = ONNXConversion(onnx_path)

    casadi_sym_in = ca.SX.sym("input_0_sx", input_dim, 1)
    conversion_outputs = converter.convert(input_0=casadi_sym_in)
    casadi_expr_out = conversion_outputs["output_0"]
    casadi_func = ca.Function("mlp_func_relu", [casadi_sym_in], [casadi_expr_out])

    test_input_np = np.random.randn(input_dim) 
    compare_outputs(pytorch_model, casadi_func, test_input_np)
    
    try:
        J = casadi_func.jacobian_old(0,0)
        J_eval = J(test_input_np.reshape(-1,1))
        assert J_eval.shape == (output_dim, input_dim)
    except Exception as e:
        pytest.fail(f"CasADi Jacobian computation failed for ReLU MLP: {e}")

# Add more tests for Sigmoid, Elu, Identity if implemented in ONNXOperations
# Add tests for Gemm vs MatMul+Add equivalence if both are supported ways to export layers