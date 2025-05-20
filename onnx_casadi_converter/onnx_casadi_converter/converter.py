# onnx_casadi_converter/converter.py
import casadi
import onnx
from onnx import numpy_helper
import numpy as np
from typing import List, Dict, Tuple, Union, Optional, Any

from .operations import ONNXOperations
from .utils import get_logger, ensure_casadi_type
from .exceptions import ONNXConversionError, UnsupportedONNXOperationError, ONNXGraphError, ShapeMismatchError

logger = get_logger()

class ONNXConversion:
    """
    Transforms an ONNX model into CasADi symbolic expressions.

    This class parses an ONNX model graph, and for each node, it calls a corresponding
    method in the ONNXOperations class to generate a CasADi expression.
    The resulting CasADi expressions can then be used to build CasADi Functions,
    suitable for optimization problems like NMPC.

    Warning:
        This feature is experimental. Support for ONNX operators is limited.
        Shape inference and handling for dynamic/symbolic shapes in CasADi SX
        can be challenging for certain ONNX operations (e.g., Reshape, Flatten
        with non-constant target shapes or inputs with unknown ranks).
    """
    def __init__(self, onnx_model_path_or_proto: Union[str, onnx.onnx_ml_pb2.ModelProto], model_name: Optional[str] = None):
        if isinstance(onnx_model_path_or_proto, str):
            try:
                self.onnx_model = onnx.load(onnx_model_path_or_proto)
                logger.info(f"Successfully loaded ONNX model from path: {onnx_model_path_or_proto}")
            except Exception as e:
                logger.error(f"Failed to load ONNX model from path: {onnx_model_path_or_proto}. Error: {e}")
                raise ONNXGraphError(f"Failed to load ONNX model: {e}")
        elif isinstance(onnx_model_path_or_proto, onnx.onnx_ml_pb2.ModelProto):
            self.onnx_model = onnx_model_path_or_proto
            logger.info("Initialized ONNXConversion with provided ONNX ModelProto object.")
        else:
            raise TypeError("Input 'onnx_model_path_or_proto' must be a path string to an .onnx file or an ONNX ModelProto object.")

        self.name = model_name if model_name else getattr(self.onnx_model.graph, 'name', "casadi_onnx_model")
        if not self.name: self.name = "casadi_onnx_model" # Ensure a default name

        self.graph = self.onnx_model.graph
        self.nodes = list(self.graph.node)

        self.initialized_tensors: Dict[str, casadi.DM] = {}
        for initializer in self.graph.initializer:
            np_array = numpy_helper.to_array(initializer)
            # Ensure float64 for CasADi, common for weights/biases
            self.initialized_tensors[initializer.name] = casadi.DM(np_array.astype(np.float64))
            logger.debug(f"Loaded initializer: '{initializer.name}' with shape {np_array.shape}, CasADi DM shape {self.initialized_tensors[initializer.name].shape}")

        self.input_info: Dict[str, Dict[str, Any]] = {}
        for graph_input_proto in self.graph.input:
            if graph_input_proto.name not in self.initialized_tensors:
                shape_dims = graph_input_proto.type.tensor_type.shape.dim
                # Use -1 for dynamic dimensions (None or 0 in ONNX dim_value often mean dynamic)
                # ONNX dim_value can be 0 if it's from a placeholder like None in TF.
                shape = tuple([d.dim_value if d.dim_value > 0 else -1 for d in shape_dims])
                elem_type = graph_input_proto.type.tensor_type.elem_type
                self.input_info[graph_input_proto.name] = {'shape': shape, 'dtype': elem_type, 'proto': graph_input_proto}
                logger.debug(f"Found graph input: '{graph_input_proto.name}' with ONNX shape {shape}, type {elem_type}")

        self.output_info: Dict[str, Dict[str, Any]] = {}
        for graph_output_proto in self.graph.output:
            shape_dims = graph_output_proto.type.tensor_type.shape.dim
            shape = tuple([d.dim_value if d.dim_value > 0 else -1 for d in shape_dims])
            elem_type = graph_output_proto.type.tensor_type.elem_type
            self.output_info[graph_output_proto.name] = {'shape': shape, 'dtype': elem_type, 'proto': graph_output_proto}
            logger.debug(f"Found graph output: '{graph_output_proto.name}' with ONNX shape {shape}, type {elem_type}")


        self.all_defined_names_in_graph = set(self.initialized_tensors.keys()) | set(self.input_info.keys())
        for node in self.nodes:
            for out_name in node.output:
                self.all_defined_names_in_graph.add(out_name)

        self.operations = ONNXOperations()
        self.converted_expressions: Dict[str, casadi.SX] = {}
        self._conversion_done = False

    def __repr__(self) -> str:
        repr_message = f"ONNXConversion for model '{self.name}' (Converted: {self._conversion_done})\n"
        repr_message += "----------------------------------\n"
        repr_message += "Model Inputs (from ONNX graph):\n"
        if not self.input_info: repr_message += "  (None defined or all are initializers)\n"
        for name, info in self.input_info.items():
            shape_repr = ['batch' if s == -1 or s == 0 else str(s) for s in info['shape']]
            repr_message += f"  - Name: '{name}', Expected ONNX Shape: ({', '.join(shape_repr)})\n"
        
        repr_message += "----------------------------------\n"
        repr_message += "Model Outputs (from ONNX graph):\n"
        if not self.output_info: repr_message += "  (None defined)\n"
        for name, info in self.output_info.items():
            shape_repr = ['batch' if s == -1 or s == 0 else str(s) for s in info['shape']]
            repr_message += f"  - Name: '{name}', Expected ONNX Shape: ({', '.join(shape_repr)})\n"
        
        if not self._conversion_done:
            repr_message += "----------------------------------\n"
            repr_message += "Call '.convert(**kwargs)' by supplying CasADi symbolic inputs for the model inputs.\n"
        else:
            repr_message += "----------------------------------\n"
            repr_message += "Model has been converted. Query instance['node_name'] for CasADi expressions.\n"
            repr_message += f"Available expression names: {sorted(list(self.converted_expressions.keys()))[:10]}...\n" # Show a few
        return repr_message

    def convert(self, **kwargs: Union[casadi.SX, casadi.MX]) -> Dict[str, Union[casadi.SX, casadi.MX]]:
        self.converted_expressions.clear()
        self._conversion_done = False
        logger.info(f"Starting ONNX to CasADi conversion for model '{self.name}'...")

        for name, casadi_input_sx in kwargs.items():
            if name not in self.input_info:
                logger.warning(f"Provided input '{name}' is not a defined graph input in the ONNX model. Ignoring.")
                continue
            if not isinstance(casadi_input_sx, (casadi.SX, casadi.MX)):
                raise TypeError(f"Input '{name}' must be CasADi SX or MX. Got {type(casadi_input_sx)}.")
            
            # Basic shape compatibility check (can be made more sophisticated)
            # Assumes CasADi input is for batch_size=1
            onnx_expected_shape = self.input_info[name]['shape']
            num_casadi_input_elements = casadi_input_sx.numel() # This is symbolic for SX!
            
            # If ONNX shape is (batch, D1, D2, ...), then product of D1*D2*... should match CasADi input numel.
            # This is still tricky due to symbolic numel.
            # For now, rely on user to provide correctly shaped CasADi inputs.
            # A practical check for MLP: if onnx is (batch, features), casadi sx is (features,1) or (1,features)
            if len(onnx_expected_shape) > 1 and onnx_expected_shape[1] > 0 : # e.g. (batch, N_feat)
                expected_features = np.prod([d for d_idx, d in enumerate(onnx_expected_shape) if d_idx > 0 and d > 0])
                # This check is only useful if casadi_input_sx.numel() is concrete
                # if casadi_input_sx.is_constant() and casadi_input_sx.numel() != expected_features:
                # logger.warning(f"Input '{name}': Numel mismatch. CasADi input has {casadi_input_sx.numel()} elements, ONNX expects ~{expected_features} features (excluding batch). Ensure shapes are compatible for batch_size=1.")


            self.converted_expressions[name] = casadi_input_sx
            logger.debug(f"Set symbolic model input '{name}' from CasADi var of shape {casadi_input_sx.shape}")

        for name, tensor_dm in self.initialized_tensors.items():
            self.converted_expressions[name] = tensor_dm
            logger.debug(f"Added initializer '{name}' (shape {tensor_dm.shape}) to expressions.")

        for node_proto in self.nodes:
            op_type = node_proto.op_type
            node_name_str = node_proto.name if node_proto.name else f"{op_type}_node_{node_proto.output[0]}" # Create a name if empty
            logger.debug(f"Processing ONNX Node: Name='{node_name_str}', Type='{op_type}', Inputs={list(node_proto.input)}, Outputs={list(node_proto.output)}")

            casadi_node_inputs = []
            input_names_for_op = [] # For ops like Unsqueeze/Squeeze that might take optional inputs from node.input list
            for input_idx, input_name in enumerate(node_proto.input):
                if not input_name: # Optional input not provided
                    logger.debug(f"  Node '{node_name_str}': Optional input at index {input_idx} is empty, skipping.")
                    # The operation handler must be able to deal with fewer arguments or Nones
                    # For variadic inputs, this is tricky. For now, assume named inputs must exist.
                    # Or pass None and let the op handle it.
                    # For ops like Unsqueeze/Squeeze, the 'axes' input is sometimes optional in older opsets
                    # and comes from attributes. New opsets have 'axes' as a mandatory or optional input tensor.
                    # We'll pass it along, and the specific op must handle if it's None (meaning from attribute)
                    # or if it's a true optional input.
                    # Let's always append to input_names_for_op for attribute parsing later
                    input_names_for_op.append(None) # Placeholder for actual CasADi expression
                    casadi_node_inputs.append(None) # This signals to the op to look for attribute
                    continue

                if input_name not in self.converted_expressions:
                    raise ONNXGraphError(f"ONNX Node '{node_name_str}' (type {op_type}): Cannot find input '{input_name}' in expressions or initializers.")
                casadi_node_inputs.append(self.converted_expressions[input_name])
                input_names_for_op.append(input_name)

            if hasattr(self.operations, op_type):
                op_func = getattr(self.operations, op_type)
                try:
                    # Pass all potential inputs, op_func should handle Nones for optional ones if designed so
                    node_outputs_sx = op_func(*casadi_node_inputs, attribute=node_proto.attribute)
                except Exception as e:
                    logger.error(f"Error executing CasADi op for ONNX node '{node_name_str}' (type {op_type}): {e}")
                    raise ONNXConversionError(f"Error in op '{op_type}': {e}") from e
            else:
                raise UnsupportedONNXOperationError(op_type, node_name_str)

            if len(node_proto.output) == 1:
                if isinstance(node_outputs_sx, (list, tuple)) and len(node_outputs_sx) == 1:
                    node_outputs_sx = node_outputs_sx[0]
                elif isinstance(node_outputs_sx, (list, tuple)) and len(node_outputs_sx) != 1:
                     raise ONNXGraphError(f"Node '{node_name_str}' expects 1 output, CasADi op returned {len(node_outputs_sx)}.")
                self.converted_expressions[node_proto.output[0]] = node_outputs_sx
                logger.debug(f"  -> Output '{node_proto.output[0]}' shape: {node_outputs_sx.shape if hasattr(node_outputs_sx, 'shape') else 'scalar'}")
            else:
                if not isinstance(node_outputs_sx, (list, tuple)) or len(node_outputs_sx) != len(node_proto.output):
                    raise ONNXGraphError(f"Node '{node_name_str}' (type {op_type}) expects {len(node_proto.output)} outputs, CasADi op returned {len(node_outputs_sx) if isinstance(node_outputs_sx, (list,tuple)) else 1}.")
                for i, out_name in enumerate(node_proto.output):
                    self.converted_expressions[out_name] = node_outputs_sx[i]
                    logger.debug(f"  -> Output '{out_name}' shape: {node_outputs_sx[i].shape if hasattr(node_outputs_sx[i], 'shape') else 'scalar'}")
        
        self._conversion_done = True
        logger.info(f"ONNX to CasADi conversion completed for model '{self.name}'.")
        return self.get_model_outputs()

    def __getitem__(self, key: str) -> Union[casadi.SX, casadi.MX, casadi.DM]:
        if not self._conversion_done:
            # Check if it's a predefined input or initializer even if convert wasn't called
            if key in self.input_info:
                # This should not be directly queryable before convert sets its symbolic value
                raise ONNXGraphError(f"Input node '{key}' is defined. Call '.convert()' with this name as a keyword argument to provide its symbolic CasADi expression first.")
            if key in self.initialized_tensors:
                return self.initialized_tensors[key] # Allow access to initializers always
            raise ONNXGraphError("Conversion not performed yet. Call '.convert()' first.")
        
        if key in self.converted_expressions:
            return self.converted_expressions[key]
        else:
            raise KeyError(f"Node or tensor name '{key}' not found in converted ONNX graph expressions. Known names after conversion: {list(self.converted_expressions.keys())}")

    def get_model_outputs(self) -> Dict[str, Union[casadi.SX, casadi.MX]]:
        if not self._conversion_done:
            raise ONNXGraphError("Conversion not performed yet. Call '.convert()' first.")
        
        outputs_dict = {}
        for out_proto in self.graph.output: # Iterate through defined graph outputs
            name = out_proto.name
            if name in self.converted_expressions:
                outputs_dict[name] = self.converted_expressions[name]
            else:
                # This should not happen if conversion was successful and graph is valid
                logger.error(f"Defined graph output '{name}' not found in converted expressions!")
        return outputs_dict

    def get_casadi_function(self, input_names: Optional[List[str]] = None, 
                            output_names: Optional[List[str]] = None, 
                            func_name: Optional[str] = None) -> casadi.Function:
        if not self._conversion_done:
            raise ONNXGraphError("Conversion not performed. Call '.convert()' first.")

        if input_names is None: # Default to all defined model inputs
            input_names = list(self.input_info.keys())
            if not input_names:
                 raise ONNXGraphError("No model inputs found in ONNX graph to create CasADi function.")
            logger.info(f"Using default graph inputs for CasADi function: {input_names}")

        if output_names is None: # Default to all defined model outputs
            output_names = [out_proto.name for out_proto in self.graph.output]
            if not output_names:
                raise ONNXGraphError("No model outputs found in ONNX graph to create CasADi function.")
            logger.info(f"Using default graph outputs for CasADi function: {output_names}")
            
        casadi_sym_inputs = []
        for name in input_names:
            # The symbolic inputs provided to .convert() are stored with these names
            if name not in self.converted_expressions or not isinstance(self.converted_expressions[name], (casadi.SX, casadi.MX)):
                # Check if it's a graph input that wasn't supplied to convert() yet.
                # This function should be called AFTER convert() where inputs are symbolic.
                if name in self.input_info:
                    raise ONNXGraphError(f"Graph input '{name}' must be a symbolic variable previously supplied to '.convert()' to be used in get_casadi_function().")
                raise ONNXGraphError(f"Cannot find symbolic input '{name}' in converted expressions.")
            casadi_sym_inputs.append(self.converted_expressions[name])
        
        casadi_sym_outputs = []
        for name in output_names:
            if name not in self.converted_expressions:
                raise ONNXGraphError(f"Cannot find symbolic output '{name}' in converted expressions.")
            casadi_sym_outputs.append(self.converted_expressions[name])
        
        final_func_name = func_name if func_name else self.name + "_casadi_func"
        
        return casadi.Function(final_func_name, casadi_sym_inputs, casadi_sym_outputs, input_names, output_names)