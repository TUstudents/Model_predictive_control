# tests/test_error_handling.py
import pytest
import torch
import torch.nn as nn
import numpy as np
import onnx
import casadi as ca
from onnx_casadi_converter import ONNXConversion, UnsupportedONNXOperationError, ONNXGraphError
from .test_mlp_simple import export_to_onnx # Reuse helper

class UnsupportedOpModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.linear = nn.Linear(3,1)
    def forward(self, x):
        # Using a PyTorch op that we know isn't in our ONNXOperations
        # For example, let's pretend 'torch.erf' translates to an ONNX 'Erf' op
        # that we haven't implemented.
        # For a real test, find an actual op not in your list or make one up for the test.
        # This requires knowing how PyTorch ops map to ONNX ops.
        # A simpler test: export an ONNX file manually with an unsupported op.
        return torch.sin(self.linear(x)) # Assume 'Sin' is not implemented in ONNXOperations

def test_unsupported_operation(tmp_path):
    # Create a dummy ONNX file with an unsupported op_type
    unsupported_op_type = "SuperFancyNewOp"
    
    node_def = onnx.helper.make_node(
        unsupported_op_type,
        inputs=['X'],
        outputs=['Y'],
        name='unsupported_node'
    )
    graph_input = onnx.helper.make_tensor_value_info('X', onnx.TensorProto.FLOAT, [None, 3])
    graph_output = onnx.helper.make_tensor_value_info('Y', onnx.TensorProto.FLOAT, [None, 1])

    graph_def = onnx.helper.make_graph(
        [node_def],
        'test-graph-unsupported',
        [graph_input],
        [graph_output]
    )
    model_def = onnx.helper.make_model(graph_def, producer_name='onnx-example')
    unsupported_onnx_path = tmp_path / "unsupported_model.onnx"
    onnx.save(model_def, unsupported_onnx_path)

    converter = ONNXConversion(unsupported_onnx_path)
    casadi_sym_in = ca.SX.sym("X", 1, 3) # Match dummy graph input name and feature dim

    with pytest.raises(UnsupportedONNXOperationError) as excinfo:
        converter.convert(X=casadi_sym_in)
    assert unsupported_op_type in str(excinfo.value)
    assert "unsupported_node" in str(excinfo.value)


def test_missing_input_to_convert(temp_onnx_file):
    input_dim, output_dim = 3, 1
    pytorch_model = nn.Linear(input_dim, output_dim)
    onnx_path = export_to_onnx(pytorch_model, temp_onnx_file, input_dim) # Input name will be 'input_0'
    
    converter = ONNXConversion(onnx_path)
    with pytest.raises(ValueError) as excinfo: # Changed from ONNXGraphError for more specific check
        converter.convert(wrong_input_name=ca.SX.sym("w",1,3))
    assert "Missing required model input: 'input_0'" in str(excinfo.value)

def test_malformed_graph_input_not_found(tmp_path):
    # Create an ONNX graph where a node refers to a non-existent input tensor
    node_def = onnx.helper.make_node(
        'Identity', # A supported op
        inputs=['NonExistentInput'], # This input is not a graph input nor an initializer
        outputs=['Y'],
    )
    graph_input = onnx.helper.make_tensor_value_info('X', onnx.TensorProto.FLOAT, [None, 3]) # Actual graph input
    graph_output = onnx.helper.make_tensor_value_info('Y', onnx.TensorProto.FLOAT, [None, 1])
    
    graph_def = onnx.helper.make_graph([node_def], 'test-graph-malformed', [graph_input], [graph_output])
    model_def = onnx.helper.make_model(graph_def, producer_name='onnx-example')
    malformed_onnx_path = tmp_path / "malformed_model.onnx"
    onnx.save(model_def, malformed_onnx_path)

    converter = ONNXConversion(malformed_onnx_path)
    casadi_sym_in = ca.SX.sym("X", 1, 3) # Provide the actual graph input
    with pytest.raises(ONNXGraphError) as excinfo:
        converter.convert(X=casadi_sym_in)
    assert "Cannot find input 'NonExistentInput'" in str(excinfo.value)