# tests/test_mlp_activations.py
import pytest
import torch
import torch.nn as nn
import os
import numpy as np
import onnx
import casadi as ca
from onnx_casadi_converter import ONNXConversion, ONNXConversionError, UnsupportedONNXOperationError

from .test_mlp_simple import create_pytorch_mlp, export_to_onnx, compare_outputs # Reuse helpers

# Example:
def test_sigmoid_activation_mlp(temp_onnx_file):
    input_dim, hidden_dim, output_dim = 3, 5, 1
    pytorch_model = create_pytorch_mlp(input_dim, output_dim, [hidden_dim], nn.Sigmoid)
    # ... rest of the test logic similar to test_one_hidden_layer_tanh_mlp ...
    # (Define casadi_sym_in, convert, create Function, compare_outputs, test jacobian)
    pass # Placeholder for full implementation

def test_elu_activation_mlp(temp_onnx_file):
    # ... similar test for ELU ...
    pass