# tests/conftest.py
import pytest
import torch
import torch.nn as nn
import numpy as np
import onnx
import os

# Fixture to create a temporary ONNX file path that gets cleaned up
@pytest.fixture
def temp_onnx_file(tmp_path):
    file_path = tmp_path / "temp_model.onnx"
    yield file_path
    if file_path.exists():
        os.remove(file_path)

# You could add fixtures for common PyTorch models here if they are reused across many tests