# onnx_casadi_converter/exceptions.py

class ONNXConversionError(Exception):
    """Base exception for errors during ONNX to CasADi conversion."""
    pass

class UnsupportedONNXOperationError(ONNXConversionError):
    """Raised when an ONNX operation is not supported by the converter."""
    def __init__(self, op_type: str, node_name: str = ""):
        self.op_type = op_type
        self.node_name = node_name
        message = f"ONNX operation type '{op_type}'"
        if node_name:
            message += f" (node '{node_name}')"
        message += " is not implemented in ONNXOperations class."
        super().__init__(message)

class ONNXGraphError(ONNXConversionError):
    """Raised for issues related to the ONNX graph structure or inputs."""
    pass

class ShapeMismatchError(ONNXConversionError):
    """Raised when there's a mismatch in expected vs. actual tensor shapes."""
    pass