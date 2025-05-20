# onnx_casadi_converter/__init__.py
from .converter import ONNXConversion
from .operations import ONNXOperations # Make it accessible if someone wants to inspect/extend
from .exceptions import ONNXConversionError, UnsupportedONNXOperationError, ONNXGraphError, ShapeMismatchError
from .utils import get_logger, ensure_casadi_type # Expose key utilities if needed

__version__ = "0.1.0" # Start versioning

__all__ = [
    "ONNXConversion",
    "ONNXOperations", # Optional to export
    "ONNXConversionError",
    "UnsupportedONNXOperationError",
    "ONNXGraphError",
    "ShapeMismatchError",
    "get_logger",
    "ensure_casadi_type"
]