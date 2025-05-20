# onnx_casadi_converter/utils.py
import logging
import numpy as np
import casadi
from typing import Union, Tuple, Any

# --- Logger Setup ---
# Using a getter function allows configuring the logger from outside if needed
_logger_instance = None

def get_logger(name="ONNXCasADiConverter", level=logging.INFO):
    """Gets a configured logger instance."""
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = logging.getLogger(name)
        # Prevent adding multiple handlers if called multiple times in same session
        if not _logger_instance.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            _logger_instance.addHandler(handler)
        _logger_instance.setLevel(level)
    return _logger_instance

# --- Shape Utilities (Basic examples, can be expanded) ---
def symbolic_numel(sx_var: casadi.SX) -> Union[int, casadi.SX]:
    """
    Tries to get the number of elements. Returns int if constant, otherwise symbolic.
    Note: CasADi SX.numel() itself returns an SX.
    This is more of a conceptual helper for when we *need* a concrete number.
    """
    if sx_var.is_constant():
        return sx_var.numel() # This will be an int for DM inputs made const
    # For truly symbolic SX, numel() is symbolic. Getting a concrete int is not generally possible.
    # This highlights a core challenge with fully symbolic shape manipulation in SX.
    # For now, let's just return CasADi's numel().
    return sx_var.numel() # Returns an SX

def symbolic_shape(var: Union[casadi.SX, casadi.MX, casadi.DM]) -> Tuple[Union[int, casadi.SX], ...]:
    """
    Returns the shape of a CasADi SX/MX/DM variable as a tuple.
    Elements can be int (for DM or concrete SX/MX) or SX (for symbolic dimensions).
    """
    if isinstance(var, casadi.DM):
        return var.shape # DM shape is always concrete tuple of ints
    
    # For SX/MX, .shape can be a Sparsity object or other internal representations
    # .size1() and .size2() give rows and columns, which can be symbolic SX themselves.
    # For N-D, this gets more complex. CasADi primarily treats SX/MX as 2D matrices.
    
    if var.is_scalar(): # Checks if it's 1x1
        return (1, 1) # Or just (1,) if you prefer, but (1,1) is common for matrix view

    # Try to get rows and columns
    try:
        r = var.size1()
        c = var.size2()
        
        # If they are constant SX, convert to int
        if isinstance(r, casadi.SX) and r.is_constant():
            r_val = int(casadi.DM(r))
        elif isinstance(r, int):
            r_val = r
        else: # Symbolic row count
            r_val = r 
            
        if isinstance(c, casadi.SX) and c.is_constant():
            c_val = int(casadi.DM(c))
        elif isinstance(c, int):
            c_val = c
        else: # Symbolic col count
            c_val = c
            
        return (r_val, c_val)
    except Exception:
        # Fallback if size1/size2 not directly applicable or fail
        # This might happen for uninitialized SX or other edge cases.
        # Defaulting to (numel, 1) if it seems vector-like
        numel_val = var.numel() # This is SX if var is SX
        if isinstance(numel_val, casadi.SX) and numel_val.is_constant():
            n_val = int(casadi.DM(numel_val))
            return (n_val, 1) if n_val != 1 else (1,1) # Treat as column vector
        return (numel_val, casadi.SX(1)) if numel_val != casadi.SX(1) else (casadi.SX(1), casadi.SX(1))


def ensure_casadi_type(val: Any, target_type: str = "SX") -> Union[casadi.SX, casadi.MX, casadi.DM]:
    """Ensures the value is a CasADi type, converting NumPy arrays to DM."""
    if isinstance(val, (casadi.SX, casadi.MX, casadi.DM)):
        return val
    if isinstance(val, np.ndarray):
        return casadi.DM(val.astype(np.float64)) # Ensure float64 for CasADi
    if isinstance(val, (int, float)):
        return casadi.DM(float(val)) # Scalar
    raise TypeError(f"Cannot convert type {type(val)} to a CasADi type.")