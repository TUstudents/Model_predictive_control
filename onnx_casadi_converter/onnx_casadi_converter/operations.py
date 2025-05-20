# onnx_casadi_converter/operations.py
import casadi
import numpy as np
from typing import List, Dict, Any, Union, Optional
from .utils import get_logger # Use the utility logger
import onnx
from onnx import AttributeProto, numpy_helper
from .exceptions import UnsupportedONNXOperationError, ShapeMismatchError

logger = get_logger()

class ONNXOperations:
    """
    CasADi implementations of ONNX operations.
    This class is used by ONNXConversion to translate ONNX nodes into CasADi expressions.
    Each method corresponds to an ONNX op_type.
    Methods should accept CasADi SX/MX/DM types as inputs and return CasADi SX/MX.
    """
    def __init__(self):
        pass

    def _parse_attributes(self, attribute_proto: List[onnx.AttributeProto]) -> Dict[str, Any]:
        attrs = {}
        if attribute_proto is None: # Should not happen if node has attributes
            return attrs
        for attr in attribute_proto:
            if attr.type == onnx.AttributeProto.FLOAT: attrs[attr.name] = attr.f
            elif attr.type == onnx.AttributeProto.INT: attrs[attr.name] = attr.i
            elif attr.type == onnx.AttributeProto.STRING: attrs[attr.name] = attr.s.decode('utf-8')
            elif attr.type == onnx.AttributeProto.FLOATS: attrs[attr.name] = list(attr.floats)
            elif attr.type == onnx.AttributeProto.INTS: attrs[attr.name] = list(attr.ints)
            elif attr.type == onnx.AttributeProto.TENSOR:
                from onnx import numpy_helper # Local import
                attrs[attr.name] = numpy_helper.to_array(attr.t)
            else:
                logger.warning(f"Attribute type {attr.type} for attribute '{attr.name}' not fully parsed yet.")
                attrs[attr.name] = attr # Store raw
        return attrs

    # --- Activation Functions ---
    def Tanh(self, x: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.tanh(x)

    def Sigmoid(self, x: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.sigmoid(x) # CasADi has a direct sigmoid

    def Relu(self, x: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.fmax(0, x)

    def Elu(self, x: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        attrs = self._parse_attributes(attribute)
        alpha = attrs.get('alpha', 1.0)
        return casadi.fmax(0,x) + casadi.fmin(0, alpha * (casadi.exp(x)-1))

    def Identity(self, x: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return x

    # --- Matrix Operations ---
    def MatMul(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        if A.ndim > 2 or B.ndim > 2:
            # Attempt to handle batch matmul if first dim is the same and > 1 (very experimental)
            # This is usually not what we want for f(x,u) in MPC where batch is 1.
            # For a (batch, M, K) @ (batch, K, N) -> (batch, M, N)
            # CasADi SX doesn't directly support this. Would need a loop or map.
             logger.warning(f"MatMul encountered inputs with >2 dimensions (A:{A.shape}, B:{B.shape}). Assuming standard 2D mtimes. N-D behavior might differ from ONNX spec.")
        if A.shape[-1] != B.shape[0] and not (A.is_vector() or B.is_vector()): # Check inner dimensions for 2D
             if B.shape[-1] == A.shape[0] and A.is_vector() and B.is_vector(): # (K,) @ (K,) type ops -> scalar
                 pass # dot product case
             elif A.shape[0]==1 and A.shape[1]==B.shape[0]: # (1,K) @ (K,N)
                 pass
             elif B.shape[1]==1 and A.shape[1]==B.shape[0]: # (M,K) @ (K,1)
                 pass
             else:
                raise ShapeMismatchError(f"MatMul inner dimensions mismatch: A shape {A.shape}, B shape {B.shape}")
        return casadi.mtimes(A, B)

    def Gemm(self, A: casadi.SX, B: casadi.SX, C: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        attrs = self._parse_attributes(attribute)
        alpha = attrs.get('alpha', 1.0)
        beta = attrs.get('beta', 1.0)
        transA = attrs.get('transA', 0)
        transB = attrs.get('transB', 0)

        A_op = A.T if transA == 1 else A
        B_op = B.T if transB == 1 else B
        
        # Check shapes for mtimes
        if A_op.shape[-1] != B_op.shape[0]:
            raise ShapeMismatchError(f"Gemm inner dimensions mismatch after transpose: A_op shape {A_op.shape}, B_op shape {B_op.shape}")

        res_mult = alpha * casadi.mtimes(A_op, B_op)

        # Handle broadcasting of C: (target_shape is res_mult.shape)
        # ONNX allows C to be scalar or to be broadcastable to A_op @ B_op.
        if C.is_scalar():
            # If C is scalar, CasADi's + will broadcast correctly.
            res = res_mult + beta * C
        elif C.shape == res_mult.shape:
            res = res_mult + beta * C
        elif C.is_vector(): # C is a vector, res_mult is a matrix
            if C.shape[0] == res_mult.shape[0] and C.shape[1] == 1 and res_mult.shape[1] > 1: # C is column, broadcast across columns
                res = res_mult + beta * casadi.repmat(C, 1, res_mult.shape[1])
            elif C.shape[1] == res_mult.shape[1] and C.shape[0] == 1 and res_mult.shape[0] > 1: # C is row, broadcast across rows
                res = res_mult + beta * casadi.repmat(C, res_mult.shape[0], 1)
            elif res_mult.is_vector() and C.numel()==res_mult.numel(): # Both vectors of same numel
                 res = res_mult + beta * C # Direct element-wise
            else:
                raise ShapeMismatchError(f"Gemm: C shape {C.shape} not broadcastable to A@B shape {res_mult.shape}")
        else: # C is matrix but not same shape
             raise ShapeMismatchError(f"Gemm: C shape {C.shape} not compatible with A@B shape {res_mult.shape}")
        return res

    # --- Element-wise Arithmetic Operations ---
    def Add(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        # CasADi's + handles broadcasting
        return A + B

    def Sub(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return A - B

    def Mul(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return A * B

    def Div(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return A / B
    
    def Pow(self, A: casadi.SX, B: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        # ONNX Pow(base, exponent)
        return A**B

    def Sqrt(self, X: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.sqrt(X)

    def Exp(self, X: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.exp(X)
        
    def Log(self, X: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        return casadi.log(X)

    def Sum(self, *args: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        if not args: return casadi.SX(0) # Sum of empty set is 0
        out = args[0]
        for i in range(1, len(args)):
            out = out + args[i]
        return out

    # --- Tensor Manipulation Operations ---
    def Transpose(self, data: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        attrs = self._parse_attributes(attribute)
        perm = attrs.get('perm', None)
        
        if perm is not None:
            perm = list(perm) # Ensure it's a list of ints
            if data.ndim != len(perm):
                 raise ShapeMismatchError(f"Transpose: data has {data.ndim} dimensions, but perm has {len(perm)} elements.")
            # CasADi SX does not have a generic N-D transpose with permutation.
            # For 2D (matrix)
            if data.ndim == 2:
                if perm == [1,0]: return data.T
                if perm == [0,1]: return data # No change
                raise NotImplementedError(f"Transpose for 2D data with perm {perm} not simply data.T.")
            raise NotImplementedError(f"N-D Transpose with permutation {perm} for CasADi SX is not implemented.")
        # Default transpose for 2D matrix if no perm or perm implies standard transpose
        if data.ndim == 2: return data.T
        if data.ndim == 1: return data # Transpose of vector is itself in some contexts, or shape change
        raise NotImplementedError(f"Transpose for data with {data.ndim} dimensions without explicit perm is ambiguous.")

    def Reshape(self, data: casadi.SX, shape_tensor: Union[casadi.SX, casadi.DM], attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        if not isinstance(shape_tensor, casadi.DM):
            if shape_tensor.is_constant():
                shape_tensor_dm = casadi.DM(shape_tensor)
            else:
                raise NotImplementedError(f"Reshape with a symbolic shape_tensor ('{shape_tensor.name() if shape_tensor.name() else 'unknown'}') is not supported for CasADi SX if it contains -1 or 0 that requires inference based on symbolic input data size.")
        else:
            shape_tensor_dm = shape_tensor

        if not shape_tensor_dm.is_integer_elements():
            raise TypeError("Shape tensor for Reshape must contain integers.")
        
        new_shape_list = [int(s) for s in shape_tensor_dm.full().flatten()]

        # Handle ONNX specific shape values: 0 means copy dimension, -1 means infer.
        # This is complex for symbolic 'data' in CasADi SX if its shape is not fully concrete.
        # For this implementation, we will assume that if 0 or -1 are present,
        # they can be resolved because the *total number of elements* in `data` is known or can be made known
        # for the specific context (e.g. if `data` comes from an input with fixed feature size).
        # This is a simplification. A fully robust ONNX Reshape is very hard for CasADi SX.
        
        # If data is SX, data.numel() is SX. data.shape is symbolic.
        # We can only make this work if new_shape_list after resolving 0 and -1 is concrete.
        # Let's assume for now that new_shape_list will be concrete.
        
        # Crude handling of -1 and 0 (assumes only one -1, and data.numel() can be figured out if 'data' comes from fixed size inputs)
        # This part needs a lot of care for general symbolic 'data'.
        # For fixed-size MLP layers, the target shapes of reshapes are usually constants.
        # If numel_data is needed and data is SX:
        #   If data comes from an input ca.SX.sym('input', N_feat, 1), numel_data = N_feat.
        #   This context is lost here.
        
        # The most robust way is if new_shape_list (from ONNX shape_tensor) is already fully defined positive integers.
        if any(s < -1 for s in new_shape_list):
            raise ValueError("Invalid value in target shape for Reshape.")

        # Try to resolve 0s and -1 if present using CasADi's reshape capabilities
        # CasADi's reshape can take a tuple. If it's symbolic, its power is limited.
        # For now, we pass the list and hope CasADi's reshape can handle it.
        # This is more of a forward pass to CasADi's own reshape logic.
        try:
            # CasADi reshape needs a tuple for the shape argument
            reshaped_data = casadi.reshape(data, tuple(new_shape_list))
            logger.debug(f"Reshape to {new_shape_list}, CasADi result shape: {reshaped_data.shape}")
            return reshaped_data
        except Exception as e:
            raise ShapeMismatchError(f"CasADi reshape failed for target shape {new_shape_list}. Original data shape (symbolic): {data.shape}. Error: {e}")


    def Flatten(self, data: casadi.SX, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        attrs = self._parse_attributes(attribute)
        axis = attrs.get('axis', 1) # Default axis is 1 for ONNX Flatten

        # ONNX Flatten: Flattens X into a 2D matrix of shape (D_0*...*D_{axis-1}, D_{axis}*...*D_{N-1}).
        # Common use case from Conv (N, C, H, W) with axis=1 -> (N, C*H*W)
        # For MPC, N (batch) is typically 1. So, (1, C, H, W) -> (1, C*H*W)
        # CasADi SX shape is symbolic. data.numel() is symbolic.
        # This is very difficult to implement generically for symbolic SX without concrete dimension values.

        # Simplification: Assume data.shape[0] is batch and is 1 for an MPC step.
        # Flatten everything after the first dimension.
        if axis == 1:
            if data.ndim == 1: # Already 1D, ONNX spec says output is (1, D0)
                return casadi.reshape(data, (1, data.numel()))
            if data.ndim == 2: # (Batch, Features), already "flat" in this sense
                return data 
            # For N-D tensor, e.g. (1, C, H, W) for image-like input
            # We want to reshape to (1, C*H*W).
            # CasADi's reshape (target_shape_tuple) can sometimes infer one dimension if it's -1
            # but that requires other dimensions and total numel to be somewhat concrete.
            # If data.shape[0] is truly symbolic, this is hard.
            # If we *assume* the first dim is effectively 1:
            if data.is_ and data.shape[0] == 1: # Known to be (1, dim1, dim2 ...)
                num_elements_to_flatten = data.numel() # This will be dim1*dim2*...
                return casadi.reshape(data, (1, num_elements_to_flatten))
            else:
                logger.warning("Flatten for axis=1 on N-D tensor with symbolic/unknown batch size is complex. Attempting simple reshape to (1, numel).")
                # This might be wrong if true batch > 1, but for MPC step it's often okay.
                return casadi.reshape(data, (1, data.numel())) # Fallback

        elif axis == 0: # Flatten all dimensions into a single row vector
             return casadi.reshape(data, (1, data.numel()))
        else:
            # Flattening at an arbitrary axis for N-D symbolic tensor is very complex.
            raise NotImplementedError(f"Flatten for axis={axis} on N-D symbolic data is not implemented due to CasADi SX shape complexities.")
        return data # Fallback

    def Unsqueeze(self, data: casadi.SX, axes_tensor: Union[casadi.SX, casadi.DM], attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        # axes_tensor contains the axes to insert new dimensions of size 1
        # This op is hard with symbolic shapes in CasADi SX.
        # We need to construct the new shape tuple.
        logger.warning("Unsqueeze is difficult for symbolic CasADi SX and is likely a pass-through or very limited.")
        # The ONNX spec can take 'axes' as an attribute for older opsets.
        # Let's try to retrieve it from attribute if axes_tensor is not provided by node.input
        actual_axes = None
        if isinstance(axes_tensor, casadi.DM): # If axes is a constant initializer
            actual_axes = [int(a) for a in axes_tensor.full().flatten()]
        elif attribute: # Try to get from attributes if node.input for axes was empty
             attrs = self._parse_attributes(attribute)
             if 'axes' in attrs:
                 actual_axes = list(attrs['axes'])

        if actual_axes is None:
            logger.error("Unsqueeze called without 'axes' information (neither as input tensor nor attribute).")
            return data # Pass-through

        # Cannot easily get current rank or shapes if 'data' is SX without known sparsity.
        # This is a very simplified placeholder.
        # If we assume data is a vector (N,) and axes=[0], new_shape = (1,N)
        # If data is a vector (N,) and axes=[1], new_shape = (N,1)
        # This requires knowing original shape to insert 1s.
        # For now, this is a placeholder like in the original snippet.
        return data


    def Squeeze(self, data: casadi.SX, axes_tensor: Optional[Union[casadi.SX, casadi.DM]] = None, attribute: List[onnx.AttributeProto] = None) -> casadi.SX:
        # Removes dimensions of size 1.
        # If axes is given, only those. Otherwise all.
        logger.warning("Squeeze is difficult for symbolic CasADi SX and is likely a pass-through or very limited.")
        # Placeholder
        return data

    # Add other ONNX operations as needed...