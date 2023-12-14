from typing import Union, Any, Dict, List
import numpy as np

def omit_convert_dict(x: Union[Any, Dict]):
    if not isinstance(x, dict):
        return type(x)
    omit_print_res = dict()
    for k, v in x.items():
        omit_print_res[k] = omit_convert_dict(v)
    return omit_print_res

def convert_ndarray_in_dict_to_shape(x: dict):
    if isinstance(x, np.ndarray):
        return x.shape
    if isinstance(x, dict):
        res = dict()
        for k, v in x.items():
            res[k] = convert_ndarray_in_dict_to_shape(v)
        return res
    if isinstance(x, tuple) or isinstance(x, list):
        res = list()
        for y in x:
            res.append(convert_ndarray_in_dict_to_shape(y))
        if isinstance(x, tuple):
            res = tuple(res)
        return res
    return x