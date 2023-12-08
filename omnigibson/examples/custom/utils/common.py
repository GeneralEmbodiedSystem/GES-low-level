from typing import Union, Any, Dict, List


def omit_convert_dict(x: Union[Any, Dict]):
    if not isinstance(x, dict):
        return type(x)
    omit_print_res = dict()
    for k, v in x.items():
        omit_print_res[k] = omit_convert_dict(v)
    return omit_print_res