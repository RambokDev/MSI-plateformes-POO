from functools import reduce
from typing import List


def deep_get(_dict: dict, keys: List[str], default=None):
    def _reducer(d, key):
        if isinstance(d, dict):
            return d.get(key, default)
        return default

    return reduce(_reducer, keys, _dict)
