from typing import List


def extend_wait(tl: List[int], node: int, steps: int) -> None:
    if steps > 0:
        tl.extend([node] * steps)


def append_move_path(tl: List[int], path_nodes: List[int]) -> None:
    if len(path_nodes) > 1:
        tl.extend(path_nodes[1:])


def finalize_to_T(tl: List[int], T: int) -> List[int]:
    if len(tl) >= T:
        return tl[:T]
    last = tl[-1]
    return tl + [last] * (T - len(tl))
