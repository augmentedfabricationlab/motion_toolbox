"""Shared JSON schemas; lengths in metres, angles in radians, indices zero-based."""
from motion_toolbox.recording import recorded
import json
from pathlib import Path
from ..geometry import as_plane


@recorded
def load_planes(path, scale=1.0):
    payload = json.loads(Path(path).read_text(encoding='utf-8'))
    rows = payload['planes'] if isinstance(payload, dict) else payload
    return [as_plane(row, scale) for row in rows]


@recorded
def export_planes(planes, path, *, metadata=None, scale=1.0):
    rows = [as_plane(p, scale).to_dict() for p in planes]
    payload = rows if metadata is None else dict(planes=rows, metadata=metadata, units='metres')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
    return path


def unreachable_nodes(layers):
    return [i for i, layer in enumerate(layers) if len(layer) == 0]


def select_ik_solution(layers, node_index, solution_index=0):
    if not layers:
        return None
    node = layers[max(0, min(int(node_index), len(layers)-1))]
    return list(node[max(0, min(int(solution_index), len(node)-1))]) if node else None


def solutions_to_tree(layers):
    """Grasshopper DataTree {target;solution}; numeric core keeps nested lists."""
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    tree = DataTree[float]()
    for i, layer in enumerate(layers):
        if not layer:
            tree.EnsurePath(GH_Path(i))
        for j, q in enumerate(layer):
            path = GH_Path(i, j)
            for value in q:
                tree.Add(float(value), path)
    return tree
