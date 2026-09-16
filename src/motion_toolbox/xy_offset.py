"""Geometric upright frames along an ordered XY line; no feasibility planning."""
import numpy as np


def offset_frames(line_points, target_x_axes, target_y_axes, x_offset=-.9, y_offset=1.2):
    """One right-handed ground frame per target, in input units and order.

    Frame X is the normalized world-XY projection of target Z = target X cross Y.
    Frame Z is world up, so frame Y = world Z cross frame X. Offset is measured
    from the corresponding smooth-line point, not from the original TCP origin.
    Vertical target normals have no defined horizontal heading and are rejected.
    This construction does not check robot reach, collisions or motion limits.
    """
    points = np.asarray(line_points, dtype=float)
    tx, ty = np.asarray(target_x_axes, dtype=float), np.asarray(target_y_axes, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or not len(points):
        raise ValueError('Provide a nonempty XY or XYZ line')
    if tx.shape != (len(points), 3) or ty.shape != tx.shape:
        raise ValueError('Provide one target X and Y axis per line point')
    if not all(np.isfinite(a).all() for a in (points, tx, ty, [x_offset, y_offset])):
        raise ValueError('Geometry and offsets must be finite')
    normals = np.cross(tx, ty)
    norm = np.linalg.norm(normals, axis=1)
    horizontal = np.linalg.norm(normals[:, :2], axis=1)
    invalid = np.flatnonzero((norm == 0.) | (horizontal <= 1e-10*norm))
    if len(invalid):
        raise ValueError('Undefined horizontal target normal at target indices {}'.format(invalid.tolist()))
    x = np.column_stack((normals[:, :2]/horizontal[:, None], np.zeros(len(points))))
    z = np.tile([0., 0., 1.], (len(points), 1))
    y = np.cross(z, x)
    original = np.column_stack((points[:, :2], np.zeros(len(points))))
    origins = original+float(x_offset)*x+float(y_offset)*y
    return dict(origins=origins, x_axes=x, y_axes=y, z_axes=z,
                line_origins=original, x_offset=float(x_offset), y_offset=float(y_offset))
