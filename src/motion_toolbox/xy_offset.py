"""Geometric upright frames along an ordered XY line; no feasibility planning."""
import numpy as np


def centerline_offset_frames(centerline, mapped_points, target_x_axes, target_y_axes,
                             x_offset=-.9, y_offset=1.2):
    """Use the spatial centerline's normal/tangent, independent of travel direction.

    mapped_points are the per-target points from centerline_xy, in target order.
    Tangents interpolate arc-length derivatives at the nearest centerline segment
    (a smooth heading approximation to the exported polyline). One global sign is selected using the mean alignment with target
    Z projections: base +X faces the workpiece; -X moves away; +Y is tangential.
    No per-target sign flips are allowed, even when traversal reverses. Ambiguous
    wall-side evidence is rejected. This is geometry only, with no robot checks.
    """
    curve = np.asarray(centerline, dtype=float)
    mapped = np.asarray(mapped_points, dtype=float)
    if curve.ndim != 2 or curve.shape[1] not in (2, 3) or len(curve) < 2:
        raise ValueError('Provide a centerline with at least two XY/XYZ points')
    if mapped.ndim != 2 or mapped.shape[1] not in (2, 3) or not len(mapped):
        raise ValueError('Provide nonempty per-target XY/XYZ points')
    if not np.isfinite(curve).all() or not np.isfinite(mapped).all():
        raise ValueError('Positions must be finite')
    # Validate the reference axes and obtain their horizontal unit normals.
    reference = offset_frames(mapped, target_x_axes, target_y_axes, 0., 0.)
    delta = np.diff(curve[:, :2], axis=0)
    squared = np.sum(delta*delta, axis=1)
    if np.any(squared <= 0.):
        raise ValueError('Centerline contains zero-length segments')
    arc = np.concatenate(([0.], np.cumsum(np.sqrt(squared))))
    derivatives = np.gradient(curve[:, :2], arc, axis=0)
    # Nearest-segment projection avoids assuming that the curve is straight.
    tangent = []
    for point in mapped[:, :2]:
        u = np.clip(np.sum((point-curve[:-1, :2])*delta, axis=1)/squared, 0., 1.)
        distances = np.sum((curve[:-1, :2]+u[:, None]*delta-point)**2, axis=1)
        index = int(np.argmin(distances))
        direction = (1.-u[index])*derivatives[index]+u[index]*derivatives[index+1]
        length = np.linalg.norm(direction)
        if length <= 1e-12:
            raise ValueError('Undefined centerline tangent at mapped point {}'.format(len(tangent)))
        tangent.append(direction/length)
    tangent = np.asarray(tangent)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0], np.zeros(len(mapped))))
    alignment = float(np.mean(np.sum(normal*reference['x_axes'], axis=1)))
    if abs(alignment) < .1:
        raise ValueError('Ambiguous wall side: mean target-normal alignment {:.6g}'.format(alignment))
    normal *= 1. if alignment > 0 else -1.
    up = np.tile([0., 0., 1.], (len(mapped), 1))
    sideways = np.cross(up, normal)
    # sideways cross up = normal, using the existing upright-frame constructor.
    frames = offset_frames(mapped, sideways, up, x_offset, y_offset)
    frames['mean_target_normal_alignment'] = abs(alignment)
    return frames


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
