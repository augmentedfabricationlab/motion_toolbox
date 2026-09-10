"""Metre/radian geometry. Rhino and COMPAS are converted only at boundaries."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True, eq=False)
class Plane:
    origin: object
    xaxis: object
    yaxis: object

    def __post_init__(self):
        o, x, y = (np.array(v, dtype=float, copy=True) for v in (self.origin, self.xaxis, self.yaxis))
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (o, x, y)):
            raise ValueError('Plane requires three finite 3-vectors')
        if np.linalg.norm(x) < 1e-12:
            raise ValueError('Zero x axis')
        x /= np.linalg.norm(x)
        y -= np.dot(x, y) * x
        if np.linalg.norm(y) < 1e-12:
            raise ValueError('Plane axes are parallel or zero')
        y /= np.linalg.norm(y)
        for name, v in [('origin', o), ('xaxis', x), ('yaxis', y), ('zaxis', np.cross(x, y))]:
            v.setflags(write=False)
            object.__setattr__(self, name, v)

    @classmethod
    def world_xy(cls):
        return cls((0, 0, 0), (1, 0, 0), (0, 1, 0))

    @property
    def matrix(self):
        T = np.eye(4)
        T[:3, :3] = np.column_stack((self.xaxis, self.yaxis, self.zaxis))
        T[:3, 3] = self.origin
        return T

    @classmethod
    def from_matrix(cls, T):
        T = np.asarray(T, dtype=float)
        if T.shape != (4, 4) or not np.isfinite(T).all() or not np.allclose(T[3], [0, 0, 0, 1]):
            raise ValueError('Expected a finite homogeneous 4x4 transform')
        if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-7) or np.linalg.det(T[:3, :3]) < 0:
            raise ValueError('Transform must contain a proper rigid rotation')
        return cls(T[:3, 3], T[:3, 0], T[:3, 1])

    def rotated_z(self, radians):
        c, s = math.cos(radians), math.sin(radians)
        return Plane(self.origin, c*self.xaxis+s*self.yaxis, -s*self.xaxis+c*self.yaxis)

    def to_dict(self):
        return dict(origin=self.origin.tolist(), x_axis=self.xaxis.tolist(), y_axis=self.yaxis.tolist())


def rigid_inverse(T):
    R = np.asarray(T)[:3, :3]
    result = np.eye(4)
    result[:3, :3] = R.T
    result[:3, 3] = -R.T @ np.asarray(T)[:3, 3]
    return result


def _xyz(value):
    if hasattr(value, 'X'):
        return [value.X, value.Y, value.Z]
    return list(value)


def as_plane(value, scale=1.0):
    """Accept numeric Plane, JSON dict, COMPAS Frame or Rhino/rhino3dm Plane.

    scale converts incoming length units to metres; axes are dimensionless.
    """
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('scale must be positive')
    if isinstance(value, Plane):
        return value if scale == 1 else Plane(value.origin*scale, value.xaxis, value.yaxis)
    if isinstance(value, dict):
        o = value.get('origin', value.get('point'))
        x = value.get('x_axis', value.get('xaxis'))
        y = value.get('y_axis', value.get('yaxis'))
    elif hasattr(value, 'Origin'):
        o, x, y = value.Origin, value.XAxis, value.YAxis
    elif hasattr(value, 'point'):
        o, x, y = value.point, value.xaxis, value.yaxis
    else:
        o, x, y = value
    return Plane(np.asarray(_xyz(o))*scale, _xyz(x), _xyz(y))


def to_rhino(value, scale=1.0):
    import Rhino.Geometry as rg
    p = as_plane(value)
    return rg.Plane(rg.Point3d(*(p.origin*scale)), rg.Vector3d(*p.xaxis), rg.Vector3d(*p.yaxis))


def to_compas(value):
    from compas.geometry import Frame
    p = as_plane(value)
    return Frame(p.origin, p.xaxis, p.yaxis)
