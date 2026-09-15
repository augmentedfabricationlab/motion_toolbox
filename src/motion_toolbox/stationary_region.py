"""Constrained stationary printing region; all lengths are metres.

Target +Z points away from the robot. The calibrated arm-base origin must be
strictly behind every target and within max_distance of every target origin.
The footprint is recovered from that arm origin for each sampled base heading.
"""
from motion_toolbox.recording import recorded
import math
import numpy as np
from .geometry import Plane, as_plane


def _clip(polygon, normal, bound):
    """Intersect a convex polygon with normal.dot(x) <= bound."""
    if len(polygon) == 0:
        return polygon
    values = polygon @ normal - bound
    inside = values <= 0
    if inside.all():
        return polygon
    if not inside.any():
        return np.empty((0, 2))
    output = []
    for i, point in enumerate(polygon):
        previous = polygon[i-1]
        if inside[i] != inside[i-1]:
            fraction = values[i-1] / (values[i-1]-values[i])
            output.append(previous + fraction*(point-previous))
        if inside[i]:
            output.append(point)
    return np.asarray(output)


class StationaryRegion:
    """Exact candidate constraints and a conservative polygon for generating them.

    The 128-sided inscribed reach disks lose at most 0.53 mm radially for a
    1.75 m radius. An empty sampled region is not proof of continuous infeasibility.
    No robot/body clearance is inferred from target planes: use collision meshes.
    """
    @recorded
    def __init__(self, targets, arm_in_base, *, max_distance=1.75, base_height=0.0, projected=False):
        self.targets = [as_plane(t) for t in targets]
        if not self.targets:
            raise ValueError('At least one target required')
        if not math.isfinite(max_distance) or max_distance <= 0 or not math.isfinite(base_height):
            raise ValueError('Positive finite maximum distance and finite base height required')
        self.mount = as_plane(arm_in_base)
        self.max_distance = max_distance
        self.base_height = base_height
        self.height = base_height + self.mount.origin[2]
        self.points = np.array([t.origin for t in self.targets])
        self.normals = np.array([t.zaxis for t in self.targets])
        self.projected = projected
        self.horizontal_norms = np.linalg.norm(self.normals[:, :2], axis=1)
        self.center = self.points[:, :2].mean(axis=0)
        self.xy = self.points[:, :2]-self.center
        # One micrometre excludes the trivial on-plane origin without imposing
        # a made-up minimum physical clearance for the robot body.
        self.side_epsilon = 1e-6

    def metrics(self, base):
        base = as_plane(base)
        arm = (base.matrix @ self.mount.matrix)[:3, 3]
        delta = arm-self.points
        distances = np.linalg.norm(delta, axis=1)
        projected_distances = np.linalg.norm(delta[:, :2], axis=1)
        behind = -np.einsum('ij,ij->i', delta, self.normals)
        if self.projected:
            horizontal = self.horizontal_norms > 1e-9
            behind[horizontal] = -np.einsum('ij,ij->i', delta[horizontal, :2],
                self.normals[horizontal, :2])/self.horizontal_norms[horizontal]
        wrong_side = np.flatnonzero(behind < self.side_epsilon-1e-9).tolist()
        too_far = np.flatnonzero((projected_distances if self.projected else distances) > self.max_distance+1e-9).tolist()
        standoff = behind[self.horizontal_norms > 1e-9] if self.projected else behind
        return dict(arm_origin=arm.tolist(), target_distances=distances.tolist(),
            standoff=float(standoff.min()) if len(standoff) else 0.0,
            max_target_distance=float(distances.max()), max_projected_distance=float(projected_distances.max()),
            wrong_side_points=wrong_side, too_far_points=too_far,
            geometry_valid=not wrong_side and not too_far)

    def _behind(self, polygon, distance):
        if self.projected:
            for normal, norm, point, target in zip(self.normals, self.horizontal_norms, self.xy, self.points):
                if norm <= 1e-9:
                    if normal[2]*(self.height-target[2]) >= -self.side_epsilon:
                        return np.empty((0, 2))
                    continue
                unit = normal[:2]/norm
                polygon = _clip(polygon, unit, unit @ point-distance)
                if not len(polygon):
                    break
            return polygon
        bounds = np.einsum('ij,ij->i', self.normals[:, :2], self.xy)
        bounds += self.normals[:, 2]*(self.points[:, 2]-self.height)-distance
        for normal, bound in zip(self.normals[:, :2], bounds):
            polygon = _clip(polygon, normal, bound)
            if not len(polygon):
                break
        return polygon

    @recorded
    def polygon(self):
        radii_squared = np.full(len(self.points), self.max_distance**2) if self.projected else self.max_distance**2-(self.points[:, 2]-self.height)**2
        if np.any(radii_squared < 0):
            return np.empty((0, 2)), 'Target height exceeds {} m reach limit at this arm-base height.'.format(self.max_distance)
        radii = np.sqrt(radii_squared)
        lower = np.max(self.xy-radii[:, None], axis=0)
        upper = np.min(self.xy+radii[:, None], axis=0)
        if np.any(lower > upper):
            return np.empty((0, 2)), 'Targets have no common reach bounding box at this arm-base height.'
        polygon = np.array([lower, [upper[0], lower[1]], upper, [lower[0], upper[1]]])
        polygon = self._behind(polygon, self.side_epsilon)
        if not len(polygon):
            return polygon, 'No common negative-Z region within the reach bounds. Check target normals.'
        sides = 128
        angles = (np.arange(sides)+.5)*2*math.pi/sides
        normals = np.column_stack((np.cos(angles), np.sin(angles)))
        for point, radius in zip(self.xy, radii):
            inradius = radius*math.cos(math.pi/sides)
            if np.all(np.linalg.norm(polygon-point, axis=1) <= inradius):
                continue
            for normal in normals:
                polygon = _clip(polygon, normal, normal @ point + inradius)
                if not len(polygon):
                    return polygon, 'No common sampled region satisfies every target side and distance constraint.'
        return polygon, ''

    @recorded
    def candidates(self, *, spacing=.5, yaw_steps=4):
        """Return (all footprints, initial guesses, region explanation).

        Seed by maximizing the minimum signed distance behind all target planes.
        Also sample inward positions, the region boundary and its XY interior.
        Every proposed footprint is rechecked against the exact 3D constraints.
        """
        if not math.isfinite(spacing) or spacing <= 0:
            raise ValueError('Positive finite grid spacing required')
        if int(yaw_steps) != yaw_steps or yaw_steps < 1:
            raise ValueError('yaw_steps must be a positive integer')
        polygon, reason = self.polygon()
        if not len(polygon):
            return [], [], reason
        lo, hi = self.side_epsilon, self.max_distance
        farthest = polygon
        for _ in range(32):
            middle = (lo+hi)/2
            clipped = self._behind(polygon, middle)
            if len(clipped):
                lo, farthest = middle, clipped
            else:
                hi = middle
        farthest = farthest.mean(axis=0)
        middle = polygon.mean(axis=0)
        seed_points = [farthest*(1-t)+middle*t for t in (0, .2, .4, .7, 1)]
        # Sample the boundary by arc length so vertex density does not bias it.
        closed = np.vstack((polygon, polygon[0]))
        lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        cumulative = np.concatenate(([0], np.cumsum(lengths)))
        boundary = []
        if cumulative[-1] > 1e-12:
            for value in np.linspace(0, cumulative[-1], 16, endpoint=False):
                i = min(np.searchsorted(cumulative, value, side='right')-1, len(polygon)-1)
                t = (value-cumulative[i])/max(lengths[i], 1e-12)
                boundary.append(closed[i]*(1-t)+closed[i+1]*t)
        lower, upper = polygon.min(axis=0), polygon.max(axis=0)
        axes = [np.linspace(a, b, int(math.ceil((b-a)/spacing))+1) for a,b in zip(lower, upper)]
        samples = seed_points + boundary + [np.array((x,y)) for x in axes[0] for y in axes[1]]
        result, guesses, seen = [], [], set()
        for i, point in enumerate(samples):
            arm_xy = point+self.center
            toward = -point
            facing = math.atan2(toward[1], toward[0]) if np.linalg.norm(toward) > 1e-8 else 0.0
            facing -= math.atan2(self.mount.xaxis[1], self.mount.xaxis[0])
            for yaw in facing + np.arange(int(yaw_steps))*2*math.pi/yaw_steps:
                c, s = math.cos(yaw), math.sin(yaw)
                offset = np.array([c*self.mount.origin[0]-s*self.mount.origin[1],
                                   s*self.mount.origin[0]+c*self.mount.origin[1]])
                base = Plane((*(arm_xy-offset), self.base_height), (c,s,0), (-s,c,0))
                key = tuple(np.round(base.matrix.ravel(), 9))
                if key not in seen and self.metrics(base)['geometry_valid']:
                    seen.add(key)
                    result.append(base)
                    if i < len(seed_points):
                        guesses.append(base)
        result.sort(key=lambda base: -self.metrics(base)['standoff'])
        return result, guesses, '' if result else 'No sampled footprint satisfies the exact side and distance constraints.'
