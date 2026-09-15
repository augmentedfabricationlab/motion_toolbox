"""Configurable analytic UR solver; any callable IK solver can replace this."""
from motion_toolbox.recording import recorded
from ..geometry import Plane, as_plane, rigid_inverse
from .ur import inverse_kinematics
import math

UR20 = (0.2363, -0.8620, -0.7287, 0.201, 0.1593, 0.1543)


class URKinematics:
    revolute_joints = tuple(range(6))

    @recorded
    def __init__(self, parameters=UR20, tool=None, arm_in_base=None):
        self.parameters = tuple(float(v) for v in parameters)
        if len(self.parameters) != 6 or not all(math.isfinite(v) for v in self.parameters):
            raise ValueError('Six UR geometry parameters required')
        if self.parameters[1] == 0 or self.parameters[2] == 0:
            raise ValueError('UR upper-arm and forearm lengths must be nonzero')
        self.tool = as_plane(tool) if tool is not None else Plane.world_xy()
        self.arm_in_base = as_plane(arm_in_base) if arm_in_base is not None else Plane.world_xy()
        self._tcp_to_flange = rigid_inverse(self.tool.matrix)
        self._arm_inverse = rigid_inverse(self.arm_in_base.matrix)

    @recorded(detail=True)
    def __call__(self, target, base):
        # World TCP -> arm flange, with independent footprint and arm frames.
        T = self._arm_inverse @ rigid_inverse(as_plane(base).matrix) @ as_plane(target).matrix @ self._tcp_to_flange
        return inverse_kinematics(Plane.from_matrix(T), self.parameters)
