"""Joint representative selection and filtering, without robot network access."""
import math
import numpy as np


def snap_angle(theta, reference):
    return float(theta) + round((float(reference)-float(theta))/(2*math.pi))*2*math.pi


def snap_configuration(configuration, reference, periodic=None):
    if len(configuration) != len(reference):
        raise ValueError('Configuration/reference dimensions differ')
    mask = [True]*len(configuration) if periodic is None else list(periodic)
    if len(mask) != len(configuration):
        raise ValueError('Periodic mask dimensions differ')
    return [snap_angle(q, r) if p else float(q) for q, r, p in zip(configuration, reference, mask)]


def edit_solutions(layers, reference, *, tolerance=None, periodic=None):
    """Snap allowed revolutions, then filter every constrained joint exactly once."""
    if tolerance is not None and np.any(np.asarray(tolerance) < 0):
        raise ValueError('Nonnegative tolerance required')
    output = []
    for layer in layers:
        snapped = [snap_configuration(q, reference, periodic) for q in layer]
        output.append([q for q in snapped if tolerance is None or np.all(np.abs(np.array(q)-reference) <= tolerance)])
    return output


def unwrap_configurations(configurations, reference=None, periodic=None):
    result = []
    for q in configurations:
        q = list(q) if reference is None else snap_configuration(q, reference, periodic)
        result.append(q)
        reference = q
    return result
