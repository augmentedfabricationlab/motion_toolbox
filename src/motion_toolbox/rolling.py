"""Transport-independent rolling replanning and execution-buffer bookkeeping."""
from motion_toolbox.recording import recorded
from .planning import calculate_partial_trajectory


def select_buffer_tail_pose(*, exec_index, replan_start_index, replanned_configurations, buffer_size):
    if buffer_size < 1:
        raise ValueError('buffer_size must be positive')
    index = exec_index + buffer_size
    offset = index - replan_start_index
    if 0 <= offset < len(replanned_configurations):
        return index, replanned_configurations[offset]
    return None


class RollingPlanner:
    @recorded
    def __init__(self, targets, *, lookahead=10, buffer_size=2, **planning_options):
        if not isinstance(lookahead, int) or not isinstance(buffer_size, int) or lookahead < buffer_size or buffer_size < 1:
            raise ValueError('lookahead must cover a positive buffer_size')
        self.targets = list(targets)
        self.lookahead, self.buffer_size = lookahead, buffer_size
        self.options = planning_options
        self.buffer = {}
        self.last_exec = -1
        self.last_published = -1

    @recorded
    def update(self, exec_index, current_pose, bases=None):
        """Return newly publishable (global index, joints) pairs.

        Call with executed index -1 to seed. Later updates preserve committed
        buffered targets and append the new tail. bases is one current measured
        base or a complete predicted sequence aligned with the original targets.
        """
        if not isinstance(exec_index, int) or exec_index < self.last_exec or exec_index < -1:
            raise ValueError('Execution progress must be monotonic')
        if exec_index >= len(self.targets):
            raise ValueError('Execution index outside path')
        if exec_index > self.last_published:
            raise ValueError('Execution advanced beyond the published buffer')
        if bases is not None and len(bases) not in (1, len(self.targets)):
            raise ValueError('Provide one measured base or a complete predicted base sequence')
        self.last_exec = exec_index
        self.buffer = {i: q for i, q in self.buffer.items() if i > exec_index}
        start = max(exec_index+1, self.last_published+1)
        if start >= len(self.targets) or start > exec_index+self.buffer_size:
            return []
        reference = self.buffer.get(start-1, current_pose)
        selected_bases = bases if bases is None or len(bases) == 1 else bases[start:]
        options = dict(self.options)
        if bases is not None and len(bases) > 1 and start > 0:
            options['start_base'] = bases[start-1]
        result = calculate_partial_trajectory(reference, self.targets[start:], self.lookahead,
            base_planes=selected_bases, **options)
        if not result['configurations']:
            return []
        count = min(len(result['configurations']), max(0, exec_index+self.buffer_size-start+1))
        updates = [(start+i, q) for i, q in enumerate(result['configurations'][:count])]
        for i, q in updates:
            self.buffer[i] = q
            self.last_published = i
        return updates
