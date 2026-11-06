"""Independent geometric checks for recorded simulator motion and planned paths.

Reference boxes belong to the evaluator only. Runtime nodes never import this module.
"""
import math


def distance_to_box(x: float, y: float, box: dict) -> float:
    """Return distance from a point to a closed, possibly rotated collision rectangle."""
    dx = x - box['x']
    dy = y - box['y']
    local_x = dx * math.cos(box['yaw']) + dy * math.sin(box['yaw'])
    local_y = -dx * math.sin(box['yaw']) + dy * math.cos(box['yaw'])
    outside_x = max(0.0, abs(local_x) - box['sx'] / 2)
    outside_y = max(0.0, abs(local_y) - box['sy'] / 2)
    return math.hypot(outside_x, outside_y)


def segment_is_clear(start: tuple, end: tuple, boxes: list[dict], margin: float) -> bool:
    """Sample an external reference segment at 2 cm to detect blocked direct routes."""
    length = math.dist(start, end)
    steps = max(1, math.ceil(length / 0.02))
    for step in range(steps + 1):
        fraction = step / steps
        x = start[0] + fraction * (end[0] - start[0])
        y = start[1] + fraction * (end[1] - start[1])
        if any(distance_to_box(x, y, box) < margin for box in boxes):
            return False
    return True


def path_has_detour(path: list[tuple], boxes: list[dict]) -> bool:
    """Require a clear non-straight planned route whose direct chord crosses an obstacle."""
    if len(path) < 3 or math.dist(path[0], path[-1]) < 0.5:
        return False
    if segment_is_clear(path[0], path[-1], boxes, 0.12):
        return False
    for start, end in zip(path, path[1:]):
        if not segment_is_clear(start, end, boxes, 0.12):
            return False
    length = sum(math.dist(start, end) for start, end in zip(path, path[1:]))
    return length > 1.05 * math.dist(path[0], path[-1])
