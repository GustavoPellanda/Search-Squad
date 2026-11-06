#!/usr/bin/env python3
"""Generate ignored navigation fixtures and an explicit entrance assignment from the test world."""
import argparse
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def world_boxes(path: Path) -> list[dict]:
    """Read static box collision geometry solely for test-map generation and evaluation."""
    result = []
    for model in ET.parse(path).getroot().findall('world/model'):
        if model.findtext('static') != 'true':
            continue
        pose = [float(value) for value in model.findtext('pose', '0 0 0 0 0 0').split()]
        for collision in model.findall('link/collision'):
            size = collision.findtext('geometry/box/size')
            if size is None:
                continue
            dimensions = [float(value) for value in size.split()]
            if pose[2] - dimensions[2] / 2 <= 0.18 <= pose[2] + dimensions[2] / 2:
                result.append({'name': model.attrib['name'], 'x': pose[0], 'y': pose[1],
                               'sx': dimensions[0], 'sy': dimensions[1], 'yaw': pose[5]})
    return result


def entrance(boxes: list[dict], room: str) -> tuple[dict, dict]:
    """Derive a doorway gap from corridor-wall segments; provide no room bounds to the task."""
    side, half = room.split('_')
    walls = sorted((box for box in boxes if box['name'].startswith(side + '_corridor_wall')),
                   key=lambda box: box['x'])
    gaps = [(left['x'] + left['sx'] / 2, right['x'] - right['sx'] / 2)
            for left, right in zip(walls, walls[1:])]
    gap = gaps[0 if half == 'west' else 1]
    x = (gap[0] + gap[1]) / 2
    y = walls[0]['y']
    direction = 1 if side == 'north' else -1
    yaw = direction * math.pi / 2

    def pose(distance: float) -> dict:
        """Build a map-frame pose with a normalized inward-facing quaternion."""
        return {'header': {'frame_id': 'map'}, 'pose': {
            'position': {'x': x, 'y': y + direction * distance, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2),
                            'w': math.cos(yaw / 2)}}}

    task = {'task_id': 'test_' + room, 'approach': pose(-0.55), 'threshold': pose(0.0),
            'entry': pose(0.65), 'doorway_width': gap[1] - gap[0], 'exploration_timeout': 600.0}
    outer_x = next(box['x'] for box in boxes if box['name'] == 'outer_' + half)
    outer_y = next(box['y'] for box in boxes if box['name'] == 'outer_' + side)
    reference = {'room': room, 'x_min': min(0.0, outer_x), 'x_max': max(0.0, outer_x),
                 'y_min': min(y, outer_y), 'y_max': max(y, outer_y), 'boxes': boxes}
    return task, reference


def generate(root: Path, output: Path, room: str) -> None:
    """Write fixtures only outside Git candidates; generated truth never enters the controller."""
    output = output.resolve()
    if output.is_relative_to(root):
        ignored = subprocess.run(['git', 'check-ignore', str(output / 'test_map.yaml')],
                                 cwd=root, capture_output=True)
        if ignored.returncode != 0:
            raise ValueError('Fixture output inside the repository must be ignored by Git')
    output.mkdir(parents=True, exist_ok=True)
    boxes = world_boxes(root / 'src/simulation/worlds/four_rooms.world')
    resolution = 0.05
    columns = -8.0 + (np.arange(320) + 0.5) * resolution
    rows = -6.0 + (np.arange(240) + 0.5) * resolution
    x, y = np.meshgrid(columns, rows)
    pixels = np.full(x.shape, 254, dtype=np.uint8)
    for box in boxes:
        dx, dy = x - box['x'], y - box['y']
        local_x = dx * math.cos(box['yaw']) + dy * math.sin(box['yaw'])
        local_y = -dx * math.sin(box['yaw']) + dy * math.cos(box['yaw'])
        pixels[(abs(local_x) <= box['sx'] / 2) & (abs(local_y) <= box['sy'] / 2)] = 0
    (output / 'test_map.pgm').write_bytes(b'P5\n320 240\n255\n' + np.flipud(pixels).tobytes())
    metadata = {'image': 'test_map.pgm', 'resolution': resolution, 'origin': [-8.0, -6.0, 0.0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    (output / 'test_map.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    task, reference = entrance(boxes, room)
    (output / 'room_task.yaml').write_text(yaml.safe_dump(task, sort_keys=False))
    (output / 'reference.yaml').write_text(yaml.safe_dump(reference, sort_keys=False))
    print('TEST FIXTURES ONLY:', output)
    print('Room assignment:', output / 'room_task.yaml')
    print('The normal room launch loads no test_map.pgm or reference.yaml.')


def main() -> None:
    """Select a room in the current project world and generate reproducible test resources."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('artifacts/scenario'))
    parser.add_argument('--room', choices=['north_west', 'north_east', 'south_west', 'south_east'],
                        default='north_west')
    args = parser.parse_args()
    generate(Path(__file__).resolve().parents[1], args.output, args.room)


if __name__ == '__main__':
    main()
