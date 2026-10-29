#!/usr/bin/env python3
"""Generate centered pedsim XML and a deterministic PCD from blank_world walls."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

MAX_PEDESTRIANS = 5


def load_blank_scene(name, config=None):
    config = Path(config) if config else Path(__file__).resolve().parents[1]/'config/pedsim/blank_scenarios.yaml'
    scenes = yaml.safe_load(config.read_text())
    if name not in scenes:
        raise ValueError('unknown blank scenario: '+name)
    source = scenes[name]
    people = source.get('pedestrians', [source])
    if not isinstance(people, list) or not 1 <= len(people) <= MAX_PEDESTRIANS:
        raise ValueError('blank scenario requires 1..5 pedestrians')
    scene = dict(robot_start=source['robot_start'], robot_goal=source['robot_goal'], pedestrians=[])
    def point(value):
        if not isinstance(value, list) or len(value) != 2 or any(
                isinstance(v, bool) or not isinstance(v, (float, int)) or
                not math.isfinite(v) or abs(v) >= 14.5 for v in value):
            raise ValueError('Endpoint must have at least 0.5m arena clearance')
        return list(value)
    scene['robot_start'] = point(scene['robot_start'])
    scene['robot_goal'] = point(scene['robot_goal'])
    for person in people:
        speed = person['desired_max_speed']
        if (person.get('initial_speed', 0) != 0 or isinstance(speed, bool) or
                not isinstance(speed, (float, int)) or not math.isfinite(speed) or not 0 < speed <= 3):
            raise ValueError('unsupported pedestrian speed')
        scene['pedestrians'].append(dict(start=point(person['start']), goal=point(person['goal']),
                                         initial_speed=0., desired_max_speed=float(speed)))
    return scene


def centered_walls(template):
    root = ET.parse(template).getroot()
    walls = np.array([[float(w.attrib[k]) for k in ('x1', 'y1', 'x2', 'y2')]
                      for w in root.findall('obstacle')])
    # This exporter deliberately handles only the official rectangular blank map.
    expected = {(0., 0., 30., 0.), (0., 0., 0., 30.),
                (0., 30., 30., 30.), (30., 30., 30., 0.)}
    if root.tag != 'scenario' or set(map(tuple, walls)) != expected:
        raise ValueError('Expected the four 30x30m walls from upstream blank_world.xml')
    return walls - 15.


def write_pcd(path, walls):
    # Wall centerlines match pedsim. A 0.10m thickness conservatively represents
    # them in the planner; sampling is deterministic, with no sensor noise/FOV.
    chunks = []
    z = np.arange(1, 11) * .2
    for x1, y1, x2, y2 in walls:
        start = np.array([x1, y1]); delta = np.array([x2-x1, y2-y1])
        length = np.linalg.norm(delta)
        line = start + np.linspace(0., 1., int(np.ceil(length/.05))+1)[:, None]*delta
        normal = np.array([-delta[1], delta[0]]) / length
        xy = (line[:, None, :] + np.array([-.05, 0., .05])[None, :, None]*normal).reshape(-1, 2)
        chunks.append(np.column_stack((np.repeat(xy, len(z), axis=0), np.tile(z, len(xy)))))
    # Floor supplies known traversable coverage for the optional static map.
    # Its height is below the obstacle filters on both mapping paths.
    floor = np.arange(-150, 150)*.1 + .05
    x, y = np.meshgrid(floor, floor)
    chunks.append(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.1))))
    points = np.vstack(chunks)
    with Path(path).open('w') as handle:
        handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                     'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                     f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                     f'POINTS {len(points)}\nDATA ascii\n')
        np.savetxt(handle, points, fmt='%.5f')
    return len(points)


def export_world(template, directory, scenario):
    scene = load_blank_scene(scenario)
    walls = centered_walls(template)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element('scenario')
    for wall in walls:
        ET.SubElement(root, 'obstacle', dict(zip(('x1', 'y1', 'x2', 'y2'), map(str, wall))))
    for i, pedestrian in enumerate(scene['pedestrians']):
        goal_id = 'ped_goal_%d' % i
        ET.SubElement(root, 'waypoint', id=goal_id, x=str(pedestrian['goal'][0]), y=str(pedestrian['goal'][1]), r='0.2')
        person = ET.SubElement(root, 'agent', x=str(pedestrian['start'][0]), y=str(pedestrian['start'][1]),
                               n='1', dx='0', dy='0', type='0')
        ET.SubElement(person, 'addwaypoint', id=goal_id)
    ET.SubElement(root, 'agent', x=str(scene['robot_start'][0]), y=str(scene['robot_start'][1]),
                  n='1', dx='0', dy='0', type='2')
    ET.ElementTree(root).write(directory/'world.xml', encoding='utf-8', xml_declaration=True)
    count = write_pcd(directory/'world.pcd', walls)
    metadata = dict(world='blank', scenario=scenario, scene=scene,
                    upstream_template_sha256=hashlib.sha256(Path(template).read_bytes()).hexdigest(),
                    coordinate_offset=[-15., -15.], wall_thickness=.1, wall_height=2.,
                    floor_height=-.1, map_size=[40., 40., 5.], points=count)
    (directory/'world.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('template', help='Upstream pedsim_simulator/scenarios/blank_world.xml')
    parser.add_argument('directory', help='Output directory for world.xml, world.pcd and world.json')
    parser.add_argument('--scenario', default='headon')
    args = parser.parse_args()
    print(json.dumps(export_world(args.template, args.directory, args.scenario), indent=2))


if __name__ == '__main__':
    main()
