#!/usr/bin/env python3
"""Geometry contract for the shared single-pedestrian blank arena."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from export_pedsim_blank import export_world, load_blank_scene


TEMPLATE = '''<scenario>
<obstacle x1="0" y1="0" x2="30" y2="0"/>
<obstacle x1="0" y1="0" x2="0" y2="30"/>
<obstacle x1="0" y1="30" x2="30" y2="30"/>
<obstacle x1="30" y1="30" x2="30" y2="0"/>
<agent x="14" y="6" n="1" dx="0" dy="0" type="2"/>
</scenario>'''


class BlankWorld(unittest.TestCase):
    def test_shared_geometry_scenarios_and_determinism(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); template = base/'blank.xml'
            template.write_text(TEMPLATE)
            geometry_hashes = set()
            for scenario in ('headon', 'crossing', 'overtake', 'crowd_2', 'crowd_5'):
                out = base/scenario
                metadata = export_world(template, out, scenario)
                root = ET.parse(out/'world.xml').getroot()
                n=len(metadata['scene']['pedestrians'])
                self.assertEqual(['0']*n+['2'], [a.attrib['type'] for a in root.findall('agent')])
                self.assertEqual(['1']*(n+1), [a.attrib['n'] for a in root.findall('agent')])
                self.assertEqual(4, len(root.findall('obstacle')))
                self.assertEqual(n, len(root.findall('waypoint')))
                for agent,person,waypoint in zip(root.findall('agent'),metadata['scene']['pedestrians'],root.findall('waypoint')):
                    self.assertEqual(person['start'],[float(agent.attrib[k]) for k in ('x','y')])
                    self.assertEqual(person['goal'],[float(waypoint.attrib[k]) for k in ('x','y')])
                    self.assertEqual(waypoint.attrib['id'],agent.find('addwaypoint').attrib['id'])
                self.assertEqual([40., 40., 5.], metadata['map_size'])
                geometry_hashes.add(hashlib.sha256((out/'world.pcd').read_bytes()).hexdigest())
            self.assertEqual(1, len(geometry_hashes))
            pcd = base/'crossing/world.pcd'
            with pcd.open() as handle:
                for line in handle:
                    if line.startswith('POINTS '): point_count = int(line.split()[1])
                    if line.startswith('DATA '): break
                xyz = np.loadtxt(handle)
            self.assertEqual(point_count, len(xyz))
            self.assertTrue(np.isfinite(xyz).all())
            wall = xyz[xyz[:, 2] >= .15]
            # Every obstacle point follows a shared wall, including its finite
            # thickness. In particular, no obstacle appears inside the arena.
            wall_distance = np.min(np.abs(np.abs(wall[:, :2])-15.), axis=1)
            self.assertTrue((wall_distance <= .05001).all())
            floor = xyz[xyz[:, 2] < .15]
            self.assertEqual(90000, len(floor))
            self.assertTrue((floor[:, 2] == -.1).all())
            self.assertTrue((np.abs(floor[:, :2]) < 15.).all())

    def test_other_map_is_not_silently_replaced_by_blank(self):
        with tempfile.TemporaryDirectory() as tmp:
            template = Path(tmp)/'different.xml'
            template.write_text(TEMPLATE.replace('x2="30"', 'x2="31"', 1))
            with self.assertRaisesRegex(ValueError, 'blank_world'):
                export_world(template, Path(tmp)/'out', 'headon')
            self.assertFalse((Path(tmp)/'out/world.pcd').exists())

    def test_invalid_multi_scene_rejected_before_export(self):
        import yaml
        scene=load_blank_scene('crowd_5')
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'scenes.yaml'
            for field,value in (('desired_max_speed',0),('desired_max_speed',float('nan')),('start',[15,0])):
                source=yaml.safe_load(yaml.safe_dump(scene));source['pedestrians'][-1][field]=value
                path.write_text(yaml.safe_dump({'bad':source}))
                with self.assertRaises(ValueError):load_blank_scene('bad',path)
            scene['pedestrians'].append(scene['pedestrians'][0])
            path.write_text(yaml.safe_dump({'bad':scene}))
            with self.assertRaisesRegex(ValueError,'1..5'):load_blank_scene('bad',path)


if __name__ == '__main__':
    unittest.main()
