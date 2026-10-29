#!/usr/bin/env python3
"""Portable scene files reject invalid edits before live replacement."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from cane_pedestrian import social_scene as scene
from export_pedsim_blank import load_blank_scene


class SocialScenes(unittest.TestCase):
    def setUp(self):self.config=scene.from_startup(load_blank_scene('crowd_5'))

    def test_all_presets_and_round_trip(self):
        for name in ('headon','crossing','overtake','crowd_2','crowd_5'):
            value=scene.from_startup(load_blank_scene(name))
            self.assertEqual(value,scene.loads(scene.dumps(value)))
        self.config['pedestrians']=[]
        self.assertEqual([],scene.loads(scene.dumps(self.config))['pedestrians'])

    def test_frames_counts_ids_and_overlap(self):
        for field,value in [('frame_id','map'),('world','office'),('format','legacy'),('start_on_goal',1)]:
            candidate=copy.deepcopy(self.config);candidate[field]=value
            with self.assertRaises(ValueError):scene.validate(candidate)
        for change in ('count','id','overlap'):
            candidate=copy.deepcopy(self.config);people=candidate['pedestrians']
            if change=='count':people.append(people[0])
            if change=='id':people[1]['id']=people[0]['id']
            if change=='overlap':people[1]['start']=people[0]['start'][:]
            with self.assertRaises(ValueError):scene.validate(candidate)

    def test_numbers_and_behavior(self):
        for value in (float('nan'),float('inf'),True,-.1,3.1,'fast'):
            candidate=copy.deepcopy(self.config);candidate['pedestrians'][0]['desired_speed']=value
            with self.assertRaises(ValueError):scene.validate(candidate)
        for value in (None,'disappear',7):
            candidate=copy.deepcopy(self.config);candidate['pedestrians'][0]['end_behavior']=value
            with self.assertRaises(ValueError):scene.validate(candidate)
        candidate=copy.deepcopy(self.config);candidate['pedestrians'][0]['desired_speed']=0
        scene.validate(candidate)

    def test_yaml_malformed_oversize_and_incomplete(self):
        for value in ('[broken', 'a'*65537, '!!python/object:foo {}', 'pedestrians: []'):
            with self.assertRaises(ValueError):scene.loads(value)
        for value in (None,[],{},dict(id=1)):
            with self.assertRaises(ValueError):scene.person(value)


if __name__=='__main__':unittest.main()
