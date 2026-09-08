#!/usr/bin/env python3
"""Offline office-world projection. Requires existing PCL converter, not ROS runtime."""
import argparse
import yaml
import math
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path
from collections import deque
import numpy as np


def boundaries(cells):
    groups = {}
    for x, y in cells:
        for neighbor, key, coordinate in [((x-1,y),('v',x),y),((x+1,y),('v',x+1),y),
                                          ((x,y-1),('h',y),x),((x,y+1),('h',y+1),x)]:
            if neighbor not in cells:
                groups.setdefault(key, []).append(coordinate)
    segments = []
    for (axis, fixed), values in sorted(groups.items()):
        values = sorted(set(values)); start = end = values[0]
        for value in values[1:] + [None]:
            if value == end + 1:
                end = value
                continue
            segments.append((fixed,start,fixed,end+1) if axis == 'v' else (start,fixed,end+1,fixed))
            start = end = value
    return segments


def load_scene(name, config=None):
    config = Path(config) if config else Path(__file__).resolve().parents[1]/'config/pedsim/scenarios.yaml'
    scenes=yaml.safe_load(config.read_text())
    if name not in ('headon','overtake','crossing') or name not in scenes: raise ValueError('unknown pedsim scenario: '+name)
    scene=scenes[name]
    for key in ('start','goal'):
        if len(scene[key])!=2 or not all(math.isfinite(v) and abs(v)<100 for v in scene[key]):raise ValueError('invalid '+key)
    if scene['initial_speed']!=0 or not math.isfinite(scene['desired_max_speed']) or not 0<scene['desired_max_speed']<=3:raise ValueError('unsupported speed')
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd'); parser.add_argument('output')
    parser.add_argument('--scenario',default='headon')
    args = parser.parse_args()
    scene=load_scene(args.scenario)
    with tempfile.TemporaryDirectory() as tmp:
        ascii_file = Path(tmp)/'map.pcd'
        subprocess.run(['pcl_convert_pcd_ascii_binary',args.pcd,str(ascii_file),'0'],check=True,stdout=subprocess.DEVNULL)
        with ascii_file.open() as f:
            fields = None
            while True:
                line = f.readline()
                if not line: raise ValueError('missing PCD DATA')
                if line.startswith('FIELDS '): fields = line.split()[1:]
                if line.startswith('DATA '): break
            data = np.loadtxt(f)
        xyz = data[:,[fields.index(k) for k in ('x','y','z')]]
    if not np.isfinite(xyz).all(): raise ValueError('nonfinite world points')
    selected = xyz[(xyz[:,2]>=.45)&(xyz[:,2]<=1.8)]
    if not len(selected): raise ValueError('empty body-height band')
    cells = set(map(tuple,np.floor(selected[:,:2]/.1).astype(int)))
    lower = np.floor(xyz[:,:2].min(0)/.1).astype(int)-1
    upper = np.ceil(xyz[:,:2].max(0)/.1).astype(int)+1
    # Verify centreline connectivity with conservative 0.4m centre clearance.
    blocked = set()
    for x,y in cells:
        for dx in range(-4,5):
            for dy in range(-4,5): blocked.add((x+dx,y+dy))
    start=tuple(np.floor(np.array(scene['start'])/.1).astype(int)); goal=tuple(np.floor(np.array(scene['goal'])/.1).astype(int))
    for q in (start,goal):
        if not (lower[0]<q[0]<upper[0] and lower[1]<q[1]<upper[1]):raise ValueError('endpoint outside map')
    if start in blocked or goal in blocked: raise ValueError('pedestrian endpoint blocked')
    seen={start}; queue=deque([start])
    while queue:
        x,y=queue.popleft()
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            q=(x+dx,y+dy)
            if lower[0]<q[0]<upper[0] and lower[1]<q[1]<upper[1] and q not in blocked and q not in seen:
                seen.add(q);queue.append(q)
    if goal not in seen: raise ValueError('pedestrian endpoints disconnected')
    lines=boundaries(cells)
    out=Path(args.output)
    xml=['<?xml version="1.0"?><scenario>']
    for a,b,c,d in lines: xml.append('<obstacle x1="%.8f" y1="%.8f" x2="%.8f" y2="%.8f"/>'%(a*.1,b*.1,c*.1,d*.1))
    xml += ['<waypoint id="ped_goal" x="%s" y="%s" r="0.2"/>'%tuple(scene['goal']),
            '<agent x="%s" y="%s" n="1" dx="0" dy="0" type="0"><addwaypoint id="ped_goal"/></agent>'%tuple(scene['start']),
            '<agent x="0" y="0" n="1" dx="0" dy="0" type="2"/>','</scenario>']
    out.write_text('\n'.join(xml)+'\n')
    print(json.dumps(dict(scenario=args.scenario,scene=scene,pcd_sha256=hashlib.sha256(Path(args.pcd).read_bytes()).hexdigest(),
                         points=len(xyz),selected=len(selected),occupied_cells=len(cells),segments=len(lines),
                         minimum=xyz.min(0).tolist(),maximum=xyz.max(0).tolist(),resolution=.1,
                         height_band=[.45,1.8],connected_with_square_clearance=.4),indent=2))

if __name__=='__main__': main()
