#!/usr/bin/env python3
from __future__ import annotations
import sys
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_project_config

robot,vision=load_project_config(PROJECT_ROOT/'config'/'robot.yaml',PROJECT_ROOT/'config'/'vision.yaml')
errors=[]
for color in ('BLUE','ORANGE','RED','GREEN'):
    if color not in vision.get('colors',{}): errors.append(f'Missing color {color}')
if robot.get('drive',{}).get('minimum_speed',0)>robot.get('drive',{}).get('speed',0):
    errors.append('drive.minimum_speed must not exceed drive.speed')
if robot.get('mission',{}).get('max_turns',0)<1: errors.append('mission.max_turns must be >=1')
if errors:
    print('CONFIG INVALID'); [print(' -',e) for e in errors]; raise SystemExit(1)
print('CONFIG OK')
print('Robot:',robot['_source_path']); print('Vision:',vision['_source_path'])
