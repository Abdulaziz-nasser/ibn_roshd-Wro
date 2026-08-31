#!/usr/bin/env python3
"""Read yaw and verify sign without moving motors."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.serial_link import SerialLink
parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config); link=SerialLink(cfg['serial']); link.start(); time.sleep(1); link.send('ZERO_YAW')
print('Rotate the robot by hand. In this project, physical RIGHT should make yaw positive; LEFT should make it negative.')
try:
    while True:
        link.tick(); t=link.telemetry(); print(f'yaw={t.yaw:8.2f}',end='\r',flush=True); time.sleep(0.05)
except KeyboardInterrupt: print()
finally: link.stop()
