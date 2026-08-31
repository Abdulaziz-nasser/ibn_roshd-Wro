#!/usr/bin/env python3
"""Read all four ultrasonic sensors and flag invalid values."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.serial_link import SerialLink
parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config); link=SerialLink(cfg['serial']); link.start(); time.sleep(1)
try:
    while True:
        link.tick(); t=link.telemetry(); vals=(t.dF,t.dL,t.dR,t.dB)
        print('F/L/R/B='+' / '.join('INVALID' if v<0 else f'{v:5.0f}cm' for v in vals),end='\r',flush=True); time.sleep(0.1)
except KeyboardInterrupt: print()
finally: link.stop()
