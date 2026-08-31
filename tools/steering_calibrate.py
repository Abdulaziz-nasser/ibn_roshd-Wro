#!/usr/bin/env python3
"""Move only the steering servo while drive PWM stays at zero."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.serial_link import SerialLink

parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config)
print('SAFETY: motor power should be disconnected. Steering linkage must move freely.')
if input('Type SERVO to continue: ').strip()!='SERVO': raise SystemExit('Cancelled')
link=SerialLink(cfg['serial']); link.start(); time.sleep(1); link.send('ARM')
print('Keys: l=full left, c=center, r=full right, q=quit')
try:
    while True:
        key=input('> ').strip().lower()
        if key=='q': break
        norm={'l':-1.0,'c':0.0,'r':1.0}.get(key)
        if norm is None: continue
        link.send('DRIVE',dir='F',steer=norm,pwm=0)
        print('Telemetry steer:',link.telemetry().steer)
finally:
    link.send('STOP'); link.stop()
