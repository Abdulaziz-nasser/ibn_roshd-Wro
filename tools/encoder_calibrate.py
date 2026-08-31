#!/usr/bin/env python3
"""Measure encoder counts-per-centimetre with motors stopped."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.serial_link import SerialLink

parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config)
print('Keep motor power off. Mark the wheel/floor and move the robot by hand on a measured straight line.')
link=SerialLink(cfg['serial']); link.start(); time.sleep(1.0); link.send('STOP'); link.send('RESET_ENCODER')
try:
    input('Press Enter after the robot is at the zero mark...')
    start=link.telemetry().enc_ticks
    distance=float(input('Enter the exact distance you will move the robot in cm (for example 100): '))
    input('Move the robot that distance by hand, then press Enter...')
    end=link.telemetry().enc_ticks
    counts=abs(end-start)
    if distance<=0 or counts<=0: raise SystemExit('No usable movement was measured.')
    cpcm=counts/distance
    print(f'Encoder ticks travelled: {counts}')
    print(f'Calculated ENCODER_COUNTS_PER_CM = {cpcm:.6f}')
    print('Put that value in arduino/robot_firmware/config.h, upload again, and repeat the test.')
finally:
    link.stop()
