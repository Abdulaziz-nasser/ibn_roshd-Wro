#!/usr/bin/env python3
"""Low-speed staged motion test. Use only with wheels raised and adult supervision."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.motion import MotionController
from robot.serial_link import SerialLink
from robot.protocol import wrap180

parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config)
print("SAFETY: disconnect the robot from the floor, raise the drive wheels, and keep hands clear.")
if input("Type RAISED to continue: ").strip() != "RAISED":
    raise SystemExit("Cancelled")
link=SerialLink(cfg['serial']); link.start(); motion=MotionController(link,cfg)
time.sleep(1.0); link.send('ARM')
print("Commands: f=forward 10cm, b=back 10cm, l=left 30deg, r=right 30deg, s=STOP, q=quit")
try:
    while True:
        link.tick(); result=motion.poll()
        if result: print("Result:",result)
        command=input('> ').strip().lower()
        if command=='q': break
        if command=='s': motion.stop(); continue
        if motion.active: print('A command is still active'); continue
        tlm=link.telemetry()
        if command=='f': motion.start_move('F',10,28,3.0,hold_heading=True)
        elif command=='b': motion.start_move('B',10,28,3.0,hold_heading=True)
        elif command=='l': motion.start_turn(wrap180(tlm.yaw-30),30,4,2.0,direction=-1,mode='F')
        elif command=='r': motion.start_turn(wrap180(tlm.yaw+30),30,4,2.0,direction=+1,mode='F')
        while motion.active:
            link.tick(); result=motion.poll()
            if result: print('Result:',result); break
            time.sleep(0.02)
finally:
    motion.stop(); link.stop()
