#!/usr/bin/env python3
"""Read-only Arduino telemetry monitor."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.config import load_yaml
from robot.serial_link import SerialLink

parser=argparse.ArgumentParser(); parser.add_argument('--config',default=str(PROJECT_ROOT/'config'/'robot.yaml'))
args=parser.parse_args(); cfg=load_yaml(args.config)
link=SerialLink(cfg['serial']); link.start()
try:
    while True:
        link.tick(); t=link.telemetry()
        print(f"boot={t.boot_id} armed={t.armed} yaw={t.yaw:7.2f} enc={t.enc_cm:8.2f} "
              f"F/L/R/B={t.dF:5.0f}/{t.dL:5.0f}/{t.dR:5.0f}/{t.dB:5.0f} state={t.state}", end='\r', flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print()
finally:
    link.stop()
