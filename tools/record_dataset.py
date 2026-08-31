#!/usr/bin/env python3
"""Record labelled one-camera frames for future learning/model training."""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import cv2 as cv
PROJECT_ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT_ROOT))
from robot.camera import OneCamera
from robot.config import load_yaml

parser=argparse.ArgumentParser(); parser.add_argument('--vision',default=str(PROJECT_ROOT/'config'/'vision.yaml'))
parser.add_argument('--output',default=str(PROJECT_ROOT/'dataset'))
args=parser.parse_args(); cfg=load_yaml(args.vision); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
labels={ord('1'):'BLUE_LINE',ord('2'):'ORANGE_LINE',ord('3'):'RED_PILLAR',ord('4'):'GREEN_PILLAR',ord('0'):'NONE',ord('u'):'UNCERTAIN',ord('U'):'UNCERTAIN'}
cam=OneCamera(cfg['camera']); cam.start(); cv.namedWindow('dataset',cv.WINDOW_NORMAL); last=-1
print('Keys: 1 blue, 2 orange, 3 red, 4 green, 0 none, U uncertain, Q quit')
try:
    while True:
        packet=cam.latest(copy_frame=True)
        if packet is None or packet.sequence==last: time.sleep(0.01); continue
        last=packet.sequence; cv.imshow('dataset',packet.image); key=cv.waitKey(1)&0xFF
        if key in (ord('q'),ord('Q')): break
        if key in labels:
            label=labels[key]; folder=out/label; folder.mkdir(exist_ok=True)
            stem=f"{int(time.time()*1000)}"; image_path=folder/f"{stem}.jpg"
            cv.imwrite(str(image_path),packet.image,[cv.IMWRITE_JPEG_QUALITY,95])
            (folder/f"{stem}.json").write_text(json.dumps({'label':label,'timestamp':packet.timestamp,'vision_config':str(args.vision)},indent=2))
            print('Saved',image_path)
finally:
    cam.stop(); cv.destroyAllWindows()
