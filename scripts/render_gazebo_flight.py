#!/usr/bin/env python3
"""Annotate actual Gazebo camera frames with synchronized measured flight telemetry."""
from __future__ import annotations
import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np


def annotate(run: Path, dest: Path):
    dest.mkdir(parents=True, exist_ok=True)
    raw = json.loads((run/'result.json').read_text())
    evidence = json.loads((run/'gazebo-video.json').read_text())
    video = run/'gazebo-flight.mp4'
    assert hashlib.sha256(video.read_bytes()).hexdigest() == evidence['mp4_sha256']
    stamps = [json.loads(line)['gazebo_stamp_s'] for line in (run/'gazebo-video-frames.jsonl').read_text().splitlines()]
    tracks = raw['flight_record']
    ids = list(tracks)
    times = {k:np.array([s['t_s'] for s in tracks[k]['samples']]) for k in ids}
    origins = {d['id']:np.array(d['world_offset_enu']) for d in raw['drones']}
    positions = {k:np.array([s.get('truth_p') or s['p'] for s in tracks[k]['samples']]) for k in ids}
    distances = {k:np.linalg.norm(positions[k][:,:2]-origins[k][:2],axis=1) for k in ids}
    capture = cv2.VideoCapture(str(video))
    assert capture.isOpened()
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == len(stamps) == evidence['frames']
    out = dest/'gazebo-flight-annotated.mp4'
    process = subprocess.Popen(['ffmpeg','-nostdin','-y','-loglevel','warning','-f','rawvideo','-pixel_format','bgr24',
        '-video_size','1600x900','-framerate','10','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast',
        '-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
    colors = [(190,215,60),(70,175,245),(245,150,135),(230,130,220),(100,210,230),(240,205,100)]
    def text(img, value, x, y, size=.55, color=(218,228,238), thick=1):
        cv2.putText(img,value,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,thick,cv2.LINE_AA)
    try:
        for i, stamp in enumerate(stamps):
            ok, frame = capture.read()
            assert ok, i
            t = stamp-float(raw['mission_t0_s'])
            canvas = np.full((900,1600,3),(24,20,16),np.uint8)
            canvas[80:800,:1280] = frame
            text(canvas,'YOPO + CTBR  /  GAZEBO NATIVE FLIGHT',28,34,.85,thick=2)
            if raw.get('perception_contract',{}).get('mode')=='depth-only':
                text(canvas,'DEPTH ONLY / NO PRIOR MAP',1100,34,.5,(140,230,150),1)
            text(canvas,f'6 aircraft  |  Centre-out elastic formation  |  SIM {t:6.1f} s',28,64,.55)
            text(canvas,'Gazebo camera / full view',965,64,.45)
            text(canvas,'MEASURED DISPLACEMENT',1300,108,.52)
            current=[]
            for j, k in enumerate(ids):
                idx=max(0,min(int(np.searchsorted(times[k],t,side='right'))-1,len(times[k])-1))
                sample=tracks[k]['samples'][idx]
                point=np.array([np.interp(t,times[k],positions[k][:,axis]) for axis in range(3)])
                d=float(np.linalg.norm(point[:2]-origins[k][:2])); current.append(d)
                achieved=max(d,float(np.max(distances[k][:idx+1])))
                yy=155+j*103
                cv2.rectangle(canvas,(1294,yy-30),(1588,yy+61),(41,34,28),-1)
                cv2.circle(canvas,(1310,yy-8),5,colors[j],-1)
                text(canvas,k.upper(),1327,yy-2,.6,colors[j],2)
                text(canvas,f'{d:7.2f} m',1320,yy+28,.77,thick=2)
                text(canvas,f"{sample['control'].upper()} | {sample['phase']}",1320,yy+49,.4)
                if achieved>=900:
                    text(canvas,'900m PASS',1477,yy-3,.42,(140,230,150),1)
            minimum=min(current)
            text(canvas,f'Minimum of all six: {minimum:7.2f} m  /  Required: 900 m per aircraft',28,835,.66,thick=2)
            cv2.rectangle(canvas,(28,853),(1260,865),(55,46,38),-1)
            cv2.rectangle(canvas,(28,853),(28+int(1232*min(minimum/900,1)),865),(160,215,80),-1)
            text(canvas,'Actual Gazebo Ogre2 camera images + recorded Gazebo truth | Native depth 15 Hz | CTBR 50 Hz | SITL simulation',28,890,.45,(155,170,184))
            process.stdin.write(canvas.tobytes())
            if i==len(stamps)*3//5:
                cv2.imwrite(str(dest/'gazebo-preview.png'),canvas)
            if i==max(0,len(stamps)-30):
                cv2.imwrite(str(dest/'gazebo-arrival.png'),canvas)
    finally:
        capture.release(); process.stdin.close()
    assert process.wait()==0
    shutil.copy2(video,dest/'gazebo-flight.mp4')
    duration=len(stamps)/10
    subprocess.run(['ffmpeg','-nostdin','-y','-loglevel','warning','-i',str(out),'-an','-vf',
        f'setpts={min(1,30/duration):.8f}*PTS,fps=20','-c:v','libx264','-preset','veryfast','-crf','21',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(dest/'gazebo-preview.mp4')],check=True)
    return {'raw_frames':len(stamps),'raw_duration_s':duration,'first_stamp_s':stamps[0],
            'last_stamp_s':stamps[-1],'max_stamp_gap_s':float(np.max(np.diff(stamps))),
            'annotated_video_sha256':hashlib.sha256(out.read_bytes()).hexdigest()}


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    print(json.dumps(annotate(a.run,a.out),indent=2))
