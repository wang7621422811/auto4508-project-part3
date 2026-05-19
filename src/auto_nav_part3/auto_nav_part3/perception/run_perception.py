#!/usr/bin/env python3
"""
run_perception.py — Launch all perception nodes for Part 3.
Run from inside Docker container.

Usage:
  python3 run_perception.py

Stop with Ctrl+C.
"""

import os
import signal
import subprocess
import sys
import time

# ── CHANGE THESE IF NEEDED ───────────────────────────────────────────────
WORKSPACE     = '/root/workspace/auto4508-project-part3'
MODEL_PATH    = f'{WORKSPACE}/models/greek_letters.onnx'
PHOTO_DIR     = '/root/workspace/artifacts/photos'
ARTIFACT_DIR  = '/home/team18/team18_workspace/artifacts/markers'
COOLDOWN      = '10.0'
MIN_CONF      = '0.7'
# ─────────────────────────────────────────────────────────────────────────

NODE_DIR = f'{WORKSPACE}/src/auto_nav_part3/auto_nav_part3'
processes = []


def start(name, cmd):
    print(f'[START] {name}')
    p = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        preexec_fn=os.setsid,
    )
    processes.append((name, p))
    return p


def stop_all(sig=None, frame=None):
    print('\n[STOP] Shutting down all nodes...')
    for name, p in reversed(processes):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            print(f'  Stopped {name}')
        except Exception:
            pass
    time.sleep(1)
    for name, p in reversed(processes):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT,  stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    print('=' * 55)
    print('  Part 3 Perception Stack')
    print('=' * 55)
    print(f'  Model    : {MODEL_PATH}')
    print(f'  Photos   : {PHOTO_DIR}')
    print(f'  Manifest : {ARTIFACT_DIR}')
    print('=' * 55)

    # 1. OAK-D camera
    start('oakd_camera',
        f'cd {NODE_DIR} && python3 oakd_camera.py')
    time.sleep(3)

    # 2. Colour detector
    start('colour_detector',
        f'cd {NODE_DIR} && python3 colour_detector.py --ros-args '
        f'-p photo_dir:={PHOTO_DIR} '
        f'-p detection_cooldown_s:={COOLDOWN}')
    time.sleep(1)

    # 3. Greek detector
    start('greek_detector',
        f'cd {NODE_DIR} && python3 greek_detector.py --ros-args '
        f'-p greek_model_path:={MODEL_PATH} '
        f'-p photo_dir:={PHOTO_DIR} '
        f'-p detection_cooldown_s:={COOLDOWN} '
        f'-p min_confidence:={MIN_CONF}')
    time.sleep(1)

    # 4. Photo logger
    start('photo_logger',
        f'cd {NODE_DIR} && python3 photo_logger.py --ros-args '
        f'-p artifact_dir:={ARTIFACT_DIR}')

    print()
    print('  All nodes running. Ctrl+C to stop.')
    print('  Watch detections:')
    print('    ros2 topic echo /part3/perception/marker_event')
    print('    ros2 topic echo /part3/perception/logger_status')
    print('=' * 55)

    try:
        while True:
            time.sleep(1)
            for name, p in processes:
                if p.poll() is not None:
                    print(f'[WARN] {name} exited (code={p.returncode})')
    except KeyboardInterrupt:
        stop_all()


if __name__ == '__main__':
    main()