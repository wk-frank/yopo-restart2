#!/usr/bin/env python3
"""Prepare opt-in cameras in the run's world and disposable container model."""
import argparse
import json
from pathlib import Path

from yopo_l0.gazebo_native import prepare_native_scene
from yopo_l0.scene import load_card
from yopo_l0.sensor import CameraConfig

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--base-model", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    scene_path = Path(args.scene)
    if not scene_path.is_file():
        scene_path = Path(__file__).resolve().parents[2] / scene_path
    card = load_card(scene_path)
    summary = prepare_native_scene(args.world, args.base_model, CameraConfig.from_card(card.card["camera"]))
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")
