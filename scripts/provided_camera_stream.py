#!/usr/bin/env python3
"""Native mono320 optical fixtures over the pinned camera transport, no aircraft."""
import json
import math
import os
from pathlib import Path
import socket
import struct
import time

import mujoco
import numpy as np
from moving_landing_scene import build_scene, build_optical_preview_model
from provided_hardware import decode_mono320


def main():
    out = Path(os.environ["DRONE3_EXPERIMENT_DIR"])
    config = json.loads((out / "experiment.json").read_text())
    model = build_optical_preview_model(config, out / "scene.xml")
    data = mujoco.MjData(model)
    camera = model.camera("optical_preview").id
    slide = model.joint("vehicle_slide").qposadr[0]
    origin = np.asarray(config["vehicle"]["initial_pad_center_m"])
    renderer = mujoco.Renderer(model, height=320, width=320)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = deadline = time.monotonic()
    print("기체 없는 지정 광학점 영상: mono8 320x320, 테스트 송신 주기 20fps", flush=True)
    try:
        while True:
            t = time.monotonic()-started
            travel = config["vehicle"]["speed_m_s"] * t
            data.qpos[slide] = travel
            phase = int(t / 2.) % 4
            dx, h = [(-.35, .30), (0., .30), (0., .15), (0., .03)][phase]
            # Directly assigned optical fixtures. This does not implement pursuit.
            model.cam_pos[camera] = origin + [travel+dx, 0., h]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="optical_preview")
            mono = (renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
            payload = mono.tobytes()
            decode_mono320(320, 320, "mono8", 320, payload)
            total = math.ceil(len(payload)/60000)
            for seq in range(total):
                sender.sendto(struct.pack("<HHHH", seq, total, 320, 320) + payload[seq*60000:(seq+1)*60000], ("127.0.0.1", 5200))
            deadline += .05
            time.sleep(max(0., deadline-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        sender.close()
        renderer.close()


if __name__ == "__main__":
    main()
