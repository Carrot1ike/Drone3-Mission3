"""70도 방향/출발 위치의 실험 조건. controller에는 초기 참값을 넣지 않는다."""
import json
import math
from provided_hardware import ROOT

SCENARIOS = ("baseline", "passby70", "passby-no-marker", "rear-bearing70", "rear-bearing-minus70", "heading70", "heading-minus70", "lateral70", "combined70", "blind-search")


def configure_scenario(config, scenario):
    if scenario not in SCENARIOS:
        raise ValueError("unknown optical scenario")
    config["scenario"] = {"name": scenario}
    if scenario == "baseline":
        return config
    if scenario in ("passby70", "passby-no-marker"):
        settings = json.loads((ROOT / "config/optical-passby.json").read_text())
        config["search"] = settings["search"]
        config["spawn"] = settings["spawn"]
        config["control"]["mission_timeout_s"] = 90.
        # 고정 대기 시선 90°, 뒤쪽 선 160° -> 작은 각 70°.
        # 참값은 초기 장면 설정에만 쓰고 controller에는 넘기지 않는다.
        distance = abs(config["spawn"]["y_m"])
        rear_x = -distance/math.tan(math.radians(20.))
        config["vehicle"]["initial_pad_center_m"][0] = rear_x+config["vehicle"]["pad_length_m"]/2
        config["vehicle"]["motion_mode"] = "continuous"
        config["vehicle"]["trajectory"] = "independent constant +X motion from mission start; not triggered by marker detection"
        config["scenario"].update(
            interpretation="user drawings: fixed position and gaze while vehicle passes; detect, move behind and align, approach and descend",
            rear_reference_initial_world_m=[rear_x, 0., config["vehicle"]["initial_pad_center_m"][2]],
            angle_between_forward_and_rear_line_deg=settings["initial_forward_to_rear_smaller_angle_deg"],
            initial_heading_deg=config["spawn"]["yaw_deg"], vehicle_motion_start="mission start, independent of detection",
            physical_touchdown_tested=False)
        if scenario == "passby-no-marker":
            config["test"]["hide_marker_all_frames"] = True
        return config
    settings = json.loads((ROOT / "config/optical-acquisition-70.json").read_text())
    config["search"] = settings["search"]
    config["control"]["mission_timeout_s"] = 90.
    if scenario in ("rear-bearing70", "rear-bearing-minus70"):
        pad = config["vehicle"]["initial_pad_center_m"]
        rear_reference = [pad[0]-config["vehicle"]["pad_length_m"]/2, pad[1], pad[2]]
        sx, sy = config["spawn"].get("x_m", 0.), config["spawn"]["y_m"]
        line_bearing = math.degrees(math.atan2(rear_reference[1]-sy, rear_reference[0]-sx))
        sign = 1. if scenario == "rear-bearing70" else -1.
        config["spawn"]["yaw_deg"] = line_bearing + sign*70.
        config["scenario"].update(rear_reference_initial_world_m=rear_reference,
                                  angle_between_forward_and_rear_line_deg=70.,
                                  interpretation="user requested smaller horizontal angle between camera forward and line to vehicle rear; rear reference is pad rear edge center")
    if scenario in ("heading70", "combined70", "blind-search"):
        config["spawn"]["yaw_deg"] = 70.
    elif scenario == "heading-minus70":
        config["spawn"]["yaw_deg"] = -70.
    if scenario in ("lateral70", "combined70"):
        angle = math.radians(70.)
        radius = settings["lateral_start_distance_from_pad_m"]
        px, py, _ = config["vehicle"]["initial_pad_center_m"]
        config["spawn"].update(x_m=px-radius*math.cos(angle), y_m=py+radius*math.sin(angle))
        config["scenario"].update(location_angle_from_behind_deg=70., optical_point_distance_from_pad_m=radius)
    if scenario == "blind-search":
        config["test"]["hide_marker_all_frames"] = True
    config["scenario"].update(initial_heading_deg=config["spawn"]["yaw_deg"], vehicle_motion_start="first accepted marker discovery", physical_touchdown_tested=False)
    return config
