"""하드웨어 사양, 사용자 치수와 실험 설정의 계약."""
from copy import deepcopy
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load(path=None):
    cfg = json.loads(Path(path or ROOT / "config/mission3.json").read_text())
    if (cfg["camera"]["width"], cfg["camera"]["height"], cfg["camera"]["encoding"]) != (320, 320, "mono8"):
        raise ValueError("제공 카메라는 native mono8 320×320이어야 합니다")
    vehicle, marker = cfg["vehicle_plant_only"], cfg["marker"]
    if abs(marker["code_side_m"] + 2 * marker["quiet_zone_each_side_m"] - marker["board_side_m"]) > 1e-9:
        raise ValueError("인쇄 코드와 흰 여백의 크기가 판 크기와 다릅니다")
    expected = [vehicle["pad_length_m"] / 2 + vehicle["marker_plane_gap_from_pad_front_edge_m"],
                0., vehicle["marker_board_height_m"] / 2]
    if expected != marker["known_marker_to_pad_vehicle_m"] or expected != vehicle["marker_center_from_pad_vehicle_m"]:
        raise ValueError("상판→마커 오프셋 치수 불일치")
    return cfg


def controller_settings(cfg):
    """차량 live 상태/실험 seed/plant 속도에 접근할 수 없는 공개 설정."""
    allowed = ("camera", "arena", "track", "marker", "landing_geometry", "entry", "observer", "control")
    return {key: deepcopy(cfg[key]) for key in allowed}


def decoded_candidate_gaze_options(settings):
    """한 decoded ID는 시선/제동만 허용한다. 기존 coarse 확인은 별도다."""
    options = settings.get("decoded_candidate_gaze", {})
    if options.get("enabled", False):
        age = options.get("max_age_s")
        if age is None or not math.isfinite(age) or not 0 < age <= .7:
            raise ValueError("candidate gaze requires a finite age no longer than coarse evidence")
    return options
