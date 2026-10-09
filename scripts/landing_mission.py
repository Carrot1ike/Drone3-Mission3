"""카메라 광학점의 이상적 운동학용 이동 패드 임무 상태기계.

LandingMission(config).update(now_s, observation)의 입력/출력 축은 동일하다.
relative_pad_m = 패드 중심 - 카메라 광학점이며, +Z는 위쪽이다. 따라서
패드 위 카메라 높이는 -relative_pad_m[2], 상대속도는 이 벡터의 시간미분이다.
stamp_s는 마지막 영상 측정의 시각이며 PC/시뮬레이션의 now_s와 같은 시계를
사용한다. prediction_stamp_s(선택)는 relative_pad_m이 예측된 시각이다. 없으면
위치 시각도 stamp_s로 간주한다. 위치 시각부터 now_s까지만 상대속도로 선형
예측하므로 이미 now_s까지 예측한 state를 이중 예측하지 않는다. 영상보다
빠른 제어 루프는 동일 stamp의 캐시를 age 범위 안에서 사용한다. 같은 stamp는
관측 손실 시계를 갱신하지 않는다. 역순 stamp는 거부하고 확인 시간을 초기화한다.

출력 velocity_m_s는 광학점의 절대 속도, yaw_rate_rad_s는 물리적 회전 부호의
각속도다. ROS 부호 변환, 영상 인식, 기체/카메라 변환, 관성/추력/접촉은 없다.
차는 +X 방향으로 움직인다. 기본 모드에서 ALIGN 확인 뒤 APPROACH에 진입한다.
search.enabled에서는 SEARCH 회전→ACQUIRE 마커 중심 추적→ALIGN 방향 맞춤→
LINEUP 횡방향 정렬을 먼저 수행한다. 마커 발견 시 차량 이동을 시작할 수 있다.
초기 회전 탐색의 제한시간과 발견 후 영상 손실 제한시간은 구분한다.
search.passive_wait에서는 WAIT 중 회전·병진을 모두 0으로 유지하고, 독립적으로
지나가는 차량을 영상으로 발견한다. ACQUIRE 뒤 LINEUP에서 마커를 바라보며
차량 뒤쪽 설정 거리와 횡방향·heading을 함께 맞춘 뒤 APPROACH로 진행한다.
차량 시작 요청은 보내지 않으며 알려진 차량 속도 FF를 발견 이후 적용한다.
이동 중에는 설정된 차 속도를
FF로 더한다. test.ff_enabled=False는 FF를 제거하는 비교 실험이다.

COMPLETE는 약 0.03 m의 near_pad_ready 조건을 관측으로 확인한 결과일 뿐이다.
done과 inferred_completion은 COMPLETE에서만 True다. COMPLETE/ABORT 출력은
0이며, 이상적 runner의 종료/인계를 위한 값이지 실제 기체의 착륙/모터 명령이
아니다. 관측 손실/시간 오류/하강 금지 시 즉시 정지는 정상 가속도 제한보다
우선한다. 손실 복구 후에는 0에서 가속도 제한을 적용해 다시 명령한다.

아래 기본값은 공개된 하드웨어 수치가 아니라 이 모듈의 로컬 실험 설정이다.
기존 control/observer/test/vehicle 값을 우선하며 입력 config를 수정하지 않는다.
takeoff/land/post_touchdown 및 Flow 표면 보정 설정은 광학점 운동학에 적용하지
않는다. 물리 접촉 평가값을 완료 조건으로 해석하지 않는다.
"""

from collections.abc import Mapping
import math


CONTROL_DEFAULTS = {
    "kp_per_s": 0.8,
    "deadband_deg": 1.0,
    "max_rate_deg_s": 15.0,
    "max_command_gap_s": 0.25,
    "loss_abort_s": 2.0,
    "cruise_height_above_pad_m": 0.30,
    "xy_kp_per_s": 0.7,
    "xy_kd": 0.5,
    "max_velocity_m_s": 0.55,
    "max_accel_m_s2": 0.50,
    "height_gain": 0.8,
    "align_hold_s": 1.0,
    "track_hold_s": 2.0,
    "track_error_m": 0.045,
    "track_relative_velocity_m_s": 0.065,
    "descent_m_s": 0.035,
    "descent_position_gate_m": 0.065,
    "descent_velocity_gate_m_s": 0.10,
    "touchdown_height_m": 0.035,  # 기존 이름; 접촉이 아닌 near_pad_ready 상한
    "touchdown_hold_s": 0.30,    # 기존 이름; SETTLE 관측 확인 시간
    "mission_timeout_s": 65.0,
    # 기존 JSON에 없는 항목은 명시적인 로컬 기본값이다.
    "align_tolerance_deg": 2.0,
    "track_yaw_gate_deg": 2.0,
    "descent_yaw_gate_deg": 2.0,
    "cruise_height_tolerance_m": 0.025,
    "track_vertical_velocity_m_s": 0.025,
    "descent_max_visual_age_s": 0.15,
    "virtual_completion_height_m": 0.030,
    "completion_height_tolerance_m": 0.005,
    "settle_vertical_velocity_m_s": 0.015,
}
OBSERVER_DEFAULTS = {"max_visual_age_s": 0.40}
VEHICLE_DEFAULTS = {"speed_m_s": 0.30}
TEST_DEFAULTS = {"ff_enabled": True}
SEARCH_DEFAULTS = {
    "enabled": False, "rate_deg_s": 12.0, "direction": -1,
    "timeout_s": 35.0, "acquire_hold_s": 0.30, "bearing_tolerance_deg": 3.0,
    "lineup_tolerance_m": 0.015, "lineup_speed_tolerance_m_s": 0.025,
    "lineup_hold_s": 1.0, "start_vehicle_on_discovery": True,
    "passive_wait": False, "lineup_standoff_m": 0.35,
    "lineup_distance_tolerance_m": 0.025, "lineup_x_speed_tolerance_m_s": 0.025,
    "translation_bearing_gate_deg": 12.0,
}
_EPS = 1e-9


def _number(value):
    """유한 스칼라를 읽는다. bool/문자열을 센서 수치로 받아들이지 않는다."""
    if value is None or isinstance(value, (bool, str, bytes)):
        raise ValueError("finite numeric scalar required")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("finite numeric scalar required")
    return result


def _vector(value):
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError("three numeric components required")
    result = tuple(_number(v) for v in value)
    if len(result) != 3:
        raise ValueError("three numeric components required")
    return result


def _norm(vector):
    return math.hypot(*vector)


def _clip_norm(vector, limit):
    magnitude = _norm(vector)
    if magnitude > limit:
        return [v * (limit / magnitude) for v in vector]
    return list(vector)


def _settings(config, section, defaults):
    supplied = config.get(section, {})
    if not isinstance(supplied, Mapping):
        raise ValueError(f"{section} must be a mapping")
    return {key: supplied.get(key, value) for key, value in defaults.items()}


class LandingMission:
    """영상/ROS/기체 모델에 의존하지 않는 순수 상태기계와 FF+피드백 제어기.

    None, valid=False, 누락/비유한 값, 오래된/역순/미래 관측은 사용하지 않고
    명령을 0으로 만든다. 동일 stamp는 캐시이며 새 영상으로 취급하지 않는다.
    마지막 승인 영상 stamp에서 2초 이상 지난 경우 observation_loss로 ABORT한다.
    초기 SEARCH/WAIT는 별도의 search.timeout_s를 사용한다.
    hold/하강에는 age <= descent_max_visual_age_s(기본 0.15초)가 필요하다.
    임무 제한시간은 첫 update부터 SEARCH를 포함한다. 호출 시간 역행/중복은
    확인 시간을 초기화하고 처리하지 않으며 시계 최댓값을 뒤로 옮기지 않는다.
    """

    def __init__(self, config):
        if not isinstance(config, Mapping):
            raise ValueError("config must be a mapping")
        self.control = _settings(config, "control", CONTROL_DEFAULTS)
        self.observer = _settings(config, "observer", OBSERVER_DEFAULTS)
        self.vehicle = _settings(config, "vehicle", VEHICLE_DEFAULTS)
        self.test = _settings(config, "test", TEST_DEFAULTS)
        self.search = _settings(config, "search", SEARCH_DEFAULTS)
        for key in ("enabled", "start_vehicle_on_discovery", "passive_wait"):
            if not isinstance(self.search[key], bool):
                raise ValueError(f"search.{key} must be bool")
        for key in ("rate_deg_s", "timeout_s", "acquire_hold_s", "bearing_tolerance_deg",
                    "lineup_tolerance_m", "lineup_speed_tolerance_m_s", "lineup_hold_s",
                    "lineup_standoff_m", "lineup_distance_tolerance_m", "lineup_x_speed_tolerance_m_s",
                    "translation_bearing_gate_deg"):
            self.search[key] = _number(self.search[key])
            if self.search[key] <= 0:
                raise ValueError(f"search.{key} must be positive")
        if self.search["direction"] not in (-1, 1):
            raise ValueError("search.direction must be -1 or 1")
        self.passive_wait = self.search["passive_wait"]
        if self.passive_wait and (not self.search["enabled"] or self.search["start_vehicle_on_discovery"]):
            raise ValueError("passive wait needs enabled acquisition and independent vehicle motion")
        self.marker_offset = _vector(config.get("vehicle", {}).get("marker_offset_from_pad_m", (.45, 0., .15)))
        zero_allowed = {"xy_kp_per_s", "xy_kd", "kp_per_s", "deadband_deg"}
        for key, value in self.control.items():
            self.control[key] = value = _number(value)
            if value < 0 or (value == 0 and key not in zero_allowed):
                raise ValueError(f"control.{key} must be positive")
        self.observer["max_visual_age_s"] = _number(self.observer["max_visual_age_s"])
        self.vehicle["speed_m_s"] = _number(self.vehicle["speed_m_s"])
        if self.observer["max_visual_age_s"] <= 0 or self.vehicle["speed_m_s"] < 0:
            raise ValueError("max_visual_age_s must be positive; speed_m_s nonnegative")
        if not isinstance(self.test["ff_enabled"], bool):
            raise ValueError("test.ff_enabled must be bool")
        c = self.control
        if not (0 < c["completion_height_tolerance_m"] < c["virtual_completion_height_m"]
                <= c["touchdown_height_m"] < c["cruise_height_above_pad_m"]):
            raise ValueError("virtual completion height/tolerance/cruise ordering invalid")
        if c["deadband_deg"] > min(c["align_tolerance_deg"], c["track_yaw_gate_deg"],
                                   c["descent_yaw_gate_deg"]):
            raise ValueError("yaw deadband must not exceed a readiness gate")

        self.phase = "WAIT" if self.passive_wait else "SEARCH"
        self.start_vehicle = False
        self.failure = None
        self._start_s = None
        self._last_now_s = None
        self._last_measurement_s = None
        self._hold_since_s = None
        self._last_velocity = [0.0, 0.0, 0.0]
        self._descent_height_m = c["cruise_height_above_pad_m"]
        self._descent_step_s = None
        self._descent_reanchor = True
        self._lineup_x_m = None

    def _result(self, velocity=None, yaw_rate=0.0):
        return {
            "phase": self.phase,
            "velocity_m_s": list(self._last_velocity if velocity is None else velocity),
            "yaw_rate_rad_s": float(yaw_rate),
            "start_vehicle": self.start_vehicle,
            "done": self.phase == "COMPLETE",
            "failure": self.failure,
            "inferred_completion": self.phase == "COMPLETE",
        }

    def _stop(self):
        self._last_velocity = [0.0, 0.0, 0.0]
        return self._result()

    def _pause(self):
        self._hold_since_s = None
        self._descent_step_s = None
        self._descent_reanchor = True

    def _abort(self, reason):
        self.phase, self.failure = "ABORT", reason
        self._pause()
        return self._stop()

    def _enter(self, phase):
        self.phase = phase
        self._hold_since_s = None
        if phase == "APPROACH" and not self.passive_wait:
            self.start_vehicle = True

    def _held(self, now_s, condition, duration_s):
        if not condition:
            self._hold_since_s = None
            return False
        if self._hold_since_s is None:
            self._hold_since_s = now_s
        return now_s - self._hold_since_s + _EPS >= duration_s

    def _read_observation(self, now_s, observation):
        if not isinstance(observation, Mapping) or observation.get("valid") is not True:
            return None
        try:
            stamp = _number(observation["stamp_s"])
            position_stamp = _number(observation.get("prediction_stamp_s", stamp))
            position = _vector(observation["relative_pad_m"])
            velocity = _vector(observation["relative_velocity_m_s"])
            yaw = _number(observation["heading_error_rad"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return None
        age = now_s - stamp
        if (age < 0 or age > self.observer["max_visual_age_s"] + _EPS
                or position_stamp < stamp or position_stamp > now_s
                or (self._last_measurement_s is not None and stamp < self._last_measurement_s)):
            return None
        predicted = tuple(p + v * (now_s - position_stamp) for p, v in zip(position, velocity))
        if not all(math.isfinite(p) for p in predicted):
            return None
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        if self._last_measurement_s is None or stamp > self._last_measurement_s:
            self._last_measurement_s = stamp
        return position, predicted, velocity, yaw, age

    def _limit_velocity(self, desired, dt_s, freeze_z=False, forbid_down=False):
        """3축 norm/벡터 가속도 제한. 안전 하강정지는 Z 가속도 제한보다 우선."""
        c = self.control
        desired = _clip_norm(desired, c["max_velocity_m_s"])
        delta = _clip_norm([v - old for v, old in zip(desired, self._last_velocity)],
                           c["max_accel_m_s2"] * dt_s)
        command = [old + change for old, change in zip(self._last_velocity, delta)]
        if freeze_z:
            command[2] = 0.0
        elif forbid_down:
            command[2] = max(0.0, command[2])
        self._last_velocity = command
        return command

    def update(self, now_s, observation):
        """상태를 한 단계 갱신하고 JSON 직렬화 가능한 명령 dict를 반환한다."""
        if self.phase in ("COMPLETE", "ABORT"):
            return self._stop()
        try:
            now_s = _number(now_s)
        except (ValueError, TypeError, OverflowError):
            return self._abort("invalid_clock")
        if self._start_s is None:
            self._start_s = now_s
        if self._last_now_s is not None and now_s <= self._last_now_s:
            self._pause()
            return self._stop()
        dt_s = 0.0 if self._last_now_s is None else now_s - self._last_now_s
        self._last_now_s = now_s
        c = self.control
        if now_s - self._start_s + _EPS >= c["mission_timeout_s"]:
            return self._abort("mission_timeout")
        if self.phase in ("SEARCH", "WAIT") and self.search["enabled"]:
            if now_s-self._start_s >= self.search["timeout_s"]:
                return self._abort("wait_timeout" if self.passive_wait else "search_timeout")
            sample = self._read_observation(now_s, observation)
            if sample is None:
                self._pause()
                self._last_velocity = [0., 0., 0.]
                if dt_s > c["max_command_gap_s"]:
                    return self._stop()
                if self.passive_wait:
                    return self._stop()  # 영상 발견 전에는 회전·병진 명령 모두 0.
                scan_rate = self.search["direction"] * math.radians(min(self.search["rate_deg_s"], c["max_rate_deg_s"]))
                return self._result(yaw_rate=scan_rate)
            self._lineup_x_m = sample[1][0]
            self._enter("ACQUIRE")
            self.start_vehicle = self.search["start_vehicle_on_discovery"]
        else:
            sample = None
        loss_reference = self._start_s if self._last_measurement_s is None else max(
            self._start_s, self._last_measurement_s)
        # 긴 공백 뒤의 단발 관측으로 이미 발생한 장기 손실을 취소하지 않는다.
        if now_s - loss_reference + _EPS >= c["loss_abort_s"]:
            return self._abort("observation_loss")
        if sample is None:
            sample = self._read_observation(now_s, observation)
        if sample is None or dt_s > c["max_command_gap_s"] + _EPS:
            self._pause()
            return self._stop()
        raw, position, relative_velocity, yaw_error, age = sample
        height = -position[2]
        xy_error = max(_norm(raw[:2]), _norm(position[:2]))
        xy_speed = _norm(relative_velocity[:2])
        track_ready = (
            xy_error <= c["track_error_m"]
            and xy_speed <= c["track_relative_velocity_m_s"]
            and abs(relative_velocity[2]) <= c["track_vertical_velocity_m_s"]
            and abs(height - c["cruise_height_above_pad_m"]) <= c["cruise_height_tolerance_m"]
            and abs(yaw_error) <= math.radians(c["track_yaw_gate_deg"])
            and age <= c["descent_max_visual_age_s"] + _EPS
        )
        descent_safe = (
            xy_error <= c["descent_position_gate_m"]
            and _norm(relative_velocity) <= c["descent_velocity_gate_m_s"]
            and abs(yaw_error) <= math.radians(c["descent_yaw_gate_deg"])
            and age <= c["descent_max_visual_age_s"] + _EPS
        )
        if self.phase == "SEARCH":
            self._enter("ALIGN")
        bearing_error = math.atan2(position[1]+self.marker_offset[1], position[0]+self.marker_offset[0]) + yaw_error
        bearing_error = (bearing_error+math.pi) % (2*math.pi)-math.pi
        if self.phase == "ACQUIRE":
            if self._held(now_s, abs(bearing_error) <= math.radians(self.search["bearing_tolerance_deg"])
                          and age <= c["descent_max_visual_age_s"] + _EPS,
                          self.search["acquire_hold_s"]):
                if self.passive_wait:
                    self._lineup_x_m = self.search["lineup_standoff_m"]
                self._enter("LINEUP" if self.passive_wait else "ALIGN")
        if self.phase == "ALIGN":
            if self._held(now_s, abs(yaw_error) <= math.radians(c["align_tolerance_deg"])
                          and age <= c["descent_max_visual_age_s"] + _EPS,
                          c["align_hold_s"]):
                self._enter("LINEUP" if self.search["enabled"] else "APPROACH")
        elif self.phase == "LINEUP":
            aligned = (abs(position[1]) <= self.search["lineup_tolerance_m"]
                       and abs(relative_velocity[1]) <= self.search["lineup_speed_tolerance_m_s"]
                       and abs(yaw_error) <= math.radians(c["align_tolerance_deg"])
                       and age <= c["descent_max_visual_age_s"] + _EPS)
            if self.passive_wait:
                aligned = aligned and (abs(position[0]-self._lineup_x_m) <= self.search["lineup_distance_tolerance_m"]
                                       and abs(relative_velocity[0]) <= self.search["lineup_x_speed_tolerance_m_s"])
            if self._held(now_s, aligned, self.search["lineup_hold_s"]):
                self._enter("APPROACH")
        elif self.phase == "APPROACH":
            if track_ready:
                self._enter("TRACK")
                self._hold_since_s = now_s
        elif self.phase == "TRACK":
            if self._held(now_s, track_ready, c["track_hold_s"]):
                self._enter("DESCEND")
                self._descent_reanchor = True
        elif self.phase == "DESCEND":
            upper = min(c["touchdown_height_m"], c["virtual_completion_height_m"]
                        + c["completion_height_tolerance_m"])
            if (descent_safe and self._descent_height_m <= c["virtual_completion_height_m"] + _EPS
                    and c["virtual_completion_height_m"] - c["completion_height_tolerance_m"]
                    <= height <= upper + _EPS):
                self._enter("SETTLE")
        elif self.phase == "SETTLE":
            target = c["virtual_completion_height_m"]
            near_pad_ready = (
                descent_safe and xy_error <= c["track_error_m"]
                and xy_speed <= c["track_relative_velocity_m_s"]
                and abs(relative_velocity[2]) <= c["settle_vertical_velocity_m_s"]
                and target - c["completion_height_tolerance_m"] <= height
                <= min(c["touchdown_height_m"], target + c["completion_height_tolerance_m"]) + _EPS
            )
            if self._held(now_s, near_pad_ready, c["touchdown_hold_s"]):
                self._enter("COMPLETE")
                return self._stop()

        moving = self.phase in ("APPROACH", "TRACK", "DESCEND", "SETTLE")
        ff = self.vehicle["speed_m_s"] if (self.start_vehicle or self.passive_wait) and self.test["ff_enabled"] else 0.0
        if moving:
            desired = [ff + c["xy_kp_per_s"] * position[0] + c["xy_kd"] * relative_velocity[0],
                       c["xy_kp_per_s"] * position[1] + c["xy_kd"] * relative_velocity[1], 0.0]
        elif self.search["enabled"] and self.phase in ("ACQUIRE", "ALIGN", "LINEUP"):
            progress = (max(0., 1.-abs(bearing_error)/math.radians(self.search["translation_bearing_gate_deg"]))
                        if self.passive_wait and self.phase == "LINEUP" else 1.)
            desired = [ff + progress*c["xy_kp_per_s"] * (position[0]-self._lineup_x_m) + c["xy_kd"] * relative_velocity[0],
                       progress*c["xy_kp_per_s"] * position[1] + c["xy_kd"] * relative_velocity[1] if self.phase == "LINEUP" else 0., 0.]
        else:
            desired = [0.0, 0.0, 0.0]
        freeze_z = False
        forbid_down = False
        if self.phase in ("DESCEND", "SETTLE"):
            if not descent_safe:
                self._pause()
                freeze_z = True
            else:
                target = c["virtual_completion_height_m"]
                if self.phase == "DESCEND":
                    if self._descent_reanchor:
                        self._descent_height_m = max(target, height)
                    elif self._descent_step_s is not None:
                        self._descent_height_m = max(target, self._descent_height_m
                                                    - c["descent_m_s"] * (now_s - self._descent_step_s))
                    self._descent_reanchor = False
                    self._descent_step_s = now_s
                    feed_down = -c["descent_m_s"] if self._descent_height_m > target + _EPS else 0.0
                    desired[2] = feed_down + c["height_gain"] * (position[2] + self._descent_height_m)
                else:
                    desired[2] = c["height_gain"] * (position[2] + target)
                # 가상 목표에 접근하면 하강 속도를 줄여 정상 가속도 제한으로 정착한다.
                down_limit = min(c["descent_m_s"], c["height_gain"] * max(0.0, height - target))
                desired[2] = max(-down_limit, desired[2])
                forbid_down = height <= target
        else:
            desired[2] = c["height_gain"] * (position[2] + c["cruise_height_above_pad_m"])
        steer_error = bearing_error if self.phase == "ACQUIRE" or (self.passive_wait and self.phase == "LINEUP") else yaw_error
        yaw_rate = 0.0 if abs(steer_error) <= math.radians(c["deadband_deg"]) else c["kp_per_s"] * steer_error
        yaw_limit = math.radians(c["max_rate_deg_s"])
        yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))
        command = self._limit_velocity(desired, dt_s, freeze_z, forbid_down)
        return self._result(command, yaw_rate)
