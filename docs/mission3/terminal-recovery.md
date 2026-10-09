# 미션3 블라인드 종단 복구

검증 시각: 2026-10-08 03:47 KST. 변경 근거는 [제어 리뷰 7.5절](control-review.md#75-025443-strict-focus-last-fix-bridge-후보-seed-1-arc-3253m)의 025443 실행이다. 이 문서는 제어 명령 실행 시험이며 물리 착륙·제조사 사양 검증 결과가 아니다.

## 변경 이유와 종료 조건

기존 제어기는 `terminal_max_prediction_s=6`이 지나면 블라인드 예측을 거부하고, 마지막 신선 영상 이후의 시간이 `loss_reacquire_s=2`를 넘었다는 이유로 즉시 `ABORT`했다. 저고도에서도 명령은 세계 속도 `[0,0,0]`이었다. 영상 손실 후 상승·후퇴 명령을 실행하기 전에 종료하는 문제였다.

수정 후 블라인드 실패는 `BLIND_RECOVER → BLIND_RISE → ABORT`로 처리한다. 최초 실패 이유를 고정하며 복구 중 새 영상이 와도 하강을 다시 시작하지 않는다.

- `BLIND_RECOVER`: 마지막 **신선 영상**에서 저장한 차량 속도·방향 추정의 복사본을 쓴다. 차량 축 뒤로 0.12m/s, 위로 0.08m/s를 명령하고 기존 전체 속력·수평 가속도 제한을 적용한다. 진행 중이던 하강 명령은 즉시 0 이상으로 올린다. FF 예산은 기본 2.0초이며 `terminal_recovery_ff_s`로 명시할 수 있다.
- `BLIND_RISE`: FF 예산이 끝나면 복구 객체의 차량 속도 벡터를 폐기한다. 그 시점의 **자기 센서 추정 수평 속도**를 최대 명령 속력으로 제한한 뒤 선형 감속한다. 감속 시간은 `max(0.5초, 속력 / max_accel_m_s2)`다. 기본 0.70m/s·0.50m/s²에서는 최대 1.4초다. 이후 수평 명령은 0이고 상승 명령은 계속 양수다. 차량 상태를 더 외삽하지 않는다.
- `ABORT`: 추정 세계 높이가 아래 기준에 도달한 경우에만 실패 종료와 세계 속도 0을 명령한다. 복구가 시작될 때 이미 기준 이상이면 즉시 종료할 수 있다. 어느 경로도 `motors_off=True`나 물리 성공을 생성하지 않는다.

높이 기준은 공개 정적 기하로 계산한다.

```text
마커판 상단 = landing_geometry.pad_height_m
              + marker.known_marker_to_pad_vehicle_m[2]
              + marker.board_side_m / 2
복구 종료 세계 높이 = 마커판 상단 + 0.10m
```

현재 기하에서는 `0.30 + 0.10 + 0.10 + 0.10 = 0.60m`다. `terminal_recovery_world_height_m`을 주면 이 기준보다 높은 목표만 허용한다. 0.10m 여유와 복구 속도·시간은 소프트웨어 실험값이다. 추정 높이 조건은 실제 기체·발·카메라 장착 오차나 충돌 안전을 보장하지 않는다.

상승이 실행되지 않아 추정 높이가 기준에 못 미치면 `BLIND_RISE`를 유지한다. 이때 오래된 차량 FF는 이미 폐기됐고, 수평 감속도 유한 시간 안에 끝난다. 따라서 **차량 추측항법은 유한하지만, 높이 증거가 없는 복구를 완료됐다고 처리하지 않는다.** 정해진 시각에 저고도 세계 속도 0 또는 공중 모터 off로 끝내는 fallback은 없다. 메인 소유의 물리 실행기는 미션 시간 예산이나 종료 루프 때문에 이 상승 명령 처리를 먼저 끊지 않도록 별도로 다뤄야 한다.

## 블라인드 시간·직선 여유

블라인드 시작부터의 절대 시간과 마지막 영상 나이를 모두 제한한다. 둘 중 하나가 6초에 도달하면 `terminal_prediction_expired` 복구를 시작한다. 단계 중 새 영상이 들어와도 절대 타이머는 리셋하지 않는다. 예측 상태 누락·coarse bearing만 남은 경우는 `terminal_state_unavailable`로 복구한다. 블라인드 중 미션 timeout·자기 위치 불확실도 초과·추정 경기장 경계 위반도 같은 상승 경로로 넘긴다.

`TRACK → BLIND_APPROACH` 진입 여유는 다음과 같다.

```text
예측 이동거리 = |추정 차량 수평 속도|
                × (max(남은 하강시간, terminal_max_prediction_s)
                   + post_motor_off_hold_s)
직선 잔여 거리 > 예측 이동거리 + 기존 지도 여유 0.50m
```

기본 설정은 최소 6초 + 3초 유지, 조건부 물리 설정은 최소 12초 + 3초 유지를 포함한다. 기존 코드에 누락됐던 블라인드 접근 시간을 덮는다. 블라인드 진행 중에도 남은 절대 예측 예산 + 3초에 대한 지도 여유를 다시 검사하며 실패하면 `terminal_straight_margin`으로 복구한다. 영상 기반 차량 각속도 추정이 직선 임계값을 넘으면 `terminal_straight_unconfirmed`로 복구한다.

차량 속도·방향·패드 위치는 관측기 추정 입력이다. 제어기는 `Stadium.pose`나 plant의 현재 곡선 여부를 읽지 않는다. 영상 없는 구간의 각속도는 마지막 관측 추정이므로, 실제 새 곡선 진입을 관측했다고 주장하지 않는다. 지도·자기 위치와 차량 속도 추정 오차도 기존 한계로 남는다.

## Flow 계약과 하강 보류

추정기의 `own['flow_fusion_usable']`과 `own['flow_fusion_reason']`을 함께 사용한다. 블라인드 하강에 허용하는 것은 **usable=True이고 reason=pad_patch_contained인 표본**이다. 필드가 빠져도 사용 불가로 처리한다.

| 이유 | 블라인드 하강에서의 의미 |
| --- | --- |
| `surface_unconfirmed` | 표면/전환 근거 불충분. 하강 보류 |
| `floor_patch_clear` | 바닥 Flow 융합이 가능해도 이동 상판 상대 Flow가 아님. 하강 보류 |
| `floor_patch_overlap_or_invalid` | 바닥 패치가 상판과 겹치거나 센서 무효. 하강 보류 |
| `pad_patch_contained` | usable=True일 때 상판 상대 Flow 근거로 허용 |
| `pad_patch_mixed_or_invalid` | 혼합 패치 또는 센서 무효. 하강 보류 |

`BLIND_APPROACH`에서는 가장자리 횡단의 순간 혼합을 허용하며 하강 Flow 유예 시간을 누적하지 않는다. `BLIND_DESCEND`에서 사용 가능한 상대 Flow가 끊기면 즉시 하강을 보류한다. 연속 손실이 기본 0.25초(`terminal_flow_grace_s`)에 도달하면 `terminal_relative_flow_unusable` 복구를 실행한다. 유예 안에 상판 Flow가 돌아오면 타이머를 지우고 준비 게이트를 다시 검사한다. 복구 이벤트에는 마지막 Flow 이유를 기록한다.

Flow 순도/FoV는 메인 추정기의 조건부 패치 가정이다. 이 변경으로 제조사 Flow 덱 유효성을 검증한 것은 아니다.

## 공개 기하와 조건부 물리 높이

제어기의 모든 하강·종단 높이 계산에서 하드코딩된 상판 0.30m를 `cfg['landing_geometry']['pad_height_m']`로 교체했다. `LOWER_ALIGN`의 목표와 높이 준비 게이트는 `self.c.get('lower_align_world_height_m', .50)`를 사용하며 이후 APPROACH/TRACK/BLIND_APPROACH 높이 유지에도 같은 값을 사용한다. 목표를 바꿔도 접근 단계가 다시 0.50m로 상승하지 않는다.

[조건부 물리 설정](../../config/mission3-physical-sensitivity.json)의 기존 `controller_software` override는 다음과 같으며 제어기에 실제 적용되는 것을 시험했다.

```json
{"terminal_max_prediction_s": 12.0,
 "lower_align_world_height_m": 0.40,
 "geometry_terminal_optical_height_m": 0.06}
```

이 값은 남은 실제 발 높이까지의 하강 시간을 확보하는 조건부 실험값이다. 이번 worker는 물리 설정·runner를 수정하지 않았다. 기본 [광학 설정](../../config/mission3.json)의 6초·0.50m·0.08m를 유지한다. `GEOMETRY_READY`는 여전히 실제 접촉 판정이 아니다. 공개 설정 필터와 `vehicle_plant_only` 입력 거부 계약도 유지된다.

## 변경 전후 실행 검증

[새 테스트](../../scripts/mission3/test_terminal_recovery.py)는 실제 `Mission3.update()` 명령을 20ms씩 실행하고, 출력 속도를 자기 관측 fixture 위치에 적분한다. 센서·영상 추정 입력 fixture만 사용하며 물리 plant나 접촉 성공을 만들지 않는다. 차량 속도 fixture는 0.37/0.46m/s, 방향은 두 직선의 0/π 추정값이다. 설정 차량 속도 0.25m/s를 FF로 사용하지 않는다.

같은 최종 21개 테스트를 변경 전 제어기와 변경 후 제어기에 각각 실행했다. 변경 전 원문은 `/tmp/mission3-terminal-before-mtx67_jx/controller.py`로 보존하고 별도 프로세스에서 `mission3.controller_before`로 불러 비교했다.

| 검증 | 변경 전 | 변경 후 |
| --- | --- | --- |
| 신규 복구 테스트 메서드 | 2/21 통과, 19/21 실패 | 21/21 통과 |
| 하위 사례 포함 실패 assertion | 24개, 실행 오류 0개 | 0개, 실행 오류 0개 |
| 기존 계약 테스트 | 기존 동작 보존 대상 | 8/8 통과 |
| 기존 평가기 테스트 | 수정 범위 밖 | 26/26 통과 |
| 통합 unittest 실행 | — | 55/55 통과 |

핵심 시험은 전체 6초 강제 만료와 하강 보류, 0.6초 단축 예산, 신선 영상이 들어오는 동안의 절대 만료, 모든 Flow 이유·필드 누락·짧은 혼합 후 정상 재개, 곡선 근처 양방향 진입 거부, 블라인드 중 지도 여유/추정 각속도 위반의 실제 복구, 새 영상에 의한 위험 하강 재시작 금지, FF 폐기 후 정지한 높이 fixture의 계속 상승 명령, 비기본 상판 높이 및 물리 높이 override를 포함한다. 모터 off·물리 성공은 모든 실행 표본에서 False임을 검사한다.

전체 6초 만료 시험의 적분된 명령 이력은 다음과 같다. BLIND_DESCEND는 t=10초에 시작하고, 상대 속도 게이트 위반으로 하강을 보류한다. 아래 높이는 해당 명령을 적용한 fixture 적분값이다.

| 시각 | 변경 전 | 변경 후 단계 | 변경 후 추정 높이 | 변경 후 세계 속도 명령 (m/s) | done |
| --- | --- | --- | --- | --- | --- |
| 16.00s | ABORT, z=.390m, v=[0,0,0] | BLIND_RECOVER | .390111m | [.361681,0,.005547] | False |
| 17.00s | 이미 종료 | BLIND_RECOVER | .460849m | [.250000,0,.080000] | False |
| 18.00s | 이미 종료 | BLIND_RISE | .540849m | [.250000,0,.080000] | False |
| 18.76s | 이미 종료 | ABORT | .600049m | [0,0,0] | True |

종료 사유는 `terminal_prediction_expired`다. 별도 상승 정지 fixture에서는 복구 6초 이후에도 done=False, 수평 명령 0, 상승 명령 양수를 확인했다. 이는 높이가 확인되지 않은 상태에서 오래된 차량 FF나 공중 모터 off를 계속 사용하는 문제를 막는 시험이다.

재실행 명령과 환경:

```bash
cd ./scripts
../src/crazyflie-ros2-sil/.venv/bin/python -m unittest \
  mission3.test_terminal_recovery mission3.test_contracts mission3.test_evaluation
```

팀 baseline의 프로젝트 `.venv`를 사용했다. `/src/crazyflie-ros2-sil/.venv/bin/python`은 이 호스트에 없으며 실제 절대 경로는 `./src/crazyflie-ros2-sil/.venv/bin/python`이다. 패키지 설치나 시스템 환경 변경은 없었다.

검증 대상 SHA-256:

| 파일/시점 | SHA-256 |
| --- | --- |
| controller.py 변경 전 | `f884b851baca2bcbfc7cc1750c3396be3d42d2126faa0bc7d5a5f45a13af0602` |
| controller.py 변경 후 | `24787756cf23f06d122becd99c699f7dccb180a6e07bf41d3672d6609adcb715` |
| test_terminal_recovery.py | `a40cfed6a1ebb5a46d18638f4b61e1d76c4620bb227330dc866f042c8e7c22e1` |
| test_contracts.py (무수정) | `9e56fe3c16980061a7d0949aa2d07c56f20c5d9029de6b544e5a0f0589226990` |
| config/mission3.json (무수정) | `066e67aaf2e15d764d4e444c0f29be4d7ec9d847ef2070558b9fc612955be5a4` |
| config/mission3-physical-sensitivity.json (무수정) | `dcbeaabeb4b91a4b784368a3561f0ff3969419819e12c0d271b162392f6442c3` |

이 결과는 블라인드 제어 복구 계약 통과다. 실제 센서 오차 아래의 상승·후퇴, runner 시간 예산과 안전 종료, 실제 접촉·모터 종료·3초 유지 및 독립 리뷰는 메인 통합 검증 범위에 남는다. 이 worker의 시험으로 물리 성공 분자를 늘리지 않는다.
