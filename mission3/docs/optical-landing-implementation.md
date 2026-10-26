# 영상 폐루프 차량 추종·하강 첫 구현

2026-10-04. **흑백 320×320 영상 → 마커 관측 → FF+피드백 속도 명령 → 이동 → 다음 영상**의 순환을 구현하고 실행했다.
기존 가시성 시험과 달리 카메라 경로를 지정하지 않는다. 목표와 정렬·하강 전이는 영상에서 얻은 상대 상태로 결정한다.
이번 플랜트는 명령을 완벽하게 따라가는 **카메라 광학점 운동학**이다. 제조사 미공개 기체 관성·추진·장착값을 대입하지 않았다.
실제 드론 이륙·물리적 접촉 착륙·제공 기체 성능 검증은 미완료다.

현재 기본 실행은 사용자 그림에 맞춘 [고정 대기·차량 통과 모드](optical-passby.md)다.
아래 원래 기준선을 재실행하려면 `--scenario baseline`을 지정한다.

같은 날 처음에는 시선과 차량 뒤쪽 기준선의 작은 각 70°인 조건을 회전 탐색으로 잘못 해석했다.
회전 탐색→마커 추적→방향 정렬→횡방향 정렬을 기존 접근 앞에 넣었으며,
이 모드에서는 차량이 최초 마커 발견 시점부터 움직인다.
그 시험은 [과거 회전 탐색 기록](optical-acquisition-70.md)으로 보존한다.
아래 시작 heading 5°·정렬 이후 차량 이동 조건과 명목 결과는 원래 기준선 기록이다.

## 실행

프로젝트를 설치한 이 PC에서 실행한다. 외부 자료 변경·업로드·저장소 push는 수행하지 않았다.

```bash
# 빠른 영상 폐루프: 결과 영상·그래프용 CSV 생성
bash ./scripts/run_optical_landing.sh --scenario baseline

# 같은 제어기의 공유 CPX + ROS 영상/속도 명령 경로 (약 25초)
bash ./scripts/run_optical_landing_ros.sh --scenario baseline

# FF를 제거한 비교. 뒤처짐 때문에 하강하지 않는 결과를 검사
bash ./scripts/run_optical_landing.sh --scenario baseline --no-ff

# 하강 중 짧은 영상 누락 후 재개 / 긴 누락 후 중단
bash ./scripts/run_optical_landing.sh --scenario baseline --dropout 0.6
bash ./scripts/run_optical_landing.sh --scenario baseline --dropout 2.5
```

빠른 시험은 시뮬레이션 시간을 사용한다. ROS 시험은 PC monotonic 시간과 수신 시각으로 동작한다.
`artifacts/optical-landing/<시각>/` 또는 `artifacts/optical-landing-ros/<시각>/`에
`mission.mp4`, `truth.csv`, `observations.jsonl`, `commands.jsonl`, `evaluation.json`, 설정·코드 SHA-256을 저장한다.
분석 환경의 `imageio-ffmpeg==0.6.0`으로 기존 AVI를 MP4로 내보낸다. 시스템 패키지나 ROS 환경에는 설치하지 않았다.
그림 생성은 `build/yaw-analysis-venv/bin/python scripts/plot_optical_landing.py <결과 폴더>`다.
두 시험은 실제 기체 명령을 발행하지 않는다. ROS 명령은 `/drone3/optical/cmd_velocity`로만 발행하며 가상 광학점이 이를 소비한다.

## 설정·관측·명령

차량은 정렬 완료 전 정지하고 이후 설정한 **0.3m/s +X 직진**을 한다.
상판은 30×30cm, 높이 20cm, 앞 깃발 마커는 임시 15cm, 중심은 상판 기준 `(0.45, 0, 0.15)` m이다.
이 차량·마커 수치는 시험 조건이며 기체 사양이 아니다. 시험 카메라 광학점의 시작 높이는 world 0.5m,
상판 위 0.3m, lateral 2.5cm, 가상 heading 5°다. 실제 AI-deck의 장착점/피치라고 해석하지 않는다.

영상은 처음부터 320×320 단색 센서 출력으로 생성하고, 입력 해상도·encoding·행 길이·데이터 길이를 검사한다.
공식 단색 모듈의 공칭 FOV 87°로 만든 이상적 K를 사용한다. 실제 캘리브레이션·왜곡·노출·rolling shutter·blur는 없다.
두 IPPE 후보를 보존하고, 이 시험대에서 정의한 **가상 optical heading의 roll/pitch=0** 조건으로 yaw·translation을 적합한다.
실기체의 roll/pitch가 0이라고 가정하거나 미확정 카메라→기체 변환을 만드는 것이 아니다.
마커 축으로부터 알려진 시험 마커→상판 오프셋을 빼서 광학점 기준 상대 위치를 얻는다.
수학적 방향·PnP 후보의 근거는 [OpenCV 공식 문서](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)를 따른다.

상대 위치 αβ 필터는 영상 시각을 보존하고 제어 호출 시 현재 위치를 예측한다.
`stamp_s`는 마지막 승인 영상의 시각, `prediction_stamp_s`는 위치를 예측한 시각이다.
같은 프레임을 50Hz에서 재사용해도 영상 freshness가 연장되지 않으며 위치를 이중 예측하지 않는다.
역순·비유한·미래 시각·큰 위치/yaw innovation은 거부한다.
공유 CPX에는 촬영 timestamp가 없으므로 ROS 이미지 수신 stamp와 설정된 근사 추가 지연을 사용한다.
실제 촬영 동기화가 검증된 것은 아니다. 빠른 시험의 전달 지연은 50ms로 지정했다.

상대 위치 `e = pad − camera`, 상대 속도 `e_dot`에서 수평 명령은
`v_cmd = [0.3, 0] + 0.7*e_xy + 0.5*e_dot_xy`다.
0.3은 **알고 있는 시험 차량 설정값 FF**이며 영상에서 추정한 차량 속도 FF가 아니다.
heading 오차는 yaw rate P 명령으로 보정한다. 3축 속도 norm 0.55m/s, 가속도 0.50m/s²,
yaw rate 15°/s를 제한한다. 제어 이득·제한은 실험 설정이며 제조사 성능 보장값이 아니다.
이 운동학 시험에는 Flow 출력·그 표면 보상·기체 내부 PID가 들어가지 않는다.

## 상태 전이·실패 처리

`SEARCH → ALIGN → APPROACH → TRACK → DESCEND → SETTLE → COMPLETE`를 구현했다.
정렬은 heading ±2°를 1초 유지할 때 종료한다. 추종은 상대 위치 ≤4.5cm, 상대 속도 ≤6.5cm/s,
지정 높이·heading 조건을 2초 유지할 때 하강으로 전환한다.
하강은 최대 3.5cm/s로 목표 optical height 3cm까지 진행한다.
위치·속도·heading·영상 freshness 조건이 부족하면 하강을 멈춘다.
COMPLETE는 영상으로 추정한 약 3cm 높이에서 안정 추종을 확인한 **near_pad_ready**이지 접촉/모터 정지가 아니다.
종료 명령 0은 시험 runner를 끝내기 위한 것이다. 움직이는 차량 위 실제 기체의 모터 명령으로 사용하지 않는다.

영상 age >150ms이면 하강/안정 확인을 금지하고, >400ms이면 속도 명령을 0으로 만든다.
2초 지속 손실은 ABORT이며 성공으로 기록하지 않는다. 짧은 손실 후에는 현재 추정 높이에서 하강을 다시 시작한다.
설정상 65초 임무 제한을 둔다. FF 없는 비교만 30초 제한으로 같은 오차가 유지되는지 본다.
실제 기체의 손실 복구·안전 착륙은 별도로 설계해야 한다.

편집 가능한 블록도는 [optical-landing-control.mmd](diagrams/optical-landing-control.mmd)다.
실기체의 low-level setpoint 지속 전송·high-level 우선순위는 [Bitcraze 공식 Commander 설명](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/)을 따르는 후속 adapter에서 처리해야 한다.

## 실행 결과

모든 기준은 `config/moving-landing.json`의 `optical_evaluation`에 기록했다.
참값 위치/속도는 별도 CSV로만 저장하고 관측기·제어기에 제공하지 않았다.
기본 기하 시험의 내부 기준은 중심 오차 ≤7cm, 상대 속도 ≤0.10m/s, terminal optical height 3cm±1.5cm,
차량 이동 ≥2m, 영상 유효 비율 ≥50%다. ROS에서는 종료 전 1초 구간을 평가했다.

| 시험 | 결과 | 검증 범위 |
| --- | --- | --- |
| FF+피드백, 영상 폐루프 | 20.36초 COMPLETE, 차량 5.262m 이동, 최종 중심 오차 1.85mm, 실제 optical height 3.565cm | 이상적 광학점 추종·하강 |
| 같은 조건 FF 제거 | 30초 timeout, 약 41.5cm 뒤처짐, 하강 시작 안 함 | 현재 P/D 피드백의 정상상태 뒤처짐 |
| 하강 중 영상 0.6초 누락 | 하강 정지 후 복구, 23.70초 COMPLETE | 관측 손실 처리와 재개 |
| 하강 중 영상 2.5초 누락 | 12.72초 observation_loss ABORT | 긴 손실을 성공으로 표시하지 않음 |
| 공유 CPX/ROS 폐루프 | 21.32초 COMPLETE, 355/355 영상 승인, 명령 1,066개; 마지막 1초 최대 중심 오차 1.28mm·상대속도 3.64mm/s | ROS 영상·명령 전송 포함한 이상적 광학점 |
| 순수 제어 시험 | 7/7 통과; 지연·잡음·캐시·시계·포화·손실·하강 게이트 | 상태기계·제어 계약 |
| 이동 표면 센서 벤치 | 22/22 통과 | 가상 ray·상대 각도 flow; 실제 Flow deck 보정 아님 |

영상·그림의 대표 경로:

- [ROS 결과](../artifacts/optical-landing-ros/20261004-184209-985569/evaluation.json), [영상](../artifacts/optical-landing-ros/20261004-184209-985569/mission.mp4).
- [원본 320×320 영상과 평가용 상판/광학점 위치를 함께 표시한 검토 영상](../artifacts/optical-landing/20261004-184135-911690/review.mp4). 오른쪽 도식은 제어 입력이 아닌 종료 후 평가 자료다.
- [FF 명목 결과](../artifacts/optical-landing/20261004-184135-911690/evaluation.json), [FF 비교 그래프](../artifacts/optical-landing/20261004-184135-911690/result.png).
- [FF 제거](../artifacts/optical-landing/20261004-184147-020133/evaluation.json), [짧은 누락](../artifacts/optical-landing/20261004-184252-629289/evaluation.json), [긴 누락](../artifacts/optical-landing/20261004-184148-282134/evaluation.json).
- [이동 표면 센서 결과](../artifacts/surface-sensor-bench/results.json), [설명](surface-sensor-bench.md).
- [전체 검증 요약](../artifacts/optical-landing/validation-summary.json).

센서 벤치는 차량/가상 센서가 함께 0.3m/s로 움직일 때 flow 0,
ground 위 센서 이동은 +0.6rad/s, 이동 pad 위 정지 센서는 −1rad/s,
바닥→상판 ToF ray 거리 0.50→0.30m를 확인했다. 실제 PMW pixel 상수나 미공개 장착값은 사용하지 않았다.
공유 시뮬레이터의 기존 Flow 계산은 이동 표면 속도를 빼지 않는 반례가 확인됐다. upstream은 변경하지 않았다.

FF 제거 결과는 이 P/D 구성의 비교다. 적분 제어·추정 속도 FF·RL보다 FF가 항상 우월하다는 비교가 아니다.
ROS 수신 약 16.7fps·제어 약 50Hz였으며 그 fps는 이번 렌더/전송 시험의 값이다. 실물의 fps/지연 보장값이 아니다.
MP4는 프레임 기록을 20fps로 인코딩했으므로 ROS 영상의 재생 시간과 로그 시간은 다를 수 있다.

처음 두 실패 실행도 보존한다. `20261004-183851-003590`은 중복 프레임 때문에 ALIGN hold가 리셋되어 timeout,
`20261004-184054-791793`은 정면 PnP 후보의 초기값이 최적화 범위를 벗어나 중단됐다.
캐시 측정/예측 시각 분리와 후보 유효성/fit bounds 처리로 수정했고 기존 yaw 렌더링 회귀 검사도 통과했다.

## 아직 남은 기체 착륙 작업

1. 제조사 근거가 있는 제공 구성의 모델·카메라 표준 장착/캘리브레이션을 확정한다. 미확정값은 [provided-hardware.json](../config/provided-hardware.json)에 null로 유지한다.
2. 카메라 광학점 명령을 기체 중심/body 좌표 명령으로 변환하고, 회전에 따른 카메라 레버암 속도를 포함한다.
3. 이동 표면의 Flow 상대속도와 ToF 표면 전환이 기체 내부 추정·hover 높이 명령에 주는 영향을 반영한다.
4. 실제 기체 동역학, 차량 접촉·마찰, 착지 판정, 모터 정지, 영상 손실 시 회복을 검증한다.
5. 같은 관측/제한 조건에서 추정 차량 속도 FF와 RL을 비교한다. 이번 작업에는 RL 구현이 없다.

이 자료는 상위 제어·소프트웨어 연동의 첫 실행 결과다. mm 오차를 실기체의 착륙 정밀도로 제시하지 않는다.
