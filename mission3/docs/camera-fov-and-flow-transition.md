# 카메라 화각 표시와 바닥·차량 상판 전환

2026-10-04. 사용자 그림의 삼각형은 카메라 화각과 마커 인식 가능 영역이다.
표시를 수정하고 `passby70`의 영상 제어 시험을 다시 실행했다.
현재 플랜트는 가상 카메라 광학점의 이상적 운동학이다. Flow/ToF와 기체 내부 상태 추정기는 제어 루프에 연결되지 않는다.

## 결과물

- [최신 ROS 검토 영상](../artifacts/optical-landing-ros/20261004-202153-182926/review.mp4): 원본 흑백 320×320, 상면 화각, 측면 높이, 가상 센서 재생.
- [대기→인식→정렬·접근→하강 장면](../artifacts/optical-landing-ros/20261004-202153-182926/fov-sequence.png).
- [높이·Flow·표면 전환 그래프](../artifacts/optical-landing-ros/20261004-202153-182926/surface-replay/height-flow-trace.png), [센서 재생 검사](../artifacts/optical-landing-ros/20261004-202153-182926/surface-replay/summary.json).
- [ROS 제어 평가](../artifacts/optical-landing-ros/20261004-202153-182926/evaluation.json), [오프라인 제어 평가](../artifacts/optical-landing/20261004-202153-188384/evaluation.json), [대기·정렬 검사](../artifacts/optical-passby/validation-summary.json).

ROS에서는 마커 발견 1.484초, 후방 정렬 후 접근 22.444초, 추종 27.224초,
하강 29.224초, 상판 위 근접 안정 완료 38.944초였다.
마지막 1초의 최대 중심 오차는 약 1.95mm, 상대 수평 속도는 약 3.64mm/s,
광학점 높이는 상판 위 약 3.63–4.09cm였다. 이는 이상적 운동학의 내부 검사 수치이며 실기 성능이나 접촉 착륙 결과가 아니다.
오프라인 명목 실행과 기존 마커 미발견 중단 검사도 통과했다.

## 삼각형 표시의 의미

[제조사 AI-deck mono 카메라 자료](https://www.bitcraze.io/documentation/hardware/ai_deck_mono_camera/ai_deck_mono_camera-datasheet.pdf)의
유효 320×320, 수평·수직 화각 87°를 공칭 이상적 투영에 사용한다.
옅은 영역과 점선은 수평 화각을 나타내며 화면 바깥으로 계속 이어진다.
짙은 영역은 가상 15cm 마커가 정면을 향하고, 네 모서리 모두 영상 경계 안에 있으며,
가장 짧은 변이 검출기의 실험 기준 24px 이상인 위치다.
실제 인식 여부는 저장된 영상 검출 결과로 별도 표시한다. 화각 안에 있다는 사실만으로 인식이 보장되지 않는다.

24px은 소프트웨어 실험 설정이다. 짙은 영역은 제조사가 보장한 인식 거리나 실제 광학 성능이 아니다.
이전 표시의 임의 고정 거리 상한을 제거했다. 제조사가 명시한 초점이 맞는 거리 35cm와 피사계 심도도 ArUco 최대 인식 거리로 쓰지 않는다.
실제 내부 보정·왜곡·카메라와 기체 간 장착 변환은 계속 미확정 값으로 유지했다.
드론 기호는 가상 광학점에 놓은 설명용 기호이며 실제 기체 중심·크기·데크 위치를 나타내지 않는다.

## 차 위에 있는데 높이가 유지되는 이유

LINEUP 도중 상판 위를 잠깐 지나간 뒤 후방 정렬 위치로 이동한다.
이때는 높이 유지 명령이며 최종 하강 단계가 아니다.
29.224초에 추종 안정 조건을 만족한 후 하강을 시작한다.
새 영상의 측면도는 바닥 기준 world Z와 상판 기준 높이를 따로 보여준다.

## Flow deck에서 생기는 문제

[제조사 Flow deck v2 설명](https://www.bitcraze.io/products/flow-deck-v2/)에 따르면
PMW3901은 바닥 대비 움직임을 관측하고 VL53L1x는 아래 표면까지 거리를 측정한다.
이동 상판을 바라보면 병진 Flow는 센서와 상판의 상대 이동에 영향을 받는다.
회전 보정과 거리를 올바르게 처리한 이상적 병진 사례에서는 다음과 같다.

`표면 상대 속도 = 센서 world 속도 − 관측 표면 world 속도`

드론과 차량이 모두 +0.3m/s로 동행하면 상대 병진 속도는 0이다.
이를 정지 바닥 관측으로 처리하면 world 속도와 0.3m/s 차이가 생긴다.
바닥에서 20cm 높이 상판으로 관측 표면이 바뀌면, 같은 world 높이 50cm에서도 거리 측정은 50cm→30cm로 바뀐다.
이 변화는 실제 20cm 하강과 구분해야 한다. 실제 펌웨어가 얼마나 흔들리는지는 아직 측정하지 않았다.

저장된 최신 ROS 경로에 가상 하향 probe를 재생한 결과는 다음과 같다.

| 시각 | 단계 | 관측 표면 전환 | 바닥 기준 광학점 높이 | 아래 표면까지 거리 |
| --- | --- | --- | ---: | ---: |
| 10.227초 | LINEUP | 바닥→상판 | 50.11cm | 30.11cm |
| 12.047초 | LINEUP | 상판→바닥 | 50.10cm | 50.10cm |
| 24.347초 | APPROACH | 바닥→상판 | 50.10cm | 30.10cm |

첫 전환 때 기체가 내려간 것처럼 보이는 거리 변화는 표면 높이 때문이다.
단일 광선의 두 표면 검출, 높이·거리 관계, 상판에서 −0.3m/s 속도 해석 차이,
바닥에서 속도 해석 차이 0을 검사해 4개 모두 통과했다.
검사 통과는 이 단순 가정의 재현성 확인이다. 센서 융합으로 문제가 해결됐다는 뜻은 아니다.

공유 고정 코드의 [compute_flow](../src/crazyflie-ros2-sil/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py)는
기체 속도와 ToF 거리를 사용하며 관측 표면 속도를 빼지 않는다.
펌웨어의 [Flow 측정 모델](../src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/kalman_core/mm_flow.c)은 state Z와 기체 속도로 예측하고,
[ToF 측정 모델](../src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/kalman_core/mm_tof.c)은 state Z를 경사각으로 보정한 거리와 비교한다.
이 모델에는 차량의 속도·상판 높이가 포함되지 않는다. PC의 차량 속도 피드포워드만 추가해도 내부 추정기 문제가 자동으로 해결되지는 않는다.
공유 저장소는 수정하지 않았다.

## 다음 구현 구조

1. 영상에서 얻은 차량 기준 상대 위치·방향과 관측 품질을 보존하고, IMU 및 이동 명령과 함께 상대 상태를 추정한다.
   이번 실험의 알려진 0.3m/s FF와 실제 관측으로 추정한 차량 속도는 구분한다.
2. world 높이와 아래 표면까지 거리를 분리한다. 영상 변화·IMU·ToF 잔차를 비교해 표면 전환 후보를 감지한다.
   실제 제어에 평가용 상판 ID나 차량 참값을 넣지 않는다.
3. 전환이 의심되거나 Flow·영상·IMU가 일치하지 않으면 해당 측정의 반영을 제한한다.
   PC뿐 아니라 펌웨어 추정기의 Flow/ToF 반영 경로도 다뤄야 한다.
   바닥·상판이 섞여 보이는 구간에서 상판 속도를 모든 Flow에 그대로 더하는 보정도 신뢰하지 않는다.
4. 상대 위치·속도가 안정되고 높이 관측이 신뢰되는 경우에만 하강한다.
   관측 품질이 떨어지면 하강을 멈추고, 복구 실패 시 중단한다. Flow를 끈 상태로 대기·추종이 안정된다고 가정하지 않는다.
5. 먼저 센서와 추정기를 실제로 연결한 시뮬레이션에서 기존 융합과 수정 융합을 비교한다.
   바닥→상판→바닥 재전환, 동행, 영상 유실, 표면 혼합, 거리 이상치를 포함한다.
   하강 명령·추정 높이/속도·실제 궤적의 차이와 중단 결과를 확인한 뒤 기체 동역학·접촉 시험으로 진행한다.

위 구조는 후속 구현 계획이다. 현재 완료한 것은 표시 수정, 영상 제어 재실행, 표면 센서 진단이다.
실제 Flow deck 장착 원점·방향, PMW 픽셀 보정, 회전 성분·노이즈·텍스처·반사율·차폐·혼합 반환은 이 재생에 포함되지 않는다.
진단 probe는 가상 카메라 위치에서 수직 아래를 본다고 둔 수학적 가정이며 제조사 장착 좌표가 아니다.
무개조 기체의 실제 명령 변환·내부 추정·착륙 접촉·모터 정지는 여전히 미검증이다.

## 재생과 검사

프로젝트 루트에서 공유 Python으로 실행한다.

```bash
PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/replay_surface_on_mission.py artifacts/optical-landing-ros/20261004-202153-182926
PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/review_optical_video.py artifacts/optical-landing-ros/20261004-202153-182926
PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/verify_optical_visibility.py
```

화각·투영 검사 4개는 임의 거리 제한 제거, 수평 화각과 전체 모서리 가시성의 차이,
마커 뒷면 제외, 독립 OpenCV 투영 및 좌표 회전 대조를 확인한다.
