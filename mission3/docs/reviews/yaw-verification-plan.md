# yaw 보정 구현 전 검증 계획

작성일: 2026-10-04 (Asia/Seoul). 사용자 요청에 따른 설계 검토의 통합 검증 계획이다.
이 문서는 통과 결과가 아니며 시뮬레이터/기체에 명령을 보내지 않고 작성했다.

## 현재 증거의 범위

- `artifacts/landing-visibility.json`: 320×320 이상적 카메라의 지정 경로에서 201프레임을 관측했다.
  기체 동역학과 yaw 제어는 없다. `marker_rotation_error_deg`는 전체 3차원 회전 오차다.
  yaw 오차로 바꾸어 보고하지 않는다.
- 그 기록에서 전체 회전 오차 중앙값은 약 0.631°, 최댓값은 약 19.731°다.
  5°를 넘는 프레임은 57개다. 이는 그 명목 경로의 결과이며 다양한 비행 조건의 실패율이 아니다.
- 최악 착륙점 위치 오차 프레임 t=1.55초에서는 재투영 오차 약 0.121px와
  전체 회전 오차 약 19.731°가 함께 나타났다. 낮은 재투영 오차만으로 자세 품질을 판정하기 어렵다는 증거다.
- 팀 ROS 설치 검사에서는 324×244 영상·pose·Flow·ToF 수신을 확인했다.
  ArUco 자세, 촬영 시각 동기화, yaw 명령의 응답은 그 검사에 포함되지 않았다.

수치와 검증 경계는 `artifacts/yaw-design-review/20261004/existing-evidence.json`에 저장했다.

## 구현 준비에서 확인한 문제

공유 ROS `.venv`에는 NumPy 2.2.6이 설치되어 있다. `cv2`는 Ubuntu system-site의
`/usr/lib/python3/dist-packages/cv2.cpython-310-x86_64-linux-gnu.so`로 검색되며,
실제 import에서 `_ARRAY_API not found` / `numpy.core.multiarray failed to import`로 실패한다.
재현 결과는 `artifacts/yaw-design-review/20261004/shared-vision-import.json`에 있다.

이 문제는 현재 카메라 수신 baseline이 실패했다는 뜻이 아니다. 아직 검증하지 않았던
영상 인식 기능을 같은 환경에 추가할 때의 실행 차단 요인이다.
[NumPy 공식 설명](https://numpy.org/doc/stable/user/troubleshooting-importerror.html#downstream-importerror-attributeerror-or-c-api-abi-incompatibility)은
이 오류를 종속 확장 모듈의 NumPy ABI 불일치로 설명한다.
팀 고정 NumPy를 임의로 낮추지 않고, 해당 NumPy와 호환되는 OpenCV/ArUco 확장 구성을
프로젝트 별도 의존성으로 기록하고 검사하는 방법을 후속 구현 때 선택한다.
이번 분석에서는 패키지 설치나 소스 변경을 하지 않았다.

별도로 보존한 기존 루트 `.venv`에서는 OpenCV 4.14.0 / NumPy 2.2.6의 import와
`ArucoDetector`·`solvePnPGeneric` API 존재를 확인했다.
기록은 `artifacts/yaw-design-review/20261004/legacy-vision-import.json`이다.
그 환경이 작동한다는 사실만으로 공유 ROS 환경과 섞어 사용해도 된다고 판단하지 않는다.

기존 `scripts/aruco_pose.py:102`의 `detect_pose()`는 두 PnP 후보를 내부에서 계산하지만
반환값은 최소 재투영 오차의 단일 `PoseEstimate`다. 왜곡도 이상적 렌더 규약으로 0을 사용한다.
따라서 설계안의 후보 유지·시간 일관성·왜곡 보정은 아직 구현되지 않았다.
현재 함수를 그대로 ROS 제어기에 붙이는 것으로 설계안이 완성되지는 않는다.
팀 기본 `scene.xml`에는 바닥과 배경 등이 있으며 ArUco 표적은 정의하지 않는다.
정지 마커 시험 장면과 현재 카메라 모드에 맞는 투영 모델도 별도로 준비해야 한다.

## 권장 시험 순서

팀의 기본 launch는 `--flowdeck`를 전달하며 `--sensor-noise`는 전달하지 않는다.
고정 시뮬레이터는 `args.sensor_noise`가 참일 때만 `SensorNoiseModel`을 생성한다.
따라서 기본 실행만으로 자이로 bias/드리프트에 대한 보정 효과를 검증했다고 주장하지 않는다.
gyro bias, 시간에 따른 bias 변화, 영상 손실은 후속 검증에서 명시적인 교란 조건으로 넣고
교란값을 실험 설정으로 기록한다. upstream noise 코드의 수치도 제조사 출처를 확인하기 전
실기체 사양으로 채택하지 않는다.
근거: `single_cf.launch.py`의 ExecuteProcess 인자와 `crazysim.py`의 `args.sensor_noise` 생성 분기.

| 순서 | 시험 | 실제로 확인할 항목 |
| --- | --- | --- |
| 0 | 목표·좌표계 계약 | 초기 방향 유지, 차량 축과 평행, 마커 중심 보기 중 목표 선택. 출력 각도 기준과 단위·부호 고정 |
| 1 | 영상 인식 import 및 정적 영상 | 공유 baseline을 보존한 호환 OpenCV 구성, ArUco ID/모서리 검출. 324×244 모드에 맞는 K·왜곡·카메라 변환 |
| 2 | 회전 없는 횡이동 관측 | 화면 중심 변화만으로 yaw 보정값을 만들지 않는지. 차량 축 기준 yaw와 마커 방위를 별도 출력 |
| 3 | 회전·기울기·거리 관측 | yaw-only 오차, 전체 회전 오차, 후보 모호성, 체계적 오프셋. 정면·측면·작은 투영 크기 조건 구분 |
| 4 | 영상 지연·누락을 넣은 오프라인 추정 | 촬영/수신/IMU 시각, 측정 나이, 과거 상태 보정, 재예측, 후보 거절 및 불확실성 증가 |
| 5 | 정지 바닥의 작은 양·음 yaw 명령 | ROS→CFLib→firmware 단위·부호, 실제 회전 방향, 초기 이륙 명령에서 제어권 인계, 명령 중단 동작 |
| 6 | 정지 표적·호버 폐루프 | 작은 초기 yaw 오차 회복, overshoot/정착시간, 수평 drift, 포화, 시야 이탈. 기본 IMU 유지와 비교 |
| 7 | 차량 0.3m/s 직진 | 상대 yaw 유지와 차량 추종, Flow 관측 표면의 의미, 지면→차량 전환. 하강·접촉은 추가 시험 |

위 표는 향후 구현 순서이며 이번 검토에서 2~7번을 실행한 결과가 아니다.
일반 기체 시뮬레이션 값과 제조사 확인값을 구분하고, 시뮬레이션 참값은 평가에만 사용한다.

## 기록할 지표와 판단

기수 방향 오차와 yaw 추정 오차를 별도로 기록한다. 실제 방향 유지가 좋아져도
firmware 내부 yaw 추정이 같은 기준으로 보정되었다는 증거가 되지 않는다.
각도 오차에는 반드시 기준 좌표계, 평가 참값, wrap 정의를 함께 명시한다.

관측 후보, 신뢰도/불확실성, 영상 촬영·수신 시각, IMU 시각, 명령 시각,
실제 상대 yaw, 출력 yaw_rate, 포화 시간, 검출 누락, 수평 위치 drift를 저장한다.
누락 처리 상태와 재획득 후 각도 급변도 기록한다.

수치 이득·제한·합격 문턱은 시험 전에 내부 실험 설정으로 정하고,
제조사 공개 성능 또는 대회 합격 기준처럼 표현하지 않는다.
오프라인 관측 검사를 통과하기 전에 불확실한 시각 yaw를 비행 폐루프의 강한 기준으로 사용하지 않는다.
