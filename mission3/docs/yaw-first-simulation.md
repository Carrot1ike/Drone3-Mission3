# 정지 마커 yaw 보정 첫 구현

2026-10-04. 공유 ROS 2 SIL에서 **이륙 → 양·음 회전 시험 → 초기 기수 방향 복귀 → 착륙**을 실행하는 PC 노드를 구현했다.
상위 yaw 제어가 동작하며 기체의 기존 PID·Kalman·Flow·ToF 구성을 사용한다.
현재 시험은 정지 마커에 한정한다. 이동 차량의 속도 추종, 접근, 접촉 착륙 및 firmware 내부 yaw 측정 융합은 후속 작업이다.

이 기록은 `cf2x_P250`·324 × 244 공유 기준선의 통합 시험이다. 제공 기체의 제조사 성능이나 고정 320 × 320 입력을 검증한 결과는 아니다.
현재 제공 하드웨어 적용 기준은 [provided-hardware-enforcement.md](provided-hardware-enforcement.md)를 따른다.

## 실행

```bash
# 시뮬레이션이 켜져 있으면 그 실행 터미널에서 먼저 종료한다.
bash ./scripts/run_yaw_demo.sh

# PC 관측기 입력에만 가상 오차 0.5°/s 추가
bash ./scripts/run_yaw_demo.sh --test-gyro-bias 0.5

# 호버 종료 구간에서 영상 관측 3초 차단, 착륙 복구 검사
bash ./scripts/run_yaw_demo.sh --test-image-dropout 3.0
```

한 실행은 약 1분이다. firmware·MuJoCo·CPX·ROS server·영상 처리 노드를 자동으로 시작하고,
시험 결과를 저장한 후 자신이 시작한 프로세스를 종료한다. 장비·Crazyradio는 필요하지 않다.
이 실행은 생성된 localhost UDP 설정만 허용한다. 실기체 실행용 인터페이스는 아직 제공하지 않는다.
`Ctrl+C`는 전체 가상 실행을 종료한다. 정상 시험 종료는 상태기계가 착륙·모터 정지를 확인한 뒤 수행한다.

출력은 `./artifacts/yaw/<시각>/`, ROS 로그는 `logs/shared-ros-<시각>/`에 저장한다.
`artifacts/yaw/latest.json`은 마지막 실행의 위치를 안내한다. 검증 결과의 정본은 각 실행의 `evaluation.json`이다.

## 구현 구조

| 파일 | 역할 |
| --- | --- |
| `config/yaw-static.json` | 실험 크기·위치·높이·품질·제어·내부 판정 기준 |
| `scripts/yaw_scene.py` | 정지 마커 장면 생성, 컴파일한 공유 카메라 정의로 시뮬레이션 보정 파일 생성 |
| `scripts/yaw_core.py` | 두 IPPE 후보, roll/pitch 제약 최적화, gyro 예측·시각 보정, yaw rate 계산 |
| `scripts/yaw_control_node.py` | ROS 관측·시간 버퍼·단일 발행자·이륙/시험/복구 상태기계 |
| `scripts/yaw_static.launch.py`, `yaw_sitl.sh` | 공유 server·카메라 bridge와 프로젝트 확장 실행 |
| `scripts/yaw_sim.py` | 공유 동역학 그대로 사용, 별도 평가용 참값 파일 기록 |
| `scripts/evaluate_yaw.py`, `plot_yaw.py` | 실행 종료 후 참값과 관측 로그를 비교·그림 생성 |
| `scripts/verify_yaw.py` | 렌더링한 자세/이동 25조건 및 지연·각도 wrap·회전 속도 변환 검사 |

편집 가능한 흐름도는 [yaw-static-control.mmd](diagrams/yaw-static-control.mmd)다.

### 관측과 명령

목표는 이륙 전 유효한 영상 관측으로 정의한 **초기 기수 방향 유지**다. 마커 중심을 영상 가운데로 이동시키는 목표가 아니다.
기수 방향은 정지 마커 축이 정한 시험 좌표계에서 표현한다.
같은 heading으로 좌우 8cm 이동한 렌더링 시험에서도 방향 추정은 유지됐다.

네 모서리에서 IPPE 두 해를 보존한다. 두 해 각각을 시작값으로 쓰되, 기체에서 받은 roll/pitch를 제약으로
yaw와 카메라-마커 translation을 다시 적합한다. 재투영 잔차·양의 깊이·수렴·yaw 후보 차이로 관측을 검사한다.
낮은 재투영 오차 하나로 자세를 선택하지 않는다. 정지 장면에서 이 제약이 도움되는 것을 확인했지만,
큰 기체 경사, 틀린 실물 카메라 보정, 마커 흔들림 등에서 해의 정확성을 보장하지 않는다.

gyro 로그는 `gyro.x/y/z`, `stabilizer.roll/pitch`의 50Hz 별도 블록이다. gyro deg/s를 rad/s로 변환한다.
physical pitch는 공유 pose callback과 같이 `-stabilizer.pitch`로 변환한다.
body gyro의 y/z와 roll/pitch로 Euler yaw 변화율을 구한다. 내부 pose의 **yaw 값은 관측기에 넣지 않는다**.
pose의 높이는 이륙/착륙 확인에 사용한다. gyro bias 자동 학습은 첫 버전에 넣지 않았다.

`LogDataGeneric.timestamp`의 24-bit 밀리초 카운터를 unwrap하고 PC monotonic 시계로 근사 매핑한다.
이미지 stamp는 촬영 시각이 아닌 TCP 수신·decode 후 시각이다. ROS 수신 지연과 설정한 80ms 추가 지연을 빼서
과거 roll/pitch와 gyro 적분을 조회한다. 이는 **촬영 시각 동기화의 근사**이며 실제 지연을 측정한 결과가 아니다.

50Hz로 body vx/vy=0, 설정한 world 높이, 제한된 ROS `yaw_rate`를 보낸다.
현재 이득 1.2/s, deadband 1.5°, rate 제한 15°/s는 실험 설정이다.
양수 및 음수 10°/s 명령을 각각 1.2초 전송해 실제 회전 방향을 검사한다.
현재 고정 protocol 7에서는 ROS 양수 명령에 실제 yaw가 감소했다. PC 제어기는 관측으로 확인한 부호 -1을 사용한다.
마커/gyro 응답이 서로 반대이거나 응답이 작으면 시험을 중단한다.

### 제어권과 복구

이륙 서비스 완료 시간과 기체 높이를 확인한 뒤 호버 발행자로 넘긴다.
첫 저수준 setpoint가 이전 high-level planner를 멈추므로, 종료 시 새 착륙 계획을 만든다.
마지막 rate=0 호버 → 발행 종료 및 100ms 대기 → `land` → 서비스 완료 → `notify_setpoints_stop` 순서다.
이후 호버를 다시 보내지 않는다. 이 버전의 notify는 요청의 remain_valid_millisecs를 사용하지 않는다는 점을 반영했다.

영상의 유효 관측이 0.45초 이상 없으면 회전 명령을 0으로 유지하고, 2초 이상이면 시험을 중단해 착륙한다.
IMU age 0.20초, pose age 0.5초, 명령 타이머 간격 0.25초도 별도로 검사한다.
짧은 영상 누락은 IMU 예측으로 버티며, 오랫동안 못 보는 상태의 각도를 계속 따라 돌지 않는다.

공유 firmware는 `CONFIG_MOTORS_REQUIRE_ARMING` 미설정에 따른 auto-arm 상태를 보고했다.
착륙 후 disarm 요청을 보내도 다시 arm 상태가 될 수 있다. 따라서 auto-arm 환경의 종료 확인은
지면 높이·최신 상태의 비행 bit 해제·네 모터 PWM=0으로 한다. 이를 armed=0 확인으로 표현하지 않는다.
상태와 모터를 각각 `status.csv`, `motors.csv`에 남긴다.

## 가상 설정과 제조사 검증 경계

마커는 ID0, DICT_4X4_50, 한 변 15cm, 중심 (0.55, 0, 0.25)m의 수직 표적이다.
기존 10cm 가상값보다 크게 정해 첫 관측 시험에서 충분한 픽셀 크기를 확보했다.
원래 가상 중심 높이 30cm는 지면 상태에서 영상 경계와 너무 가까워 준비 검사에서 거부되어 25cm로 조정했다.
기체·카메라 장착 정의는 변경하지 않았다. 마커 크기·위치·호버 높이 30cm는 사용자가 허용한 가상 실험 설정이다.

카메라 profile은 고정 cf2x_P250의 컴파일한 카메라에서 추출했다. 출력 324×244, fovy 70°,
초점거리 약 174.234px, 렌더러의 왜곡 0, 모델의 카메라 회전/위치를 사용한다.
**이는 제조사 AI-deck 실물 내부 파라미터·외부 장착 보정값이 아니다.**
기체 질량·관성·모터 동역학도 기존 공유 모델을 유지하며 제조사 사양 검증 결과로 주장하지 않는다.
실물 전환에는 제조사 장착 상태의 카메라 보정과 하드웨어 revision 확인이 필요하다.

기본 IMU 노이즈 옵션을 켜지 않았다. 공유 Flow 측정 모델에는 노이즈가 이미 포함되어 있다.
`--test-gyro-bias`는 PC 관측기에만 넣는 가상 측정 오차다. firmware 내부 추정기는 바꾸지 않는다.
자이로 단독과 시각 융합의 비교는 **같은 비행 데이터를 처리한 두 관측기 비교**이며 별개 제어기 A/B 비행이 아니다.
실행의 real-time factor가 약 0.85~0.89여서 MuJoCo 시간과 PC/firmware wall 시계 차이가 gyro 적분에 영향을 준다.
이를 실제 센서 드리프트라고 해석하지 않는다. 시간을 맞추는 후속 개선도 필요하다.

평가용 위치/quat 참값은 `truth.csv`에만 기록하고 관측/명령 노드는 이 파일을 읽지 않는다.
Flow가 켜진 공유 경로는 firmware에 시뮬레이터 pose를 주입하지 않는다.
프레임 영상 `camera.avi`는 수신 프레임에 20fps를 지정한 확인용 영상이며 정확한 촬영 시계 기록이 아니다.

## 재현과 다음 단계

공유 저장소와 고정 upstream 소스·패치·기존 dependency lock은 유지했다. `manage_sources.py --check`의 23개 항목과
`pip check`가 통과했다. Ubuntu OpenCV 바이너리의 NumPy 2 ABI 오류는 공유 venv에만
[opencv-python-headless 4.14.0.94](https://pypi.org/project/opencv-python-headless/4.14.0.94/)를 `--no-deps`로 설치해 해결했다.
보충 의존성은 `config/shared_python_supplement.lock.txt`에 기록한다.
후처리 그림은 별도 `build/yaw-analysis-venv`에서 [Matplotlib 3.10.8](https://pypi.org/project/matplotlib/3.10.8/)로 만든다.
이 환경의 고정 버전은 `config/yaw-analysis.lock.txt`다. ROS 런타임 환경과 섞지 않는다.

각 새 실행은 실제 설정·scene·카메라 profile·localhost YAML과 프로젝트 코드 SHA256을 저장한다.
일반 시험의 내부 기준은 방향 복귀 평균 오차 5° 이내, 수평 이동 15cm 이내, 유효 영상 50% 이상,
양/음 회전 방향 확인, 착륙 확인이다. 신규 시험은 추가로 복구 후 호버 발행 없음과 모터 출력 0을 확인한다.
긴 영상 누락 시험은 정상 yaw 시험 중단이 예상 결과이며, `visual_loss` 발생 후 착륙/모터 정지가 성공해야 통과한다.
이 기준은 개발용이며 대회·장비 지급의 공식 합격 기준이 아니다.

다음 단계는 yaw 복귀 중 과도 진동과 영상 지연 민감도를 줄이고, 같은 heading의 횡이동·거리 변화로 검증 범위를 넓히는 것이다.
그 뒤 정지 마커 기준에서 동작하는 yaw 모듈과 0.3m/s 직진 차량의 수평 속도 추종을 결합한다.
움직이는 표적의 방향과 gyro bias를 구분하는 문제를 다시 검토한 후 접근·착륙 모듈을 붙인다.

## 실행 결과

현재 버전의 일반 시험과 오차 주입 시험, 영상 누락 복구 시험이 통과했다. 표의 방향 복귀 오차는
마지막 HOLD 구간 1.5초의 실제 yaw 평균과 이륙 전 실제 yaw 평균의 차이다.
외부 위치 추적을 제어에 사용하지 않았으며 실제 yaw는 종료 후 평가 파일에서만 읽었다.

| 시험 | 결과 | 마지막 방향 복귀 오차 | 최대 수평 이동 | 기록 |
| --- | --- | --- | --- | --- |
| 정상 정지 마커 | 통과 | 0.200° | 3.59cm | [평가 JSON](../artifacts/yaw/20261004-172414-251623/evaluation.json) |
| PC gyro +0.5°/s | 통과 | 0.055° | 2.74cm | [평가 JSON](../artifacts/yaw/20261004-172827-755621/evaluation.json) |
| HOLD 중 영상 3초 차단 | 예상 중단 후 착륙·정지 통과 | 0.321° | 2.49cm | [평가 JSON](../artifacts/yaw/20261004-172143-215469/evaluation.json) |

정상 시험에서 유효 시각 관측 비율 100%, 관측기 yaw RMSE 0.601°를 기록했다.
양/음 회전 입력으로 실제 yaw 변화 -9.64° / +9.00°를 관측했다. 최대 호버 발행 간격은 35.6ms였다.
정상·오차 주입·누락 시험 모두 착륙 후 네 모터 PWM=0을 확인했다. auto-arm 상태 bit는 14였다.

gyro 오차 주입 시험의 마지막 1.5초에서 동일 비행 기록의 자이로 단독 관측기 평균 오차는 +13.18°,
시각 융합 관측기는 +0.378°였다. 전체 활성 호버 구간 관측기 RMSE는 각각 8.697° / 0.522°였다.
실제 기수 방향의 복귀 오차 0.055°와 관측기의 오차 0.378°는 서로 다른 지표다.

긴 영상 누락 시험은 마지막 유효 시각 관측 약 2초 뒤 `visual_loss`를 기록하고 착륙했다.
시각 age 0.45초 이후 회전 명령=0, 새 착륙 계획 이후 호버 발행 없음, 지면 복귀·모터 정지를 확인했다.
이 시험의 `control-summary.json`은 정상 시험을 중단했으므로 completed=false이고,
복구 평가의 `evaluation.json`은 예상 동작과 일치하므로 passed=true다.

렌더링 검사는 yaw -20°~-10°~0°~10°~20°, 좌우 이동 ±8cm, roll/pitch 조합의 25조건을 통과했다.
현재 장면에서 yaw=0 횡이동 3조건의 방향 추정 오차는 -0.006° / +0.089° / -0.090°였다.
멀리 있어 작은 마커, 카메라가 반대 방향을 본 상태, 영상과 맞지 않는 roll 10° 입력도 모두 거부했다.
지연 보정·±180° wrap·속도 제한·body gyro→Euler yaw rate 변환 검사가 함께 통과했다.
[검사 원본](../artifacts/yaw-verification/verification.json)을 보존한다.

정상 그래프: [yaw-result.png](../artifacts/yaw/20261004-172414-251623/yaw-result.png).
오차 주입 그래프: [yaw-result.png](../artifacts/yaw/20261004-172827-755621/yaw-result.png).
확인 영상: [camera.avi](../artifacts/yaw/20261004-172414-251623/camera.avi).
그래프의 검은 선은 평가용 참값, 파란 선은 시각+gyro 추정, 주황 점선은 같은 데이터의 gyro 단독 적분이다.

초기 연결 실패와 준비 거부 기록도 삭제하지 않았다. `171109` 실행은 ROS Node의 읽기 전용 clients 이름 충돌,
`171225` 실행은 마커의 영상 경계 문제로 비행 전에 종료했다.
`171830` 실행은 복구 착륙에는 성공했지만 auto-arm 환경에서 armed=0을 요구한 종료 판정이 실패했다.
현재 버전은 auto-arm을 명시하고 모터 출력·비행 상태를 확인한다. 이 실패 기록을 통과 결과로 재분류하지 않았다.
`171323`, `171545`는 초기 정상/gyro 오차 주입 비행이며 추가 모터 로그 이전 결과로 따로 보존했다.

현재 결과는 공유 모델에서의 상위 yaw 제어 첫 검증이다. 제조사 실물 성능, 모든 상황 대응,
0.3m/s 이동 차량의 추종·착륙 성공률 또는 대회 합격을 입증한 결과로 확대하지 않는다.
