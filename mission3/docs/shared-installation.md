# 팀 공유 저장소 설치

현재 개발 기준은 [crazyflie-ros2-sil](https://github.com/unLIMited72/crazyflie-ros2-sil)입니다.
2026-10-04에 확인한 main 커밋 `7f2f5b86d9d802865508625f6b95ad214169e188`을
`./src/crazyflie-ros2-sil`에 설치합니다.
저장소의 `manifests/upstream_sources.yaml`, 고정 Python 의존성, 제공 패치를 그대로 적용합니다.

## 구성

| 항목 | 팀 기준 |
| --- | --- |
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble Desktop |
| ROS 연결 | Crazyswarm2, CFLib backend, cf231, UDP 19850 |
| Python | 공유 저장소 `.venv`, system site packages 허용, user site 비활성 |
| MuJoCo | 3.13.0 |
| 기본 모델 | cf2x_P250 |
| 펌웨어 | Kalman estimator=2, PID controller=1 |
| Flow | 활성, 시뮬레이터의 외부 pose 주입 비활성 |
| 카메라 | CPX TCP 5050 → ROS Image, 324×244, mono8 |
| ROS 통신 | ROS_DOMAIN_ID=25, rmw_fastrtps_cpp |

GPU 상태는 `nvidia-smi`로 확인했으며 RTX 4090 Laptop GPU / 드라이버 580.178.04입니다.
ROS 2 Humble의 [공식 설치 절차](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)에 CUDA 설치 요구는 없습니다.
MuJoCo 화면과 카메라 렌더링은 [OpenGL](https://mujoco.readthedocs.io/en/stable/programming/visualization.html)을 사용합니다.
현재 공유 baseline에는 CUDA Toolkit이나 Isaac ROS가 필요하지 않습니다.

## 실행

```bash
bash ./scripts/run_shared_ros.sh
```

MuJoCo 창, firmware SITL, CPX bridge, AI-deck ROS camera, Crazyswarm2 server가 함께 시작됩니다.
종료는 실행 터미널의 `Ctrl+C`입니다. 로그는 `logs/shared-ros-<실행시각>/`에 저장됩니다.
기존 시뮬레이터가 같은 포트를 사용하면 실행 전에 알려줍니다.

다른 터미널에서 ROS 명령을 쓰려면:

```bash
source ./config/shared_ros_env.bash
ros2 topic list
ros2 topic hz /cf231/camera/image_raw
```

카메라·ROS baseline 자동 검사는 다음과 같습니다. 비행 명령은 보내지 않습니다.

```bash
bash ./scripts/run_shared_ros.sh --verify
```

## 설치·검증 기록

ROS 시스템 설치, Python 의존성, firmware SITL, ROS workspace 빌드와 수신 검증을 완료했습니다.
23개 고정 소스와 제공 패치 검사가 통과했으며 ROS 패키지 7개를 빌드했습니다.
Python 패키지 의존성 검사도 통과했습니다.
최종 상태는 `artifacts/shared-baseline-install/install-status.json`, 상세 버전은
`docs/shared-installed-versions.json`에 기록합니다.

| 실제 수신 검사 | 결과 |
| --- | --- |
| 팀 baseline verifier | PASS, 15초에 영상 274장, 18.18 Hz |
| 카메라 추가 구독 | 12초에 220장, 18.33 Hz, 324×244 mono8 |
| 기체 pose | 119개, 10.00 Hz |
| status | 12개, 1.00 Hz |
| ToF | 239개, 19.99 Hz |
| Flow | 239개, 20.00 Hz |
| 종료 / 포트 정리 | launch 종료 코드 0, 19850·19950·5200·5050 해제 |

수신 기록은 `logs/shared-ros-20261004-120205-303055/sensor-reception.json`입니다.
`Ctrl+C` 종료 시 Python node의 KeyboardInterrupt 및 종료 코드 -2/130이 launch 로그에 나타났습니다.
비행 명령과 이륙·착륙 검사는 이번 공유 baseline 설치 검사에 포함하지 않았습니다.
설치 로그는 `logs/shared-ros-system-install.log`, `logs/shared-source-setup.log`,
`logs/shared-python-setup.log`, `logs/shared-firmware-build.log`, `logs/shared-bootstrap.log`입니다.

시스템 설치 재현 스크립트는 `scripts/install_shared_ros_system.sh`입니다.
소스·Python·펌웨어·ROS 재빌드는 다음 명령으로 수행합니다.

```bash
bash ./scripts/build_shared_ros.sh
```

이 스크립트는 팀의 `bootstrap.sh`를 실행한 다음 로컬 보충 의존성을 확인합니다.
재설치는 시스템 관리자 인증이 필요한 단계와 사용자 권한 빌드를 구분합니다.

현재 Ubuntu의 system-site PyNaCl 패키지가 CFFI를 요구하여 공유 `.venv`에만
cffi 1.17.1 / pycparser 2.22를 보충했습니다. 버전은 `config/shared_python_supplement.lock.txt`에 기록합니다.
팀의 고정 의존성 파일과 지정된 버전은 변경하지 않았습니다.

2026-10-04 yaw 개발에서 NumPy 2.2.6과 Ubuntu OpenCV의 ABI 충돌을 확인했습니다.
공유 venv에 `opencv-python-headless==4.14.0.94`를 `--no-deps`로 보충하여 ArUco와 실제 ROS 영상 처리를 검증했습니다.
기존 NumPy·SciPy·MuJoCo 및 팀 lock은 그대로입니다. 첫 정지 표적 yaw 비행 기록은
[`yaw-first-simulation.md`](yaw-first-simulation.md)에 있으며 위의 설치/수신 검사와 검증 범위를 구분합니다.

팀 저장소 launch는 전역 `pkill`을 사용합니다. 프로젝트 실행 스크립트는 원본을 수정하지 않고
해당 명령이 자신이 생성한 Linux session 안에서만 종료하게 제한합니다.
별도 session에 같은 이름의 프로세스를 두고 종료 대상으로 포함되지 않는 것을 검증했습니다.

## 기존 환경과 실기체 검증

기존 루트 `.venv`(MuJoCo 3.14.0), 직접 CFLib 실행, Gazebo Harmonic, ArUco 기하 시험과 설계 문서는 보존합니다.
이전 설치·비행 기록은 `docs/installation.md`, 관측 실험은 `docs/visibility-experiment.md`에 있습니다.
현재 ROS baseline과 이전 직접 CFLib 환경은 별개이며 같은 UDP 포트에서 동시에 실행할 수 없습니다.

팀 모델과 카메라 설정을 맞춘 것은 소프트웨어 통합 기준을 맞춘 것입니다.
제조사 사양·기본 장착과의 일치, ArUco 기반 자율비행, 이동 차량 착륙은 별도 검증 대상입니다.
