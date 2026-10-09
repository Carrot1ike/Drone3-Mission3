# 설치 및 검증 기록

검증일: 2026-09-30 (Asia/Seoul).

## 환경 및 설치 위치

- Ubuntu 22.04.5 LTS / x86_64 / Python 3.10.12.
- NVIDIA GeForce RTX 4090 Laptop GPU, 드라이버 580.178.04.
- 프로젝트: `.`.
- Gazebo Harmonic: OSRF 공식 apt 저장소의 시스템 패키지.
- MuJoCo 및 CFLib: 프로젝트 `.venv`.
- CrazySim 소스 및 서브모듈: `src/CrazySim`.
- 공식 CFLib 소스: `src/crazyflie-lib-python`, editable 설치.
- 펌웨어 바이너리: `build/firmware/cf2`.
- Gazebo 플러그인: `build/gazebo/libgz_crazysim_plugin.so`.

설치 버전과 소스 커밋은 `installed-versions.json`, Python 전체 의존성은
프로젝트 루트의 `requirements.lock.txt`에 고정했습니다.
이번 작업에서 외부 소스에 패치는 적용하지 않았습니다.

## Gazebo 버전과 빌드 설정

CrazySim README는 Garden을 안내하지만, 이 프로젝트가 체크아웃한 플러그인 CMake는
Gazebo Sim 8 (Harmonic)과 7 (Garden)을 모두 찾도록 구성되어 있습니다.
Ubuntu 22.04용 공식 패키지가 제공되는 Harmonic을 설치하고 실제 비행까지 검증했습니다.

Gazebo 플러그인은 **Debug** 설정으로 빌드합니다. 이 환경에서 Release (`-O3`) 빌드는
펌웨어 핸드셰이크 뒤 CFLib UDP 응답 및 센서 초기화가 진행되지 않았습니다.
Debug 빌드로 변경한 뒤 UDP 연결, 센서 초기화, 이륙·착륙이 통과했습니다.
`scripts/build.sh`에 검증한 빌드 설정을 저장했습니다.

실행 시 리소스 경로, 플러그인 경로, Gazebo 통신 파티션을 실행 프로세스에만 설정합니다.
전역 셸 설정을 수정하지 않았습니다. Gazebo 공식 설치 저장소와 패키지는 시스템에 설치됩니다.

## 검증 결과

아래는 설치·연결 검증용 기본 모델인 MuJoCo `cf2x_T350` 및 Gazebo
`crazyflie_thrust_upgrade`로 확인한 결과입니다. 사용자 무개조 기체의 제조사 기반
Crazyflie 2.1 + AI-deck + Flow deck 모델이나 이동 차량 착륙의 검증 결과가 아닙니다.
기체 사양 정책과 공식 근거는 [제조사 사양 기록](manufacturer-specs.md)에 정리했습니다.

| 항목 | MuJoCo | Gazebo Harmonic |
| --- | --- | --- |
| 패키지 및 CrazySim 빌드 | 통과 | 통과 |
| 펌웨어 핸드셰이크 / CFLib 연결 | 통과 | 통과 |
| 0.5m 목표의 호버 높이 중앙값 | 0.5366m | 0.5357m |
| 착륙 후 높이 중앙값 | 0.0199m | 0.0150m |
| 높이 텔레메트리 | 106개 | 106개 |
| GUI 실행 유지 및 종료 | 5초 실행 통과 | 12초 실행 통과 |

높이는 펌웨어의 `stateEstimate.z` 텔레메트리이며, 독립적인 지상 실측값은 아닙니다.
비행 통과 조건은 샘플 50개 이상, 호버 높이 0.35~0.65m,
착륙 후 높이 절댓값 0.12m 미만입니다. 단일 드론의 기본 물리·센서 설정을 검증했습니다.

MuJoCo 모델의 OpenGL 오프스크린 렌더링도 통과했고 이미지를
`artifacts/mujoco-model.png`에 저장했습니다. GUI 검증은 프로세스 시작과 유지,
오류 종료 여부를 확인한 것이며 화면을 육안으로 검수한 것은 아닙니다.
Gazebo GUI 로그에는 EGL DRI2 경고가 있지만 해당 실행은 오류 종료하지 않았습니다.

현재 CrazySim 펌웨어의 CRTP 버전은 7입니다. CFLib의 supervisor API는
이 펌웨어에서 기존 플랫폼 채널로 자동 전환하며 버전 경고를 출력합니다.
두 백엔드의 arming 및 비행 검증은 이 경로로 통과했습니다.

원시 측정값:

- `artifacts/mujoco-flight.json`
- `artifacts/gazebo-flight.json`

설치·빌드·실행 기록:

- `logs/install-python.log`
- `logs/install-gazebo.log`
- `logs/build-firmware.log`
- `logs/build-gazebo.log`
- `logs/flight-mujoco.log`
- `logs/flight-gazebo.log`
- `logs/<backend>-<KST 시각>/`: 해당 실행의 프로세스별 로그와 생성 모델.

검증용 프로세스를 종료한 뒤 기본 UDP 포트 19850/19950 및
동시 GUI 검증용 포트 20850/20950이 해제됐는지 확인했습니다.

## Python 환경 복구

외부 소스가 남아 있는 프로젝트 루트에서:

```bash
cd .
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.lock.txt
bash scripts/build.sh
```

외부 소스 커밋은 `installed-versions.json`의 `sources` 항목을 기준으로 복구합니다.
Gazebo 시스템 패키지 설치 스크립트는 `scripts/install_gazebo.sh`입니다.

## 공식 참고 자료

- [CrazySim](https://github.com/gtfactslab/CrazySim)
- [Gazebo Harmonic Ubuntu 설치](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [MuJoCo Python 설치](https://mujoco.readthedocs.io/en/stable/python.html#installation)
- [공식 CFLib 소스](https://github.com/bitcraze/crazyflie-lib-python)
