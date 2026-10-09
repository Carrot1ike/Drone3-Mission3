# 팀원 PC에서 미션3 실행하기

팀장이 공유한 [crazyflie-ros2-sil](https://github.com/unLIMited72/crazyflie-ros2-sil)의 설치를 완료한 **Ubuntu 22.04 / Linux x86_64 / Python 3.10** 환경을 대상으로 합니다. 미션3 코드는 별도 저장소에서 받고, 기존 팀 설치 경로를 한 번 지정합니다.

## 처음 한 번

```bash
git clone https://github.com/Carrot1ike/Drone3-Mission3.git
cd Drone3-Mission3
bash scripts/setup_mission3.sh --shared-root /실제/설치한/crazyflie-ros2-sil
```

마지막 경로를 팀원 PC에 있는 **팀장 저장소 최상위 폴더**로 바꾸세요. `.venv`, `manifests`, `crazyflie-firmware`가 들어 있는 폴더입니다. 비공개 미션3 저장소를 받으려면 저장소 접근 권한과 해당 PC의 GitHub 로그인이 필요합니다. ZIP으로 코드를 받아도 폴더에서 같은 설치 명령을 실행할 수 있습니다.

설치 스크립트는 다음을 처리합니다.

1. 팀 설치의 Python 3.10과 재귀 CMSIS 소스 버전을 확인합니다.
2. 미션3 안의 `src/crazyflie-ros2-sil`에 기존 팀 폴더를 가리키는 링크를 만듭니다. 기존 팀 소스를 수정하지 않습니다.
3. 제조사 Crazyflie 펌웨어의 고정 커밋을 미션3 `src/`에 다운로드합니다.
4. 미션3 전용 `.venv-mission3`에 [고정 의존성](../../requirements/mission3-lock.txt)을 설치합니다. 팀장의 `.venv`와 시스템 Python에는 설치하지 않습니다.
5. 공식 추정기·자세 PID·PC 속도 PID의 호스트 C 라이브러리 3종을 이 PC에서 빌드합니다.
6. 라이브러리 로딩, ArUco 인식, 320×320 렌더, mono FFV1 무손실 저장, H264 미리보기 인코딩을 검사합니다.

인터넷에서 읽어 내려받는 작업은 있지만 원격 저장소에 업로드하거나 변경하지 않습니다. 관리자 권한과 전역 셸 설정 변경도 사용하지 않습니다. 실행 로그·빌드 해시는 `build/mission3-setup.json`, 각 `build/mission3-*/source-manifest.json`에 남습니다.

## 화면에서 실행

```bash
bash scripts/run_mission3_automation.sh
```

브라우저에서 `http://127.0.0.1:8765`를 열면 시작·중지, 시험 거리 범위와 병렬 작업 수, 결과·영상 확인을 할 수 있습니다. 최대 4개의 독립 시험만 병렬로 실행하며, 한 비행 내부 제어 순서는 유지합니다. 모터·카메라·센서·물리 계산이 순서대로 진행된 후 영상과 자동 판정 자료를 생성합니다.

첫 실행에서는 거리 1.25m 한 조건, 작업 수 1개로 확인하는 것을 권합니다. GUI에서 시작하기 전까지 새 시험은 시작되지 않습니다. `Ctrl+C`로 GUI를 종료하면 그 실행기가 시작한 자식 작업만 중단합니다.

## 명령으로 한 조건 또는 전체 16조건 실행

```bash
# 거리 1.25m 한 번: 새 물리 시험 + 자동 평가 + 두 시점의 영상
bash scripts/run_mission3_automation.sh --batch \
  --start-distance 1.25 --end-distance 1.25 --workers 1 --render-video

# 1.25→0.50m / 5cm 간격 / 각 1회 / seed 1001
bash scripts/run_mission3_automation.sh --batch --workers 4 --render-video
```

새 결과는 `artifacts/mission3/automation/<실행시각>/`에 저장됩니다. `report.md`, `plan.json`, `progress.json`과 조건별 원로그·자동 검사·영상이 포함됩니다. 실패와 중단도 계획한 분모에서 보존합니다. 실행 전 소스·설정·빌드 파일을 동결하며, 진행 중 파일이 바뀌면 시험을 중단합니다.

최신 조건은 경기장30×5m / 그물5m / 동쪽28m 직선 / 차량 최대0.3m/s / 드론1m 호버 준비 뒤 출발입니다. 드론은 북쪽에서 동쪽으로20°를 바라보며, 마커 관측 전에는 제자리에서 회전 없이 기다립니다. 차량을 놓친 시험은 종료합니다. 패드20×20cm·높이30cm / 마커판20×20cm·코드15×15cm를 유지합니다.

## GPU가 없거나 렌더가 실패할 때

기본 렌더는 `egl`입니다. 지원되는 EGL 드라이버가 필요하지만 NVIDIA를 필수로 가정하지 않습니다. 이미 OSMesa가 설치되어 있다면 CPU 렌더를 명시적으로 선택할 수 있습니다.

```bash
MUJOCO_GL=osmesa bash scripts/check_mission3_environment.sh
MUJOCO_GL=osmesa bash scripts/run_mission3_automation.sh
```

설치 단계에서 기본 EGL 검사가 실패했다면 같은 설정으로 설치를 다시 실행할 수 있습니다.

```bash
MUJOCO_GL=osmesa bash scripts/setup_mission3.sh --shared-root /실제/설치한/crazyflie-ros2-sil
```

렌더 방식에 따라 픽셀과 인식 결과가 달라질 수 있어 EGL 결과와 자동으로 같은 모집단으로 합치지 않습니다. CPU 렌더는 처리 시간이 길 수 있으므로 우선 작업 수1로 확인하세요.

| 오류 | 확인할 것 |
|---|---|
| 팀 `.venv` 또는 CMSIS가 없음 | 팀 저장소의 `scripts/setup_python.sh`, `scripts/setup_sources.sh` 실행 결과 |
| CMSIS 커밋 불일치/수정 | 고정 기준의 깨끗한 별도 팀 설치를 지정. 자동 reset/덮어쓰기 없음 |
| `gcc` 없음 | 팀 환경의 시스템 설치 단계. 필요한 시스템 패키지는 팀원 PC 터미널에서 설치 |
| `EGL` / `OSMesa` 로딩 실패 | 해당 PC의 그래픽 드라이버/라이브러리. 설치기는 관리자 패키지를 자동 설치하지 않음 |
| Python 패키지 버전/빌드 소스 불일치 | 설치 스크립트를 같은 경로에서 다시 실행 |
| 기존 `.venv-mission3`가 다른 환경/이동된 환경 | 해당 폴더를 별도 이름으로 보관한 뒤 현재 경로에서 다시 설치 |
| `8765` 포트 사용 중 | `bash scripts/run_mission3_automation.sh --port 8767` 후 해당 주소 열기 |

`.venv-mission3`, `src`, `build`를 다른 PC로 복사하지 말고 코드만 공유해 설치 스크립트를 실행하세요. Python 공식 문서도 가상환경을 이동하기보다 의존성 목록에서 다시 생성하도록 안내합니다. [Python venv 문서](https://docs.python.org/3.10/library/venv.html)

## 실행 범위와 검증 기록

환경만 확인하거나 코드 회귀 검사를 실행하려면 다음 명령을 사용합니다. 새 비행 시험은 시작하지 않습니다.

```bash
bash scripts/check_mission3_environment.sh
bash scripts/run_mission3_tests.sh
```

미션3 실행기는 독립 MuJoCo 장면·센서 모델·호스트 C 라이브러리로 동작합니다. 팀 저장소의 ROS 노드/CrazySim을 실행하는 명령은 아니며, 팀의 기존324×244 통합 카메라를 미션3의 native mono320×320으로 변경하지 않습니다. ROS 실행과 기체 사양 검증은 각각 별도 범위입니다. 현재 카메라 보정·장착 변환·동역학·접촉 모델의 미확정 항목도 유지됩니다.

기존 [직선16회](../../results/mission3/straight-20261009/README.md)는 조건부 완료14/실패2입니다. 이번 설치 재현 검사와 새 팀원 실행을 그 결과에 추가하지 않습니다. 자동 `pass`는 제조사 일치·독립 승인·실기 성공을 뜻하지 않습니다.

새 설치 검증의 구체적인 범위와 결과는 [팀원 실행 재현 기록](teammate-reproduction-20261009.md)에 기록합니다. 오프스크린 렌더 사용 근거는 [MuJoCo 공식 시각화 문서](https://mujoco.readthedocs.io/en/stable/programming/visualization.html), 고정 버전은 [bootstrap 설정](../../config/mission3-bootstrap.json)을 참고하세요.
