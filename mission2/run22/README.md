# Mission2 run22 — 설명·코드·영상·검증 자료

삼각측량을 쓰지 않는 run22 동결 버전입니다. 먼저 설명용 PDF를 읽고 기술원자료 PDF에서 수식·상수·실제 판단과 명령·한계를 확인하세요.

- **실제 결과:** 8/8 순서 통과, 비바닥 접촉 표본 0, DONE, 230.913초. 별도 연착지 평가는 실패했습니다.
- **실제 조건:** 입구 폭 1.5m, 출구 폭 1.8m. 다음 시험의 출구 1.5m 및 전진 관측·회전 확인 방식은 run22에 포함되지 않습니다.
- **run22 변경:** 좌우45° 스캔 yaw 명령 최대30°/s, P0.8/D0.6; 통과 여유20cm + 카메라 보정3cm.
- **알려준 물체 규격:** 링 지름과 출구 폭1.8m. 카메라·기체 상태·코스 순서/개수·초기 동작 등 운용 가정은 남아 있습니다. 지도 좌표·벽방향·블록/벽 치수는 정책에 주지 않았습니다.
- **펌웨어:** 기존 kalman.ringPass v2 수정본을 사용했습니다. 원본 펌웨어 무수정 성공으로 설명하지 마세요.

## 두 문서와 전체 자료 받기

[run22 Release: PDF 2개와 자료 ZIP](https://github.com/Carrot1ike/Drone3-Mission3/releases/tag/mission2-run22-20261027)

| 자료 | 내용 |
|---|---|
| Mission2_run22_설명용.pdf | 수행 흐름과 관측·판단·명령·결과를 읽기 쉽게 정리 |
| Mission2_run22_기술원자료.pdf | 구현 수식·상수·실제 로그·명령 대비 결과·검증 한계 |
| mission2-run22-materials-20261027.zip | 두 문서의 PDF/DOCX/Markdown, 그림, 230.95초 전체 영상, 동결 코드, 설정, 평가, 분석표, 펌웨어 |
| mission2-run22-raw-20261027.zip | 판단·명령·상태 JSONL, 수신 이미지 전체, 실제 평가 맵과 GT, 동결 소스 |

## 폴더 읽기

- source/mission2_fresh: 실행한 제어 코드 원본. source/tools는 독립 평가·시각화 도구이며 비행 정책 입력이 아닙니다.
- evidence/run22: 통과/접촉/착륙 평가, 조건, 스캔 및20cm적용 검증. evidence/comparisons는 이전버전/맵의 비교 요약입니다.
- analysis: 직진 결정→명령→실제 변위의 CSV/JSON과 추출 방법. GT는 사후 평가에만 썼습니다.
- firmware: 기존 수정 패치·식별 정보. 실행 바이너리는 materials ZIP에 포함됩니다.
- figures와 media는 materials ZIP 안에 있습니다. 20fps는 저장 영상의 재생률이고 실제 입력은 평균16.11fps였습니다.
- MANIFEST_SHA256.json으로 각 사본의 크기와 SHA256을 확인할 수 있습니다. 원본 영상·측정값은 변경하지 않았습니다.

## 다른 PC에서 실행하기

원본 SITL 저장소(위 커밋)를 먼저 설치하고 **materials ZIP 전체를 해제**하세요. Git 코드만 받았다면 Release의 materials ZIP에서 firmware/ON/cf2도 받아야 합니다. Linux 실행 바이너리를 포함합니다. 호스트 Python에는 numpy, scipy, opencv-python이 필요합니다. 원본 저장소의 SITL Python 환경과 호스트 Python을 각각 지정합니다.

```bash
python run_run22.py --baseline /absolute/Dynamic-Drone-Mission-SITL --native-python /absolute/SITL-python --host-python /absolute/host-python --work-dir /absolute/new-run22 --prepare-only
```

위 명령으로 새 작업 사본과 경로 설정을 확인합니다. 실제 실행은 **다른 새 work-dir**로 같은 명령에서 --prepare-only를 빼세요. 실행기는 패키지 원본을 보존하고 새 사본의 설치 경로만 맞춘 뒤 SITL 시작 → run22 → 런타임 종료 → 독립 평가를 실행합니다. 실행 당시의 알고리즘·폭1.8m·명령 인자는 유지합니다. 기존 SITL이 사용 중인 TCP5000/UDP20050/20150 포트를 동시에 사용하지 마세요. 설치 경로 치환과 모듈 import는 배포 PC에서 확인했으며, 다른 OS/PC에서 완주를 검증한 것은 아닙니다.

## 재현 범위

기준 환경은 unLIMited72/Dynamic-Drone-Mission-SITL 커밋 3c6e805abb3a42bfd76e0851c71d8f45b5b69851입니다. 소스의 실행 당시 절대 경로는 보존했습니다. 다른 PC에서는 원본 저장소 설치 경로와 Python 환경, runtime_config의 firmware_binary 경로를 맞춰야 합니다. 아래 실행 인자는 run22 기준입니다.

```bash
python -m mission2_fresh.main --out <새_출력폴더>/host --duration 330 --ring-mask 1 --exit-width 1.8 --wall-scan
```

이 명령 전에 원본 SITL 런타임을 같은 설정으로 시작해야 합니다. 전체 절차와 원본 경로 예시는 기술원자료 문서에 있습니다. 기존 완료된 host 경로를 덮어쓰지 말고 새 경로를 사용하세요. 다른 PC 설치/실기체 검증을 완료했다는 의미는 아닙니다.

코스 통과와 연착지, 명령 속도와 실제 속도, 과거 관측과 현재 검출, 저장fps와 입력fps를 구분해 설명해주세요.
