# 미션3 Git 공유 구성

2026-10-09 사용자 요청으로 `Carrot1ike/Drone3-Mission3` 비공개 저장소를 생성하고 자체 코드·문서·최신 결과 554개 파일을 업로드했다. 대용량 ZIP의 상태와 실제 URL은 `results/mission3/straight-20261009/release-assets.json`에서 확인한다.

## 저장소에 포함하는 것

현재 실행 경로를 유지한다. 이 프로젝트의 `src/`는 외부 저장소 체크아웃이므로, 미션3 자체 코드를 여기에 옮기지 않는다.

```text
Drone3/
├── README.md                          최신 조건·빠른 시작·문서 안내
├── scripts/
│   ├── mission3/                       판단 관리자·하위 제어·추정·물리 모델·검사
│   ├── mission3_automation.py           GUI와 독립 시험 병렬 실행
│   ├── run_mission3_automation.sh       실행 진입점
│   ├── setup_mission3.sh               기존 팀 설치 연결·미션3 전용 환경 생성
│   ├── check_mission3_environment.sh   환경·렌더·영상 검사
│   └── build_mission3_*.py              공식 펌웨어 호스트 라이브러리 빌드
├── requirements/mission3-lock.txt      실행 패키지 고정 버전
├── config/                             현재 프로필·출처·버전 고정 정보
├── docs/
│   ├── mission3/                       구조·센서 근거·최신 시험 설명·공유 PDF
│   ├── diagrams/                       편집 가능한 제어 선도
│   └── git-sharing.md                  이 안내
├── assets/                             프로그램 UI·마커·공유 도면
└── results/mission3/straight-20261009/   최신 직선 시험 16회 표·판정·SHA·배포 안내
```

`README.md`가 최신 직선 조건의 시작점이다. 과거 설정·설계·작업 기록은 이력을 유지하며, 최신 결과표의 분모에 과거 실험을 합산하지 않는다. 최신 결과 디렉터리에는 30×5m 직선 시험 16회만 넣는다.

## 저장소에서 제외하는 것

`.gitignore`로 외부 소스 `src/`, `.venv`와 `.venv-mission3`, 빌드 산출물 `build/`, 로그 `logs/`, 전체 실험 `artifacts/`, 영상, ZIP, 개인 인증 파일을 제외한다. 사용자 제공 규정 원본과 스케치도 공개 배포 범위가 확정되지 않아 로컬 보관한다. 만들어 둔 보고서와 직접 작성한 도면은 포함한다. 기존 원자료·영상 파일을 삭제하는 작업은 아니다.

실행 설치는 [팀원 안내](mission3/teammate-setup.md)를 따른다. `requirements/mission3-lock.txt`는 미션3 고정 패키지 버전, `config/mission3-bootstrap.json`은 제조사/CMSIS 소스 고정 커밋이다. 설치기는 기존 팀 환경을 읽고 `.venv-mission3`와 호스트 C 라이브러리를 프로젝트 안에 생성한다. `config/mission3-git-dependencies.json`은 초기 관측 이력으로 유지하며, 과거 루트 `requirements.lock.txt`를 이번 설치에 사용하지 않는다. 같은 PC의 새 경로·새 환경에서 설치·직선1조건·영상 생성과 제공을 검증했다. [재현 범위와 근거](mission3/teammate-reproduction-20261009.md)를 참고한다.

## 영상·원자료 공유

대용량 파일은 GitHub Release 첨부파일로 분리한다. 실제 업로드가 완료된 파일에만 결과 디렉터리의 `release-assets.json`에 확인한 다운로드 URL과 상태를 기록한다.

| 파일 | 내용 | 용량 |
|---|---|---:|
| `mission3-team-share-privacy-20261010.zip` | 보고서, 전체 결과, 현재 구조/실행 당시 코드 구분, 영상 32개 | 약 49.0 MB |
| `mission3-straight-16-privacy-20261010.zip` | 최신 16회 원로그·무손실 영상·설정·실행 당시 소스 전체 | 약 490.6 MB |

GitHub는 일반 Git 파일 중 100 MiB를 초과하는 파일을 차단하며 대용량 배포에는 Release 첨부를 안내한다. 따라서 원자료 ZIP을 일반 commit에 넣지 않는다. [GitHub 공식 대용량 파일 안내](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github), [Release 안내](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases).

현재 코드에는 구조 분리 이후의 관리자와 하위 제어가 들어 있다. 16회 원실험은 분리 이전 소스 snapshot으로 실행했으므로 원 ZIP을 함께 보존한다. 구조 변경의 입력 재생·대표 물리 비교는 별도 검증이며 추가 착륙 표본으로 세지 않는다.

## 업로드 대상과 로컬 사본

공유 대상은 [Carrot1ike/Drone3-Mission3](https://github.com/Carrot1ike/Drone3-Mission3)이며 비공개다. 팀장이 제공한 crazyflie-ros2-sil 저장소를 변경하는 작업은 하지 않았다.

최초 GitHub 게시의 코드·문서·결과는 약7MB이고 `src/` 및 당시126GB 규모의 전체 실험 `artifacts/`를 제외했다. 파일별 Git blob SHA와 실행 권한을 원격 tree에서 로컬 내보내기 manifest와 대조했다. 최초 게시에는 새 비행이 없었다. 후속 팀원 실행 검증은 직선1조건을 별도 배치로 재현했으며, 기존16회 성능 분모에 추가하지 않는다.

로컬 원프로젝트는 기존 경로와 세션 기록을 유지한다. 별도 내보내기 디렉터리는 `artifacts/git-upload-20261009/`이고 최신 내용만 보이는 README와 세션 예약을 제외한 지속적인 AGENTS 규칙을 사용한다. 과거 작업 로그, 사용자 제공 원본 규정과 스케치는 게시하지 않았다. 최신 문서의 상대 링크는 게시 파일에 맞게 확인한다.

같은 PC의 별도 체크아웃·빈 의존성 경로에서 검증한 결과이며 실제 다른 PC/그래픽 드라이버에 대한 확인은 팀원 실행으로 남아 있다. 실행 근거 JSON은 `results/mission3/setup-reproduction-20261009/`에 둔다. 기존 Release ZIP은 원16회 배포 자료로 보존하므로 최신 설치 스크립트는 저장소 `main`의 코드를 받는다. 외부 소스의 commit 기록만으로 시스템 패키지와 그래픽 드라이버까지 복원한다고 주장하지 않는다.
