# 미션3 Git 공유 구성

2026-10-09 로컬 업로드 준비안. 원격 저장소 생성·push·Release 업로드는 아직 수행하지 않았다.

## 저장소에 포함하는 것

현재 실행 경로를 유지한다. 이 프로젝트의 `src/`는 외부 저장소 체크아웃이므로, 미션3 자체 코드를 여기에 옮기지 않는다.

```text
Drone3/
├── README.md                          최신 조건·빠른 시작·문서 안내
├── scripts/
│   ├── mission3/                       판단 관리자·하위 제어·추정·물리 모델·검사
│   ├── mission3_automation.py           GUI와 독립 시험 병렬 실행
│   ├── run_mission3_automation.sh       실행 진입점
│   └── build_mission3_*.py              공식 펌웨어 호스트 라이브러리 빌드
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

`.gitignore`로 외부 소스 `src/`, `.venv`, 빌드 산출물 `build/`, 로그 `logs/`, 전체 실험 `artifacts/`, 영상, ZIP, 개인 인증 파일을 제외한다. 사용자 제공 규정 원본과 스케치도 공개 배포 범위가 확정되지 않아 로컬 보관한다. 만들어 둔 보고서와 직접 작성한 도면은 포함한다. 기존 원자료·영상 파일을 삭제하는 작업은 아니다.

외부 의존성은 저장소 URL·commit·필요한 하위 소스 경로로 기록한다. `config/mission3-git-dependencies.json`은 현재 체크아웃과 패키지 버전의 관측 기록이며 설치 완료 인증이나 전체 환경 자동 복원 도구가 아니다. 기존 루트 `requirements.lock.txt`는 과거 직접 CFLib 환경이고, 최신 미션3의 MuJoCo 3.13.0 환경과 구분한다. 아직 새 PC에서 clone부터 실행까지 검증한 자동 설치 절차는 없다.

## 영상·원자료 공유

GitHub Release 첨부파일로 아래 두 파일을 올리는 방식을 제안한다. 실제 업로드 뒤 확인한 다운로드 URL을 결과 디렉터리의 `release-assets.json`에 기록한다. 현재 URL은 null이며 업로드한 것으로 표시하지 않는다.

| 파일 | 내용 | 용량 |
|---|---|---:|
| `mission3-team-share-privacy-20261010.zip` | 보고서, 전체 결과, 현재 구조/실행 당시 코드 구분, 영상 32개 | 약 49.0 MB |
| `mission3-straight-16-privacy-20261010.zip` | 최신 16회 원로그·무손실 영상·설정·실행 당시 소스 전체 | 약 490.6 MB |

GitHub는 일반 Git 파일 중 100 MiB를 초과하는 파일을 차단하며 대용량 배포에는 Release 첨부를 안내한다. 따라서 원자료 ZIP을 일반 commit에 넣지 않는다. [GitHub 공식 대용량 파일 안내](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github), [Release 안내](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases).

현재 코드에는 구조 분리 이후의 관리자와 하위 제어가 들어 있다. 16회 원실험은 분리 이전 소스 snapshot으로 실행했으므로 원 ZIP을 함께 보존한다. 구조 변경의 입력 재생·대표 물리 비교는 별도 검증이며 추가 착륙 표본으로 세지 않는다.

## 원격 대상 선택

미션3 코드·자료를 별도 저장소로 공유하는 구성을 기본안으로 둔다. 팀장이 제공한 `crazyflie-ros2-sil`은 고정 외부 의존성으로 남는다. 팀 공유 저장소에 올릴 경우에는 그 저장소의 별도 작업 브랜치에서 경로를 맞추는 작업이 필요하다. 대상 URL과 선택이 확정되기 전에는 원격을 만들거나 변경하지 않는다.

업로드 전 확인할 항목은 실제 Git 포함 파일과 크기, 제외 규칙, 최신 16회 결과 사본의 SHA, 문서 상대 링크다. 현재 로컬 프로젝트 루트는 Git 저장소로 초기화하지 않았다. 새 시험이나 제어 동작 변경은 이번 정리 범위에 포함하지 않는다.
