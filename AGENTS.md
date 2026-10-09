# Drone3-Mission3 작업 규칙

- 팀원 설치는 scripts/setup_mission3.sh --shared-root <기존 팀 저장소>를 사용한다. 별도 .venv-mission3를 만들고 팀의 소스/가상환경을 수정하지 않는다. 실행 고정 버전은 requirements/mission3-lock.txt와 config/mission3-bootstrap.json이다.
- 설치 재현은 같은 PC의 새 경로에서 검증했다. 실제 다른 PC/GPU 검증과 제조사 일치·실기 승인은 구분한다. docs/mission3/teammate-setup.md를 참고한다.
- 자체 코드는 scripts/, 외부 체크아웃은 src/, 설정은 config/, 문서는 docs/, 공유 결과는 results/에 둔다.
- 생성 로그·전체 원자료는 logs/와 artifacts/, 빌드는 build/, 가상환경은 프로젝트 내부에 둔다. 이 디렉터리들은 Git에서 제외한다.
- 현재 기준은 README.md와 config/mission3-30x5-straight-v1.json 및 mission3-rpyt-30x5-straight-v1.json이다. 과거 날짜 설정과 혼합하지 않는다.
- MissionSupervisor가 판단하고 *_control.py가 단계별 명령을 계산한다. 한 비행의 CommandLimiter 이력은 단계 전환에서도 유지한다.
- 카메라는 제공 조건 mono8 320x320와 제조사 공칭값을 따른다. 실제 미확정 사양을 제조사 일치로 주장하지 않는다. 참값은 plant와 오프라인 평가에만 사용한다.
- 원자료·frozen 소스·판정 SHA를 변경하지 않는다. 명령 포화와 실제 속도를 구분한다. 접촉/PWM0/직선3초가 모두 확인되어야 조건부 완료다.
- 결과 모집단을 구분하고 실패를 분모에서 빼지 않는다. 구조 변경 재현시험을 새 효능 표본으로 합산하지 않는다.
- 새 PC 설치 완료, 독립 검토, 정식 승인, 실기 성공을 근거 없이 표시하지 않는다.
- 독립 시험만 최대4 CPU/GPU프로세스로 병렬화한다. 한 비행의 시간 순서는 유지한다.
- 자신의 실행만 종료하며 무관한 프로세스에 broad pkill을 하지 않는다.
- 개인 인증 파일과 외부 제공 원본을 저장소에 넣지 않는다. 외부 저장소 변경·공유 대상은 현재 사용자의 승인 범위에 따른다.
- 설명·문서는 한국어로 작성한다. 사용자가 요청하지 않은 예약 실행이나 과거 세션 마감을 새 작업 지시로 적용하지 않는다.
