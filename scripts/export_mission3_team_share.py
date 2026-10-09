#!/usr/bin/env python3
"""Reviewed latest-batch data -> portable Korean team materials (no flight runs)."""
from pathlib import Path
import json
import html
import shutil
import base64

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'artifacts/mission3/team-share-20261009'
DATA = json.loads((OUT / 'reviewed-results.json').read_text())
ROWS = DATA['rows']
CAMERA_TIMES = json.loads((OUT / 'camera-time-ranges.json').read_text())
for row in ROWS:
    row['camera_start_s'] = CAMERA_TIMES[row['case_id']]['start_s']
    row['camera_end_s'] = CAMERA_TIMES[row['case_id']]['end_s']
    row['camera_max_record_gap_s'] = CAMERA_TIMES[row['case_id']]['max_dt_s']
    source = ROOT / 'artifacts/mission3/automation' / DATA['batch_id'] / row['case_id'] / 'run/camera_frames.jsonl'
    shutil.copy2(source, OUT / 'evidence' / row['case_id'] / 'camera_frames.jsonl')
(OUT / 'reviewed-results.json').write_text(json.dumps(DATA, ensure_ascii=False, indent=2) + '\n')

REFS = [
    ('Bitcraze 모노 카메라 데이터시트', 'https://github.com/bitcraze/hardware/blob/8207400d3e6090ea1dce2b22ddaa25243f718db1/src/products/ai-deck-mono-camera/datasheet/index.md', 'HM01B0 모노 모듈의 유효 320×320, 롤링 셔터, 공칭 화각·초점 정보. 실제 지급 모듈/보정과 구별.'),
    ('Bitcraze Flow deck v2', 'https://www.bitcraze.io/products/flow-deck-v2/', 'PMW3901 광류와 VL53L1x ToF, 표면·광량 의존성과 센서 융합.'),
    ('Bitcraze Commander Framework', 'https://www.bitcraze.io/2020/05/the-commander-framework/', '행동 명령 생성과 기체 setpoint 처리 분리. 우리 상위 관리자는 프로젝트 자체 구현.'),
    ('Bitcraze Controllers', 'https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/functional-areas/sensor-to-control/controllers.md', '위치·속도·자세·각속도 연쇄 PID. 실제 사용 소스는 배치에서 동결한 버전.'),
    ('SMACC2 Hierarchical States', 'https://smacc2.robosoft.ai/tutorials/tutorial-9-hierarchical-states/', '상위 맥락과 하위 상태를 공유하는 구조의 참고. SMACC2 라이브러리를 채택한 것은 아님.'),
    ('Behavior Trees survey · 2020', 'https://arxiv.org/abs/2005.05842', '행동 트리의 모듈화·재사용 대안. 현재 순차 미션은 FSM 스타일.'),
    ('Behavior Trees / FSM comparison · 2024', 'https://arxiv.org/abs/2405.16137', '구조 비교 참고. 해당 로봇 실험의 성능을 드론 착륙 성능으로 이식하지 않음.'),
    ('팀 공유 시뮬레이션 저장소', 'https://github.com/unLIMited72/crazyflie-ros2-sil', '프로젝트 개발 기반. 이번 결과는 별도 프로젝트 물리 실행 경로의 결과.'),
]

SECTIONS = [
    ('overview', '개발 목표와 현재 결과', '''미션3은 **Crazyflie 2.1 + AI-deck + Flow deck**으로 차량의 뒤쪽 마커를 인식하고, 차량 뒤에 일직선으로 정렬한 뒤 추종·접근·착륙하는 과제다. 영상 인식과 상위 제어는 PC에서 수행하고, 기체의 자세·각속도 제어 계층을 유지한다. 하드웨어 개조는 허용하지 않는다.

이번 공유 범위는 **30×5m 경기장의 최신 직선 트랙 16조건 전체**다. 옆거리 1.25→0.50m를 5cm씩 줄였고, 각 조건을 동일 seed 1001로 한 번씩 실행했다. 마커 ID 인식은 16/16, 조건부 접촉·PWM 0·3초 유지 완료는 14/16, 실패는 2/16이다. 이것은 개발 조건 비교이며 모집단 성공률이나 실기 검증 결과가 아니다.

현재 코드는 **상위 판단 관리자 → 단계별 하위 제어 → 공통 명령 제한 → 속도·자세·각속도 PID** 구조다. 16회 시험 당시 코드와 이후 구조 분리 코드를 둘 다 보존했다. 구조 변경의 재현 회귀 2회는 16회 성능 분모에 추가하지 않는다. 제조사 일치·실기·독립 승인 성공 수는 0이다.'''),
    ('arena', '시험 환경과 초기 배치', '''경기장 바닥은 **동서 30m × 남북 5m**, 그물 높이는 5m이며 실내 무풍이다. 남서쪽을 (0,0), 동쪽을 +x, 북쪽을 +y로 둔다. 차량의 패드 중심은 **(1,2.5)→(29,2.5)m**의 28m 직선을 달리고 끝에서 정지한다. 순환 트랙과 회전 구간은 이번 시험에 없다.

드론은 **(2, 2.5−d, 1)m**에서 준비한다. 출발 차량보다 동쪽으로 1m 앞이며 주행선에서 남쪽으로 d만큼 떨어져 있다. 시선은 **북쪽에서 동쪽으로 20°**, 즉 동쪽을 0°로 두는 yaw 좌표에서는 70°다. 마커 발견 전에는 제자리·고정 시선을 유지하며 회전 탐색이나 트랙 대기점 이동을 하지 않는다.

차량은 센서 기반 1m 호버 준비가 완료될 때까지 정지한다. 이후 최고 **0.30m/s**로 출발하고 종점에서 정지한다. 가속·감속 0.15m/s²는 미확정 차량의 **시험 가정 S**다. 속도·위치는 연속인 사다리꼴 속도 프로파일이며 가속도/jerk까지 연속인 프로파일은 아니다.

마커가 한 번도 인식되지 않은 채 판 전체가 화각을 통과하면 시험을 종료한다. 이 통과 판정에는 **시험 장치 전용 기하 참값**을 사용하며, 비행 제어·상태 추정 입력으로 전달하지 않는다. 이번 16회는 모두 인식했으므로 두 실패는 발견 이후에 발생했다.'''),
    ('geometry', '패드·마커와 카메라 조건', '''착륙 패드는 **20×20cm**, 지면에서 **30cm** 높이다. 뒤쪽 패드를 향한 수직 마커 판은 **20×20cm**, 실제 ArUco 코드는 **15×15cm**다. 코드 주위 여백은 한쪽 2.5cm다. 패드 앞 가장자리에서 마커 면까지 수평 간격은 15cm다.

마커 판 아래쪽은 패드 높이와 같아 z=0.30m, 위쪽은 z=0.50m, 중심은 z=0.40m다. 패드 중심에서 마커 중심까지 차량 좌표의 오프셋은 [앞 0.25m, 옆 0m, 위 0.10m]다. 이 정적 치수로 마커 관측을 착륙점의 상대 위치로 변환한다.

카메라 입력은 사용자 확정 기체 조건인 **8-bit mono 320×320 원해상도**다. 제조사 모노 카메라 자료의 공칭 화각은 수평·수직 87°, 대각 115°다. 전방 기본 장착을 유지한다. 화각 안에 있다는 것만으로 인식이 보장되지는 않는다. 거리·픽셀 크기·시야각·가림·초점과 추정 품질 검사를 모두 거친다.

실제 지급 카메라 revision, 내부 파라미터 K, 왜곡 D, 기체-카메라 변환, 노출/롤링 셔터 시각과 지연은 미확정이다. 공식 예제의 324×244/324×324 창과 실제 320×320 펌웨어 창 설정도 확인이 필요하다. 모노 320 입력 조건을 임의 리사이즈로 맞추지 않는다.'''),
    ('architecture', '상위 판단과 하위 제어 구조', '''**MissionSupervisor**는 관측의 유효 시간·불확실성·경계·안전 조건을 먼저 검사하고 실행할 단계 제어기를 선택한다. 하위 모듈은 해당 단계의 속도/yaw 명령과 완료 조건을 계산한다. 전환은 상위 관리자의 `enter/held/output`을 통해 기록하며 한 step의 최종 명령은 하나다.

하위 모듈은 **대기·발견 / 정렬 / 추종·접근 / 하강 / 복구**로 나눴다. 공통 CommandLimiter의 속도·yaw 이력과 PID·추정기 상태는 단계가 바뀌어도 유지한다. 이것은 하위 AI 에이전트나 별도 비행 프로세스를 여러 개 실행하는 구조가 아니라 **계층형 FSM 스타일**의 소프트웨어 분리다.

`run_mission3_physical.py`는 센서 전달·주기·물리 적분·기록을 연결한다. 마지막 접촉 지원 시험, 모터 종료와 3초 유지 연결부는 아직 이 실행기에 있다. 상위 판단 구조 분리는 완료했지만 실행기의 모든 로직을 분리한 상태는 아니다.

GUI의 최대 4개 병렬 프로세스는 **서로 독립인 시험**에 적용한다. 한 비행 안의 센서 → 추정 → 제어 → 물리 step은 인과 순서대로 실행한다.'''),
    ('control', '단계와 피드포워드 + 피드백', '''기본 순서는 **호버 준비 → 고정 대기 → 인식·관측 안정화 → 일직선 정렬 → 높이 준비 → 추종·접근 → 마지막 상대운동 예측 → 하강 → 접촉 확인 → 모터 종료·3초 유지**다. 품질이 떨어지면 제한된 복구를 수행하거나 실패로 종료한다.

수평 제어의 개념식은 **v_cmd = v̂_car + Kp·e_relative + Kv·ė_relative**다. v̂_car는 영상·기체 센서로 추정한 차량 속도의 피드포워드이고, 오차 항은 추종 위치·상대 속도 피드백이다. 실제 구현의 단계별 축·한계·품질 gate는 `guidance.py`와 각 제어 모듈에 있다. 설정 속도 0.30m/s나 차량 실제 위치를 제어기에 넣지 않는다.

상위 명령 계산은 50Hz, PC 속도 PID/RPYT 전달은 100Hz이며 자세·각속도는 고정한 공식 펌웨어 PID 경로를 사용한다. 여기서 소프트웨어 실행 주기를 실제 무선 통신 지연이나 실기 제어 성능으로 해석하지 않는다.

정상 명령 제한은 속력 **0.40m/s**, 가속 **0.15m/s²**, yaw 속도 **10°/s**, yaw 가속 **20°/s²**, 상승 **0.10m/s**, 하강 **0.06m/s**다. 말기 지원 하강은 0.025m/s다. 안전 중단·모터 종료의 명시적 override는 정상 가속 제한과 구분한다. 명령 포화 통과는 실제 속도의 엄격한 상한 보장이 아니다.'''),
    ('results', '최신 직선 시험 16회 전체 결과', '''**완료 판정**은 실제 움직이는 패드 접촉, 모터 PWM 0, 직선 구간 지지 3초를 모두 요구한다. 근접 정지나 GEOMETRY_READY는 완료가 아니다. 모든 조건에서 발견 전 차량 정지와 호버 준비 시각의 출발을 확인했다.

첫접촉 오차는 조건부 기체 CoM/body 원점과 패드 중심 사이 거리다. 내부 점수는 중심에서 반경 1cm 미만 10점, 다음 1cm 고리마다 1점 감소, 10cm 이상 0점이다. 대회 공식 점수가 아니며, 사각 패드의 접촉 여부 판정과 별개다. 접촉이 없는 값은 ‘—’로 표시한다. 실패 조건에 첫접촉 점수가 있더라도 완료로 세지 않는다.

16회 정상 명령 포화 검사를 통과했다. 기록된 native 11,084프레임은 FFV1 재생 픽셀 SHA와 일치한다. 각 실행의 실제 최고 속력은 **0.35623~0.42569m/s**였다. 0.40m/s 명령 상한보다 큰 물리 응답이 있으므로 실제 속도 보호는 후속 과제다.

표의 각 거리 영상과 평가 JSON은 별도 **videos.html**에서 바로 열 수 있다. 제3자 1배속 영상 16개와 기체 시점 영상 16개를 포함했다. 기체 영상은 기록 프레임을 고정 20FPS로 재생하는 미리보기이며 호버 준비·말기 공백을 포함한 전체 시험 시간과 같지 않다. 1.15m에는 약 7.2초의 기록 공백이 있어 시간 분석은 camera_frames.jsonl의 t를 사용한다. H264는 공유용 미리보기이고, 입력 픽셀 보존의 근거는 별도 원자료 압축본의 FFV1·SHA 기록이다.'''),
    ('failures', '실패와 센서 전환에서 남은 문제', '''**1.15m: 관성 예측 시간 만료.** 첫접촉 오차 2.795cm와 내부 8점 기록은 있지만 `inertial_texture_bridge_prediction_expired`로 모터 종료·3초 유지까지 완료하지 못했다. 실제 접촉이 있었다는 사실과 미션 완료를 구별한다.

**0.50m: 발견 이후 영상 소실.** 마커 ID 인식은 있었지만 `visual_loss_after_discovery`로 중단했고 접촉 점수는 없다. 가까워질수록 항상 유리하다는 결론은 나오지 않았다. 현재 기록만으로 유실의 유일 원인을 화각·검출기·상태 추정 중 하나로 확정하지 않는다.

Flow deck의 광류는 바라보는 표면에 대한 상대 운동이다. 고정 바닥에서 움직이는 패드로 표면이 바뀌면 같은 속도 의미를 그대로 사용할 수 없다. ToF도 기체의 절대 높이가 아니라 **아래 표면까지의 거리**다. 기체가 z=1m를 유지한 채 z=0.30m 패드 위로 가면 거리만 약 1.0→0.7m로 변할 수 있다.

현재 구현은 바닥·패드·혼합/무효 표면을 구분하고 시간·품질 gate를 거쳐 센서를 사용한다. 근접에서는 광류/ToF 유효성을 제한하고 짧은 IMU 예측과 지원 관측을 사용한다. 센서의 최소 거리·시야와 접촉 모델은 미확정이므로 실기 착륙의 근거는 아니다. 마지막 접근은 기존 단계별 기준을 유지하며 완전한 연속 궤적 제어가 검증된 상태는 아니다.'''),
    ('validation', '코드 분리 검증과 성능 해석', '''최신 16회는 구조 분리 전의 동결 코드로 실행했다. 이후 현재의 상위/하위 구조로 바꾸면서 **16회 저장 입력의 28,480개 명령·전환 이벤트가 정확히 일치**함을 확인했다. 기하 완료 이후 접촉 지원 연결부는 이 재생 비교 범위에 포함하지 않는다.

대표 성공 1.25m와 실패 1.15m를 동일 설정/seed로 물리 재실행한 두 회귀는 원 JSONL·설정·FFV1 영상이 바이트 단위로 일치했다. 결과 요약은 벽시계 실행 시간을 제외하고 같다. 미션 회귀 235개와 자동화 프로그램 회귀 5개가 통과했다. 두 회귀는 구조 보존 검사이므로 **시험 결과 분모는 계속 16**이다.

비행 입력은 영상과 기체 센서 추정이다. 참값은 물리 plant, 시험 장치 종료와 사후 평가에만 사용한다. 이번 결과는 미확정 동역학·보정·접촉 가정 아래의 조건부 결과다. 새 seed, 반복 시험, 다른 진입 배치와 실기 검증은 이번 범위에 없다. 작성자의 사후 검증을 독립 검토 서명으로 표시하지 않는다.'''),
    ('hardware', '제조사 자료와 시뮬레이션 가정의 구분', '''근거 표기는 **P 제조사 공개값 / C 사용자 확정 조건 / D 공개값·치수에서 계산 / S 시험·민감도 가정 / U 미확정**으로 구분한다. 고정한 소프트웨어 버전은 실행 식별 정보이며 실기 물성의 근거가 아니다.

**AI-deck:** 전방 카메라 영상 취득·전송 계층이다. 이번 판단과 제어 계산은 PC에서 수행한다. 모노 HM01B0 모듈 자료는 유효 320×320, 전자 롤링 셔터, 수평/수직 공칭 87°, 초점 35cm 및 공개 초점 범위를 제공한다. 이것이 실제 지급 덱의 모듈 revision·캘리브레이션·무선 지연을 확정하지는 않는다.

**Flow deck v2:** PMW3901 광류 센서와 VL53L1x ToF를 이용하며 제조사 펌웨어 상태 추정기에서 융합한다. 광류는 표면 무늬·광량에 의존하고 ToF의 공개 최대 거리 약 4m는 모든 표면·근접 조건의 정확도 보장이 아니다. 실제 사용 덱 revision도 장비 지급 후 확인해야 한다.

**시뮬레이션:** 원해상도 영상 → 실제 ArUco 검출, 광류/거리/IMU 샘플 → 센서 상태 추정, PC RPYT → 공식 PID/모터 경로 → MuJoCo 자유 기체·접촉을 연결했다. 렌더 영상·센서 오차·통신 지연·조립 질량/관성·추력·모터/공력·장착 좌표·근접 접촉/마찰은 S/U가 남는다. 카메라 FFV1 검증은 기록 픽셀의 보존 검사이며 현실 센서 일치 검사와 다르다.'''),
    ('usage', '팀원이 확인하고 실행하는 방법', '''압축을 풀고 **README.md → 미션3_팀공유.pdf → team-report.html → videos.html** 순서로 보면 된다. PDF는 회의용, HTML은 검색·원평가 확인용, videos.html은 16조건 영상 비교용이다. 동봉한 CSV/JSON은 전체 숫자와 null을 보존한다. camera-time-ranges.json은 기체 기록의 시작·종료·최대 공백을 보존한다.

실행은 기존 Drone3 프로젝트 루트에서 `bash scripts/run_mission3_automation.sh`를 호출하고 **http://127.0.0.1:8765**를 연다. CLI는 `bash scripts/run_mission3_automation.sh --batch --workers 4 --render-video`다. GUI는 시작·중지, 최대 4개 독립 시험, 결과표·영상·보고서를 제공한다. 온라인 자동 학습이나 실기 자동 비행 연결이 구현된 프로그램은 아니다.

팀 공유 저장소를 맞춘 Linux/ROS 2 Humble 환경과 프로젝트 `.venv`, MuJoCo/EGL, 호스트 공식 제어 라이브러리가 필요하다. 이번 착륙 시험은 프로젝트의 별도 물리 실행 경로이며 ROS 토픽으로 실기 비행한 결과가 아니다. 공유본에는 코드와 설정을 담았지만 전체 시스템 설치·가상환경·드라이버·서드파티 저장소를 복제한 머신 이미지가 아니다. 현재 코드는 `code-current-hierarchy/`, 16회 실행 당시 동결 코드는 `code-executed-batch/`로 구별했다.

다음 개발 우선순위는 두 실패의 영상/센서/상대 속도 오차 분해, 실제 속도 overshoot 보호, 말기 접근의 연속성과 착지 정확도 동시 검증이다. 이후 새 seed 반복과 별도 holdout을 사전 계획하고 장비 지급 후 카메라·센서·조립 동역학·통신 지연을 보정한다. 이번 공유 작업은 새 비행 시험을 추가하지 않았다.'''),
]

MODULES = [
    ('상위 판단', 'supervisor.py', '공통 관측·전역 안전 검사, 단계 선택·전환, 공유 상태'),
    ('대기·발견', 'acquisition_control.py', '고정 대기, 발견 후 영상 방위 제어'),
    ('정렬', 'alignment_control.py', '관측 안정화, 차량 뒤 일직선 정렬'),
    ('추종·접근', 'approach_control.py', '높이 준비, 상대 위치·속도 동기화, 거리 축소'),
    ('하강', 'landing_control.py', '영상 소실 구간 예측·하강 완료 조건'),
    ('복구', 'recovery_control.py', '제한된 후퇴/상승과 중단 조건'),
    ('공통 명령', 'command.py / command_limiter.py', '동일 Command 형식과 한 비행의 포화·slew 이력'),
    ('공통 수식', 'guidance.py', '추정 입력과 차량 속도 FF + 상대 오차 FB'),
    ('영상·차량 추정', 'vision.py', 'ArUco 검출·포즈 품질·차량 운동 추정'),
    ('기체 추정', 'physical_estimator.py', 'IMU·광류·ToF, 표면 변화와 유효성 처리'),
    ('하위 PID', 'position_bridge.py / flight_controller.py', 'PC 속도 PID → RPYT → 공식 자세·각속도 PID/모터'),
    ('실행 연결', 'run_mission3_physical.py', '주기·센서·물리·기록, 접촉 지원·모터 종료·유지 연결'),
    ('자동 시험', 'mission3_automation.py', 'GUI와 독립 시험 프로세스 관리'),
]

PHASES = [
    ('HANDOFF_PREROLL / WAIT', '센서 기반 1m 안정 호버 준비 후 차량 출발. 마커 발견 전 제자리 고정 시선.'),
    ('COARSE_ACQUIRE / ACQUIRE', 'ID/코너 기반 방위 접근, 유효 포즈·운동 추정 안정화.'),
    ('LINEUP', '차량 진행축 뒤로 일직선 정렬. 위치·yaw·불확실성 조건 확인.'),
    ('LOWER_ALIGN', '마커 가시성과 착륙 경로를 고려한 높이 준비.'),
    ('APPROACH / TRACK', '차량 속도 FF + 상대 오차 FB로 추종하며 간격을 축소.'),
    ('BLIND_APPROACH / BLIND_DESCEND', '마지막 유효 관측·센서 기반 단기 예측. 시간·불확실성 제한.'),
    ('GEOMETRY_READY', '근접 기하 완료 신호. 아직 접촉·착륙 성공 아님.'),
    ('TOUCHDOWN_DESCEND / SUPPORT_PROBE', '지원 접촉 후보 확인과 제한된 말기 하강.'),
    ('MOTOR_OFF_HOLD', 'PWM 0 후 움직이는 패드 위 3초 지지. 사후 참값 평가 대조.'),
    ('RECOVER / ABORT', '관측 소실·예측 만료·경계·안전 gate 실패 시 복구 또는 실패 전달.'),
]

def svg_assets():
    assets = OUT / 'assets'; assets.mkdir(exist_ok=True)
    head='<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="520" viewBox="0 0 1100 520"><style>text{font-family:Arial,\'NanumGothic\',sans-serif;fill:#183348} .s{font-size:17px}.b{font-size:21px;font-weight:bold}</style>'
    arena=head+'''<rect width="1100" height="520" fill="#f6f9fb"/><text x="42" y="40" class="b">30 × 5m 직선 경기장 · 위에서 본 배치</text><rect x="45" y="80" width="1000" height="167" rx="6" fill="white" stroke="#375165" stroke-width="2"/><line x1="78" y1="164" x2="1012" y2="164" stroke="#168eaa" stroke-width="4"/><polygon points="1012,164 992,154 992,174" fill="#168eaa"/><text x="445" y="145" class="b">차량 직선 주행 28m → 동쪽</text><circle cx="78" cy="164" r="6" fill="#e58a2b"/><text x="62" y="115" class="s">출발 (1, 2.5)</text><text x="867" y="220" class="s">종점 (29, 2.5)</text><circle cx="112" cy="205.5" r="6" fill="#348569"/><text x="143" y="228" class="s">드론 (2, 2.5−d), z=1m</text><text x="46" y="284" class="s">같은 초기 x=2m에서 옆거리 d만 변경: 1.25 → 0.50m, 0.05m 간격</text><text x="46" y="320" class="b">초기 배치 확대 · 시선 각도</text><line x1="80" y1="363" x2="430" y2="363" stroke="#168eaa" stroke-width="3"/><rect x="98" y="353" width="22" height="20" fill="#e58a2b"/><text x="86" y="398" class="s">차량</text><circle cx="209" cy="460" r="7" fill="#348569"/><line x1="209" y1="460" x2="239" y2="377" stroke="#348569" stroke-width="4"/><polygon points="242,369 229,383 243,389" fill="#348569"/><line x1="209" y1="450" x2="209" y2="369" stroke="#698298" stroke-dasharray="5 4"/><text x="220" y="434" class="s">북→동 20°</text><text x="270" y="478" class="s">동 기준 yaw 70°</text><line x1="177" y1="363" x2="177" y2="460" stroke="#724faf"/><text x="112" y="442" class="s">d</text><text x="468" y="377" class="s">차량 출발점보다 드론이 동쪽 1m 앞</text><text x="468" y="414" class="s">카메라 공칭 수평 화각 87° · 화각 ≠ 인식 보장</text><text x="468" y="451" class="s">마커 발견 전 회전·주행선 접근 없이 대기</text><text x="468" y="488" class="s">아래 확대도는 설명용, 위 경기장 도면은 동일 축척</text></svg>'''
    (assets/'arena.svg').write_text(arena)
    board=head+'''<rect width="1100" height="520" fill="#f6f9fb"/><text x="42" y="40" class="b">패드·마커 측면 / 정면 · 단위 cm</text><line x1="50" y1="445" x2="630" y2="445" stroke="#375165" stroke-width="2"/><rect x="120" y="265" width="120" height="12" fill="#168eaa"/><line x1="240" y1="265" x2="330" y2="265" stroke="#c2ced7" stroke-dasharray="5 5"/><line x1="330" y1="265" x2="330" y2="145" stroke="#e58a2b" stroke-width="10"/><text x="128" y="248" class="b">패드 길이 20</text><text x="256" y="296" class="s">간격 15</text><text x="353" y="211" class="s">판 높이 20</text><line x1="94" y1="265" x2="94" y2="445" stroke="#375165"/><text x="52" y="362" class="s">30</text><text x="78" y="480" class="s">지면</text><text x="123" y="318" class="s">패드 중심 → 마커 중심:</text><text x="123" y="349" class="b">앞 25 / 위 10</text><text x="123" y="394" class="s">← 마커는 뒤쪽 패드를 향함</text><rect x="730" y="118" width="252" height="252" fill="white" stroke="#e58a2b" stroke-width="3"/><rect x="761.5" y="149.5" width="189" height="189" fill="#183348"/><text x="784" y="244" font-size="24" fill="white" style="fill:white">ArUco 15×15</text><text x="740" y="98" class="b">판 20×20</text><text x="735" y="408" class="s">사방 여백 2.5</text><text x="678" y="454" class="s">마커 면: 바닥 z=0.30m</text><text x="678" y="486" class="s">중심 z=0.40m / 위 z=0.50m</text></svg>'''
    (assets/'pad-marker.svg').write_text(board)
    hierarchy=head+'''<rect width="1100" height="520" fill="#f6f9fb"/><text x="42" y="40" class="b">미션3 · 판단과 제어 계층</text><defs><marker id="a" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto"><path d="M0,0 L8,3 L0,6" fill="#587487"/></marker></defs>'''
    boxes=[(45,80,290,70,'영상 · Flow · ToF · IMU'),(45,188,290,70,'인식 · 차량/기체 상태 추정'),(410,80,635,70,'상위 판단 관리자 · MissionSupervisor'),(410,188,635,85,'대기/발견 | 정렬 | 추종·접근 | 하강 | 복구'),(410,320,290,70,'공통 명령 제한 · 한 비행 이력'),(755,320,290,70,'속도 → 자세 → 각속도 PID'),(755,429,290,60,'모터 · 자유 기체 · 접촉')]
    for x,y,w,h,label in boxes:
        hierarchy+=f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="white" stroke="#8ca7b8" stroke-width="2"/><text x="{x+w/2}" y="{y+h/2+6}" text-anchor="middle" class="s">{label}</text>'
    for x1,y1,x2,y2 in [(190,150,190,188),(335,223,382,223),(382,223,382,115),(382,115,410,115),(728,150,728,188),(555,273,555,320),(700,355,755,355),(900,390,900,429)]:
        hierarchy+=f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#587487" stroke-width="2" marker-end="url(#a)"/>'
    hierarchy+='<path d="M755,459 H22 V115 H45" fill="none" stroke="#168eaa" stroke-width="2" marker-end="url(#a)"/><text x="65" y="454" class="s">센서 피드백</text><text x="65" y="491" class="s">참값 → plant / 시험 종료 / 사후 평가만 · 제어/추정 입력 금지</text></svg>'
    (assets/'control-hierarchy.svg').write_text(hierarchy)
    app_assets=OUT/'report-app/src/content/assets';app_assets.mkdir(exist_ok=True)
    for p in assets.glob('*.svg'):shutil.copy2(p,app_assets/p.name)

def markdown():
    text='# Drone3 미션3 · 팀 공유 자료\n\n최신 직선 시험 16조건 / 시험 종료 '+DATA['ended_kst']+'\n\n'
    for ident,title,body in SECTIONS:
        text+='## '+title+'\n\n'+body+'\n\n'
        if ident=='arena':text+='![경기장 배치](assets/arena.svg)\n\n'
        if ident=='geometry':text+='![패드·마커](assets/pad-marker.svg)\n\n'
        if ident=='architecture':
            text+='![제어 구조](assets/control-hierarchy.svg)\n\n|역할|파일|책임|\n|---|---|---|\n'
            text+=''.join('|'+ '|'.join(x)+'|\n' for x in MODULES)+'\n'
        if ident=='control':
            text+='|단계|동작·조건|\n|---|---|\n'+''.join('|'+ '|'.join(x)+'|\n' for x in PHASES)+'\n'
        if ident=='results':
            text+='|옆거리 m|ID 인식|완료|첫접촉 오차 cm|내부 점수|실패|영상|\n|---:|---|---|---:|---:|---|---|\n'
            for r in ROWS:
                err='—' if r['contact_error_cm'] is None else f"{r['contact_error_cm']:.3f}"
                text+=f"|{r['distance_m']:.2f}|있음|{'완료' if r['contact_complete'] else '실패'}|{err}|{r['points'] if r['points'] is not None else '—'}|{r['failure'] or '—'}|[제3자]({r['video']}) / [기체]({r['camera']})|\n"
            text+='\n'
    text+='## 출처와 적용 범위\n\n'+''.join(f'- [{title}]({url}): {note}\n' for title,url,note in REFS)
    (OUT/'미션3_팀공유.md').write_text(text)
    readme='''# Drone3 미션3 팀 공유 패키지

이 공유본은 최신 30×5m / 차량 최고 0.30m/s / 28m 직선 시험 16조건만 정리합니다. 기존 스타디움 결과는 포함하지 않습니다.

1. `미션3_팀공유.pdf`: 회의용 보고서. 환경·구조·단계·센서·전체 결과·한계·실행법.
2. `team-report.html`: 오프라인 보고서. 브라우저로 열어 표 검색·원자료 설명 확인.
3. `videos.html`: 16조건 선택, 제3자 1배속/기체 시점 전환. 전체 영상 32개.
4. `미션3_팀공유.md`, `all-16-results.csv`, `reviewed-results.json`: 편집·숫자 재사용.
5. `evidence/`: 원평가·실행 요약·포화·픽셀 검증·동결 계획. 원파일을 수정하지 않은 복사본.
6. `code-current-hierarchy/`: 현재 상위 관리자/하위 단계 제어 구조.
7. `code-executed-batch/`: 16회 시험 당시 동결 코드·runtime. 현재 코드와 구별.
8. `config/`, `assets/`, `verification/`: 최신 설정, 도면과 구조 분리 재현 검증.
9. `package-manifest.json`: 공유본 파일 SHA256. `raw-archive-manifest.json`: 별도 원자료 ZIP 식별.

조건부 완료 14/16, 실패 2/16, ID 인식 16/16. 동일 seed 1001로 거리별 1회이므로 모집단 성공률을 주장하지 않습니다. 제조사 일치·실기·독립 승인 성공 0. 회귀 물리 2회는 분모에 추가하지 않습니다.

## 영상·보고서 열기

ZIP 전체를 풀고 `team-report.html` 또는 `videos.html`을 엽니다. 영상을 포함한 폴더 구조를 유지하세요. 제3자 영상은 1배속이고 기체 영상은 기록 프레임의 고정 20FPS 미리보기입니다. 전체 실행 시간과 같지 않으며 1.15m에는 약 7.2초 기록 공백이 있습니다. 시간 정렬은 camera-time-ranges.json과 evidence/각조건/camera_frames.jsonl의 t를 사용합니다. 영상이 file:// 환경에서 열리지 않는 브라우저에서는 공유본 폴더에서 `python3 -m http.server 8080 --bind 127.0.0.1` 실행 후 `http://127.0.0.1:8080/videos.html`로 엽니다. 이 서버는 파일 미리보기용이며 비행 실행기가 아닙니다.

## 코드 사용

코드 폴더는 검토·재현용 snapshot입니다. 완전한 실행 환경·ROS 설치·드라이버·.venv·전체 third-party 저장소를 담은 머신 이미지가 아닙니다. 기존 팀 저장소 기반 Drone3 환경에서 코드·설정 의존성을 맞추고 프로젝트 루트의 `bash scripts/run_mission3_automation.sh`로 실행합니다. GUI는 localhost:8765입니다. CLI는 `bash scripts/run_mission3_automation.sh --batch --workers 4 --render-video`입니다. 원실험을 덮어쓰지 말고 새 배치로 기록합니다.

현재 코드의 구조 분리 검증: 저장된 16조건의 28,480 명령/이벤트 일치, 대표 성공/실패 2회 raw byte 일치, 미션 회귀 235 + 프로그램 회귀 5 통과. 재생 비교는 기하 완료 이전 범위이며 접촉 지원 연결 로직은 물리 실행기에 남아 있습니다.

## 원자료 전체

별도 `mission3-straight-16-raw-20261009.zip`에는 최신 배치의 전체 JSONL, native FFV1 AVI, 영상, 설정, 소스 snapshot과 평가를 원경로로 보존했습니다. 공유용 H264만으로 원픽셀 동일성을 검증하지 않습니다. ZIP 내 SHA manifest로 추적합니다.

대회 공식 점수/실기 인정 기준이 아니라 개발 내부 평가이며 장비의 보정·조립 동역학·장착·통신·근접 접촉 등 S/U가 남습니다. 외부 업로드나 팀원 전송은 수행하지 않았습니다.
'''
    (OUT/'README.md').write_text(readme)
    (OUT/'report-content.json').write_text(json.dumps({'sections':[dict(id=i,title=t,body=b) for i,t,b in SECTIONS],'references':REFS,'modules':MODULES,'phases':PHASES},ensure_ascii=False,indent=2)+'\n')

def app():
    app=OUT/'report-app';data=json.loads((app/'src/data.json').read_text())
    data['report']={'asOf':'2026-10-09'}
    data['queries']['mission3_trials']['rows'] = ROWS
    for key,rows,definition in [
        ('mission3_scenario',[dict(arena_m=[30,5],net_height_m=5,vehicle_start=[1,2.5],vehicle_end=[29,2.5],drone_x=2,drone_z=1,drone_y_formula='2.5 - lateral_distance',drone_yaw_deg=70,vehicle_speed_m_s=.3,vehicle_accel_assumption=.15)],'사용자 확정 최신 배치, plan 및 config를 대조한 정적 조건.'),
        ('mission3_geometry',[dict(pad_side_m=.2,pad_height_m=.3,board_side_m=.2,code_side_m=.15,pad_front_gap_m=.15,marker_from_pad_m=[.25,0,.1],mono_width=320,mono_height=320,nominal_hv_fov_deg=87)],'사용자 확정 치수 C와 제조사 카메라 공칭 값 P. 실제 캘리브레이션 U.'),
        ('mission3_structure',[dict(role=a,file=b,responsibility=c) for a,b,c in MODULES],'현재 code-current-hierarchy 소스 역할 분리. 실행 당시 code-executed-batch와 별개.'),
        ('mission3_phases',[dict(phase=a,description=b) for a,b in PHASES],'현재 supervisor/phase 모듈 단계와 물리 실행기의 접촉 연결 범위.'),
        ('mission3_verification',[json.loads((OUT/'verification/validation-summary.json').read_text())],'구조 변경 전후 동작 보존 검사. 성능 표본과 독립 검토가 아님.'),
        ('mission3_sources',[dict(title=t,url=u,scope=n) for t,u,n in REFS],'읽고 검토한 공식 문서·논문. 제조사 사실과 프로젝트 적용 판단을 구별.')]:
        data['queries'][key]={'rows':rows,'source':{'description':definition,'tables':[{'name':key}], 'metricDefinitions':[{'label':key,'definition':definition,'componentIds':[]}],'evidenceFlow':[{'title':'근거 범위','detail':definition}]}}
    data['queries']['mission3_trials']['source']['tables']=[{'name':'offline-evaluation.json × 16'},{'name':'adapter-summary.json × 16'},{'name':'command-limits-audit.json × 16'},{'name':'camera-serialization-validation.json × 16'}]
    data['buildStatus']='creating'
    (app/'src/data.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    shutil.copy2(OUT/'report-content.json',app/'src/content/report/report-content.json')
    jsx='''import React from "react";
import { RichNarrative, ReportSection, DataComponent, DataTable, useDataApp } from "../../data-app-public.jsx";
import content from "./report-content.json";
import arena from "../assets/arena.svg";
import marker from "../assets/pad-marker.svg";
import control from "../assets/control-hierarchy.svg";
import "./report.css";
const queryFor={overview:"mission3_trials",arena:"mission3_scenario",geometry:"mission3_geometry",architecture:"mission3_structure",control:"mission3_phases",results:"mission3_trials",failures:"mission3_trials",validation:"mission3_verification",hardware:"mission3_sources",usage:"mission3_structure"};
const images={arena,geometry:marker,architecture:control};
const fmt=v=>v==null?"—":Number(v).toFixed(3);
const previews=Object.fromEntries(content.references.map(([title,url,summary])=>[url,{title,summary,approvedForReport:true}]));
export function ReportContent(){
 const {snapshot}=useDataApp(); const rows=snapshot.queries.mission3_trials.rows;
 const columns=[{key:"distance_m",label:"옆거리 m",renderCell:v=>Number(v).toFixed(2)},{key:"decoded",label:"ID 인식",renderCell:v=>v?"있음":"없음"},{key:"contact_complete",label:"완료 판정",renderCell:v=>v?"조건부 완료":"실패"},{key:"contact_error_cm",label:"접촉 오차 cm",renderCell:fmt},{key:"points",label:"내부 점수",renderCell:v=>v??"—"},{key:"failure",label:"실패 코드",renderCell:v=>v||"—"}];
 return <article className="report-page team-report">
  <header><p className="team-eyebrow">DRONE3 / MISSION 3 / TEAM HANDOFF</p><h1>미션3 개발·검증 자료</h1><RichNarrative id="team-intro" value="최신 직선 트랙 16조건 전체 · 2026-10-09\\n\\n환경 → 제어 구조 → 센서 → 시험 결과 → 재현 자료 순으로 정리했습니다." /></header>
  <nav className="team-nav" aria-label="보고서 목차"><a href="#arena">환경</a><a href="#architecture">구조</a><a href="#results">전체 결과</a><a href="#hardware">센서·한계</a><a href="videos.html">전체 영상 보기 ↗</a></nav>
  {content.sections.map(s=>{
   const q=queryFor[s.id],evidence=snapshot.queries[q].rows;
   return <div id={s.id} key={s.id} className="team-section">
    <ReportSection id={"team-"+s.id} title={s.title} queryId={q} sourceRows={evidence} showHeading={false}>
      <RichNarrative id={"team-"+s.id+":body"} value={"## "+s.title+"\\n\\n"+s.body} sourcePreviews={previews}/>
      {images[s.id]&&<img className="team-diagram" src={images[s.id]} alt={s.title+" 설명 도면"}/>}
    </ReportSection>
    {s.id==="architecture"&&<DataComponent id="module-table" title="코드 역할 지도" queryId="mission3_structure" sourceRows={evidence} displayRows={evidence} kind="table"><DataTable rows={evidence} columns={[{key:"role",label:"역할"},{key:"file",label:"파일"},{key:"responsibility",label:"책임"}]} caption="현재 미션3 모듈" searchable={false}/></DataComponent>}
    {s.id==="results"&&<><DataComponent id="trial-table" title="16조건 전체 · 같은 seed 1001 / 각 1회" queryId="mission3_trials" sourceRows={rows} displayRows={rows} kind="table"><DataTable rows={rows} columns={columns} rowKey="case_id" caption="직선 시험 16회 전체 결과"/></DataComponent><p className="team-media-link"><a href="videos.html">16조건의 제3자·기체 영상과 원평가 열기 →</a></p></>}
   </div>;
  })}
  <ReportSection id="team-refs" title="출처와 적용 범위" queryId="mission3_sources" sourceRows={snapshot.queries.mission3_sources.rows} showHeading={false}><RichNarrative id="team-refs:body" value={"## 출처와 적용 범위\\n\\n"+content.references.map(([t,u,n])=>"- ["+t+"]("+u+"): "+n).join("\\n")} sourcePreviews={previews}/></ReportSection>
 </article>;
}
'''
    # Keep real Markdown newlines in the JS literals (escaped once for JavaScript).
    (app/'src/content/report/ReportContent.jsx').write_text(jsx)
    (app/'src/content/report/report.css').write_text('''.team-report{--data-app-layout-intent:authored-report;--data-app-content-width:920px;max-width:960px;margin:0 auto;padding:36px 20px 64px}.team-eyebrow{font-size:12px;letter-spacing:.13em;color:var(--muted);margin-bottom:12px}.team-nav{display:flex;flex-wrap:wrap;gap:18px;margin:26px 0 34px;padding:14px 0;border-block:1px solid var(--border)}.team-nav a,.team-media-link a{color:var(--text);text-decoration:underline;text-underline-offset:4px}.team-section{scroll-margin-top:90px;margin-bottom:42px}.team-diagram{display:block;width:100%;height:auto;margin:22px 0}.team-media-link{margin:20px 0}.team-report h1{line-height:1.2}@media(max-width:600px){.team-nav{gap:12px}.team-diagram{min-width:0}}''')

def video_index():
    payload=json.dumps(ROWS,ensure_ascii=False).replace('</','<\\/')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>미션3 · 직선 16조건 영상</title><style>*{box-sizing:border-box}body{font:16px/1.6 system-ui,"NanumGothic",sans-serif;background:#f3f6f8;color:#183348;margin:0}main{max-width:1120px;margin:auto;padding:30px 24px}h1{font-size:30px;margin:12px 0}a{color:#087d98}header p{max-width:900px}.controls{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin:20px 0}select,button{font:inherit;padding:8px 12px;border:1px solid #a5bbc9;border-radius:8px;background:white;color:inherit}video{width:100%;max-height:610px;background:#142938;display:block;border-radius:10px}.note{background:white;border-left:4px solid #168eaa;padding:15px 20px;margin:18px 0}.table-scroll{overflow:auto}table{border-collapse:collapse;width:100%;white-space:nowrap;background:white}th,td{padding:10px 13px;border-bottom:1px solid #dae3e9;text-align:left}th{background:#e7eef3}td.num{text-align:right}.fail{color:#a34627;font-weight:700}tr.active{background:#e7f6fa}#status{font-weight:700}.tiny{font-size:13px;color:#546b7c}@media(max-width:600px){main{padding:20px 12px}h1{font-size:25px}}</style><main><header><a href="team-report.html">← 미션3 전체 보고서</a><h1>직선 트랙 16조건 · 전체 영상</h1><p>옆거리 1.25→0.50m, 5cm 간격, 각 1회, 같은 seed 1001. 조건부 완료 14 / 실패 2. 제3자 영상은 1배속이며, mono 320×320 기체 영상은 고정 20FPS 기록 프레임 미리보기입니다. 기체 영상의 재생 시간은 전체 시험 시간축과 다를 수 있습니다.</p></header><div class="controls"><label>조건 <select id="trial" aria-label="시험 조건"></select></label><label>시점 <select id="view"><option value="video">제3자 1배속</option><option value="camera">기체 카메라</option></select></label><button id="prev">이전</button><button id="next">다음</button><span id="status"></span></div><video id="player" controls preload="metadata"></video><div class="note" id="detail"></div><p class="tiny">H264는 공유용 미리보기입니다. 원픽셀 동일성의 근거는 별도 원자료 ZIP의 FFV1/입력 SHA입니다. 기체 영상은 로그 t로 시간 정렬해야 하며, 1.15m는 약 7.2초 기록 공백이 있습니다. 제3자 시점은 독립 검토 승인을 의미하지 않습니다.</p><div class="table-scroll"><table><thead><tr><th>옆거리 m</th><th>ID</th><th>완료</th><th>접촉 오차 cm</th><th>점수</th><th>원평가</th><th>영상 직접 열기</th></tr></thead><tbody id="rows"></tbody></table></div><p class="tiny">1.15m는 접촉 점수가 있지만 PWM 0/3초 지지 미완료로 실패입니다. 0.50m는 인식 후 영상 소실로 실패했습니다. 모든 행·영상·평가를 포함하며 공식 점수/실기 성능을 의미하지 않습니다.</p></main><script>const data=__DATA__;const trial=document.getElementById('trial'),view=document.getElementById('view'),player=document.getElementById('player');data.forEach((r,i)=>{let o=document.createElement('option');o.value=i;o.textContent=r.distance_m.toFixed(2)+'m · '+(r.contact_complete?'조건부 완료':'실패');trial.append(o)});document.getElementById('rows').innerHTML=data.map((r,i)=>'<tr data-i="'+i+'"><td class="num"><button class="pick" data-i="'+i+'">'+r.distance_m.toFixed(2)+'</button></td><td>있음</td><td class="'+(!r.contact_complete?'fail':'')+'">'+(r.contact_complete?'완료':'실패')+'</td><td class="num">'+(r.contact_error_cm==null?'—':r.contact_error_cm.toFixed(3))+'</td><td class="num">'+(r.points??'—')+'</td><td><a href="'+r.evidence+'" target="_blank">JSON</a></td><td><a href="'+r.video+'">제3자</a> / <a href="'+r.camera+'">기체</a></td></tr>').join('');function show(){const r=data[Number(trial.value)];player.pause();player.src=r[view.value];player.load();document.getElementById('status').textContent=(r.contact_complete?'조건부 완료':'실패')+' / ID 인식 있음';document.getElementById('detail').textContent=r.case_id+' · '+(r.failure||'실제 패드 접촉 → PWM 0 → 직선 지지 3초 완료')+' · 접촉 오차 '+(r.contact_error_cm==null?'없음':r.contact_error_cm.toFixed(3)+'cm')+' · 실제 최고 속력 '+r.max_actual_speed_m_s.toFixed(5)+'m/s · 기체 기록 t='+r.camera_start_s.toFixed(2)+'~'+r.camera_end_s.toFixed(2)+'s / 최대 기록 간격 '+r.camera_max_record_gap_s.toFixed(2)+'s';document.querySelectorAll('tbody tr').forEach((e,i)=>e.classList.toggle('active',i===Number(trial.value)))}trial.onchange=show;view.onchange=show;document.getElementById('prev').onclick=()=>{trial.value=Math.max(0,Number(trial.value)-1);show()};document.getElementById('next').onclick=()=>{trial.value=Math.min(15,Number(trial.value)+1);show()};document.querySelectorAll('.pick').forEach(e=>e.onclick=()=>{trial.value=e.dataset.i;show();player.scrollIntoView({behavior:'smooth',block:'center'})});show();</script></html>'''.replace('__DATA__',payload)
    (OUT/'videos.html').write_text(page)

if __name__=='__main__':
    svg_assets(); markdown(); app(); video_index()
    print('Authored Korean report, app content, portable video index and 3 diagrams.')
