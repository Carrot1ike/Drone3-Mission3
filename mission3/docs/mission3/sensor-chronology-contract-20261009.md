# 센서 시간 검사: v19

2026-10-09. 사용자 환경·카메라·포화·착륙 조건을 유지한 소프트웨어 입력 계약이다. 실제 통신 시계 동기화나 안전 회수 구현을 승인하는 자료가 아니다.

## 확인한 결함과 수정

기존 `PhysicalMotion.state_from_deck`는 PC 관측기의 시간 검사를 호출하기 전에 공식 EKF의 ToF/Flow 측정 갱신을 수행했다. 기존 `SelfMotion.update`는 producer 시각을 검사하지 않고 높이를 갱신했다. IMU 보정 회귀가 미래 표본을 거부해도 `fitted_range_velocity`의 모든 ValueError가 선형 회귀 fallback으로 들어가 미래 거리 이력을 사용했다. 중복 consumer와 NaN 시각도 일부 경로를 통과했다.

반례 6개 시험의 하위 조건에서 16 failures/1 error를 재현한 원로그를 보존했다. Infinity를 정수로 바꾸다 난 OverflowError도 원 실패다. 수정 후 같은 시험은 통과했고 전체 217개 시험도 통과했다. 새 시험은 위치·속도·이력·fallback 횟수뿐 아니라 공식 EKF 상태·측정 횟수·PC 가속도 accumulator가 거부 전후 불변인지 검사한다. 실제 무선 시험은 아니다.

`sensor_time.py`의 계약은 다음과 같다.

- consumer stamp는 finite이며 첫 입력 이후 엄격 증가한다. 기존 최대 0.2초 간격을 유지한다.
- 거리 producer stamp는 finite이며 consumer 이후일 수 없다. 새 측정은 이전 새 producer보다 엄격 증가해야 한다. held 값은 같은 producer stamp를 반복해도 되며 다시 높이/ToF 측정으로 소비하지 않는다. held metadata가 지난 새 producer보다 뒤로 가는 것은 거부한다.
- 물리 어댑터에서 deck 갱신 전에 최신 IMU/Flow producer가 미래인지 확인한다. 이 검사는 공식 ToF/Flow 호출과 PC accumulator 소비보다 앞선다.
- 회귀 이력의 nonfinite·중복·역순·미래 표본은 선형 fallback으로 우회하지 못한다. 유효한 거리 이력에 IMU 범위가 부족한 경우의 기존 명시적 선형 fallback은 유지한다. bias/coverage/gap에 대한 새 수치 개선은 주장하지 않는다.

현재 모형에서는 모든 stamp가 하나의 simulation clock에 있다(S). 실제 Flow delta의 interval, ToF enqueue/capture 시각, PC radio arrival 시각, 동기화 offset/drift는 미확정(U)이며 이 함수를 적용하기 전에 공통 시간축의 정의가 필요하다. 소비 시각에서 held ToF를 현재 자세로 변환하는 기존 근사도 유지된다. 검사 대상은 telemetry 소비와 scalar fit이며 1kHz IMU 수신 전체·카메라 수신·네트워크 replay 공격 검증은 아니다.

공식 Flow/ToF의 시간·입력 계약은 [고정 펌웨어 근거](flow-tof-runtime-contract.md)에 있다. 공식 원본을 수정하지 않고 프로젝트 wrapper에서 검사한다(C: 현재 로컬 코드). 나쁜 packet은 ValueError로 거부한다. 실제 링크 손실이나 거부 이후 계속 비행할 정책은 별도 설계가 필요하며, 이 오류 검사를 비상 착륙이나 모터 종료로 부르지 않는다. [실패 명령 전달](abort-delivery-contract-20261009.md)의 조건부 관측 종료와도 다른 계약이다.

## 사전 회귀 계획

v18b의 동일 spent 사례 네 개를 training으로 다시 사용한다: 기준 571/619 접촉 성공, 기준 613 탐색 시간 만료, wide10×3/619 추정 불확실성 실패. 착륙 기능 개선 시험이나 unused holdout이 아니다. source/runtime/config를 동결하고 네 개 모두 원 10개 raw 파일 byte 일치, strict 접촉 평가, 정상 명령 포화, native FFV1 픽셀 검사를 요구한다. 어느 하나라도 바뀌면 정상 경로 보존 회귀가 통과했다고 하지 않는다. 새로운 source에 이전 독립 서명을 복제하지 않는다. 제조사·실기·qualified 성공은 0이며 진입 2로 진행하지 않는다.

시험 계획과 새 결과는 `artifacts/mission3/acquisition/20261009-sensor-chronology-regression-training-v19/`에 보존한다. v18/v18b·이전 반례·원 성공과 실패는 그대로 남긴다.
