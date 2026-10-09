# v20 보존 이력 검사 계약

2026-10-09 08:50 KST. v19 독립 검토의 내부 이력 주입 반례를 수정하는 별도 소프트웨어 회귀다. 정상 네 비행에서 해당 오류 발생은 0이며, 새 착륙 능력 시험이나 실기 안전 회수 승인이 아니다.

`SelfMotion.validate_sample_time`는 packet 시각 외에 보존된 range/IMU 이력의 finite 값·형상·엄격 증가·미래 시각을 검사한다. 빈 이력과 단일 표본은 정상 시작 상태로 허용한다. 새 유효 range 또는 IMU를 추가할 때 마지막 이력과 중복/역순이 될지도 융합 전에 검사한다. `PhysicalMotion.state_from_deck`의 같은 호출 위치는 공식 C deck 융합·PC accumulator 소비보다 앞이다. 잘못된 이력을 뒤에서 잘라 버리거나 선형 fallback으로 숨기지 않는다.

새 CPU 시험 2개(내부 오류 16개, prospective append 오류 4개)는 실제 SelfMotion/PhysicalMotion 호출의 상태·covariance·producer latch·C 상태·measurement counts·accumulator·attitude history가 오류 전후 같음을 검사한다. 수정 전 20 실패 로그를 보존한다. 기존 packet/fallback 시험도 유지한다. 모든 수치 입력 및 임의 C 실패에 대한 전체 transaction/rollback을 주장하지 않으며, 1kHz IMU 수신·이미지·radio 시각이나 ValueError 이후 실기 제어 전달/복구 정책은 여전히 미검증이다. 공중 모터 종료를 오류 복구 정책으로 도입하지 않는다.

물리 회귀는 v19의 같은 spent training 4개(기준571/619 접촉, 기준613 timeout, wide619 uncertainty)를 재사용한다. 카메라 native mono320, 환경·원 차량 phase·포화·불확실도 문턱·접촉 판정·controller/plant/runtime은 변경하지 않는다. 전10 raw files×4가 해당 v19와 byte 동일하고 포화·실제 접촉·FFV1 pixel 검사를 통과해야 정상 입력 보존으로 판단한다. 전체 분모와 오류를 유지하고, 독립 검토 전 main의 결과를 독립 승인으로 계상하지 않는다. 이번 source는 과거 v19 서명의 적용 범위 밖이다.

계획과 이 계약의 copy/SHA는 `artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/`에 보존한다. 09:00 KST 이후 새 실험을 시작하지 않는다. 완료되지 않은 경우 부분 실행을 완전한 시험·성과 분모로 바꾸지 않는다.
