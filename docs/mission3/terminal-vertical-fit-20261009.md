# 착륙 말기 수직 속도 추정 연구 — 2026-10-09

현재 7개 저장 입력을 재생했으며, 원 선형 0.4초 필터의 높이·속도는 모두 오차 1e-9 미만으로 재현됐다. 이 결과는 새 비행이나 착륙 성공이 아니다. 영상 보정이 종료된 TOUCHDOWN_DESCEND 시점의 동일 추정 상태를 초기값으로 사용하고, 표면 분류·센서 입력·실제 궤적은 원 실행 값으로 고정했다. 참값은 추정 계산과 재현 일치 검사가 끝난 뒤 평가에만 읽었다.

거리 측정의 선형 기울기는 일정한 가속도에서도 현재 속도와 다르다. 후보는 수신 시점까지 측정한 수직 가속도를 적분해 거리 이력에서 가속도 변위 D(t)를 빼고, h(t)-D(t)의 기울기에 현재까지의 속도 증가량을 더한다. IMU 구간 평균이 오른쪽 끝 시각으로 기록됐다는 가정은 S이며, 중력 보정·자세·잔류 가속도 편향과 실제 센서 시간 동기화는 검증되지 않았다.

[Forster 외의 원 논문](https://arxiv.org/html/1512.02363)은 IMU 적분에서 회전, 잡음, 편향 보정까지 다룬다. 여기서 구현한 것은 스칼라 수직 후보식이며 논문의 전체 manifold/factor-graph 추정기가 아니다. 논문을 현재 기체의 성능 보증으로 사용하지 않는다. RSS 공식 초록도 읽었고 PDF 직접 조회는 timeout이었다.

| seed / 원 실행 | 원 .4초 선형 | .4초 IMU | .8초 선형 | .8초 IMU | 1.2초 선형 | 1.2초 IMU |
|---|---:|---:|---:|---:|---:|---:|
| 433 / v11 | 1.523 | 1.125 | 1.567 | 1.183 | 2.232 | 0.638 |
| 457 / v11 | 12.131 | 10.212 | 2.453 | 4.180 | 9.388 | 1.947 |
| 467 / v11 | 0.520 | 0.484 | 2.920 | 0.502 | 7.374 | 0.617 |
| 479 / v11 | 4.467 | 3.263 | 5.001 | 5.047 | 0.431 | 2.605 |
| 563 / v13 | 1.888 | 1.216 | 2.600 | 0.720 | 3.510 | 0.518 |
| 571 / v13 | 10.931 | 12.746 | 5.254 | 3.211 | 12.744 | 0.557 |
| 577 / v13 | 3.511 | 2.521 | 7.970 | 5.084 | 7.953 | 5.294 |

표는 마지막 새 유효 거리 측정 이후 높이 RMSE(mm)다. 571의 .4초 IMU 후보는 오히려 나빠졌다. .8/1.2초는 실패457·571의 오차를 줄였으나 성공479·577 등의 일부 오차는 늘었다. 따라서 “모든 상황 개선”이나 기존 실패를 성공으로 바꿨다고 주장하지 않는다.

백색·독립 거리 잡음 σ, 고정 측정 간격 Δt라면 선형 기울기의 분산은 대략 12σ²Δt/T³이다(D). 실제 잡음 상관·표면 혼합·잘못된 시각이 있으면 이 근사는 적용되지 않는다. IMU 편향 b를 빼지 못한 가속도 보정에는 대략 bT/2의 속도 오차가 남는다(D). 긴 구간은 거리 잡음을 줄이는 대신 편향 영향을 키우므로 .4/.8/1.2초를 모두 검토했다. 수치 구간과 필터 gain은 S이며 제조사 권고가 아니다.

현재 제어 코드에는 기본 선형 동작을 유지하면서 명시적 opt-in `imu_compensated`를 추가했다. IMU 보정 fit은 측정 시각의 중복·미래·IMU 시간 누락/큰 간격을 거부한다. 현재 wrapper는 보정 fit 거부 때 기존 선형식으로 돌아가며 횟수를 기록하므로, 이 검사가 모든 거리 높이/선형 융합을 차단한다고 해석하면 안 된다. 현재 합성 producer 시각은 과거/현재이지만 실제 입력 수신 계층의 미래/중복 packet 정책은 추가 설계가 필요하다. 원 .12 높이/.08 속도 gain과 접촉·prediction 시간·포화·영상 문턱은 유지한다. 203개 테스트가 통과했다. 최초 한 테스트는 fixture의 잘못된 entry 키로 실패했고 키 수정 전후 로그를 모두 보존했다.

06:46 KST 전후 v14 물리 시험 12개를 사전 고정했다. 기존에 사용한433·467·479·541·571·577에 대해 .8/1.2초 각각6회다. 4개 독립 CPU/GPU 프로세스만 사용하며 한 비행의 인과적 순서는 유지한다. 전체 완료·원자료·명령 포화·정확 픽셀 검사가 끝나기 전 정책을 선택하지 않는다. 같은 원 차량 phase와 scan15 조건이며, 입력만 재생하는 시험과 달리 online 분류와 피드백으로 실제 새 궤적을 만든다. 결과는 별도 v14 분모로 기록하고 83회 이전 시행과 성공률로 혼합하지 않는다.

[저장 입력 7건 재생](../../artifacts/mission3/acquisition/20261009-terminal-filter-replay-v1/summary.json) / [6개 후보 구간 비교](../../artifacts/mission3/acquisition/20261009-terminal-window-sensitivity-v1/summary.json) / [v14 물리 사전 계획](../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-training-v14/plan.json)

제조사 일치·실기체·qualified·formal 성공은0이다. 실기 장착/K/D/질량·추력/센서 품질/마찰·전송 지연/바이어스·배터리 예산 문제가 남아 진입2 진행을 승인하지 않는다.

## 07:06 KST 물리 시험 완료 및 별도 미사용 시험

사전 고정한 v14 .8/1.2초 각각6개가 모두 actual pad contact/PWM off/3초 유지/straight/정상 명령 포화를 통과했다. source65/config/runtime 동결과 native31,461 정확픽셀 비교도 통과했다. 이6개는 기존에 본 seed이고 원 성공5개 유지 및 이전 실패571의 새 비행을 확인하는 개발 시험이다. 원 실패 v11/457과 v13/571 원 로그·원 판정은 그대로 실패다. 바뀐 추정기로 새로운 물리 궤적·새 영상/센서 입력이 생겼으므로 “원 같은 입력의 실패를 고쳤다”는 인과 보증은 하지 않는다.

두 정책은6/6으로 같아 사전 tie 기준인 실제 새 비행의 마지막새유효range 이후 높이RMSE 최댓값으로 비교했다. .8초5.058mm /1.2초3.927mm로 imu12를 후속 후보로 선택했다. 수치 후보·seed·source SHA는 [선택 기록](../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-training-v14/selection.json)에 있다. 이12개 terminal 구간은 linear fallback0/미래range stamp0이었으며 실제 하드웨어 packet 처리를 검증한 것은 아니다. 독립 검토자에게12건 별도 검토를 요청했다.

이후 처음 고정한 미사용601·607·613·617·619에서는2/5 접촉 완료였다.607 접촉127.60/off128.48/131.48 hold 끝/XY29.041mm,619 접촉243.76/off244.64/247.64 hold 끝/XY9.375mm다.601 decoded20·metric0,613 decoded21·metric0,617 decoded0·metric0으로 모두300초 탐색 실패/접촉0/off없음이다. 전체5 caps 통과/native24,758exact/source65+runtime+config 동일. training6/6과 혼합하지 않는다. [holdout 원 결과](../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-holdout-v14-v2/summary.json). 처음 --split unused_holdout 옵션 오류로5건 모두 argparse에서 거부됐으며 비행/영상/run 생성0이었다. 원 오류 기록을 보존한 v2는 --split holdout과 retry 로그 이름만 고치고 seed·기체·조건은 유지했다. 이5개의 preflight 거부는 물리100회의 분모와 별도로 기록했다.

571 새 비행은 실제 접촉106.26/off107.16/hold끝110.16s, 내부9점/XY14.585mm다. qualified/formal0이고 독립 검토는 진행 중이다. [전체1× 영상](../../artifacts/mission3/replays/20261009-imu12-v14-571-full/full-flight-1x-h264.mp4)은551frames/5FPS/110.2s, 원 simulation110.18s와 대응한다. 시작25s 진입자리 ENTRY_SCAN/높이1m,108s HOLD/네발/PWM0를 시각 확인했다. 지도 삼각형은87°화각 표시이며 인식 한계가 아니다. 전 구간의 pass/내부점수 표시는 오프라인 최종 요약이다. 실시간 현재 접촉 발/PWM/phase와 구분한다. 마지막 카메라 영상은 timestamp age를 표시하며 새 영상처럼 꾸미지 않는다. 파생 render 준비에서 raw motors/XML 연결 누락으로 두 번 실패한 로그도 보존했고 원 비행은 바꾸지 않았다. 원13파일 SHA 불변/영상전체 frame/fps/size 검사와 [provenance](../../artifacts/mission3/replays/20261009-imu12-v14-571-full/replay-provenance.json)를 제공한다.

[말기 높이 비교 그림](../../artifacts/mission3/acquisition/20261009-terminal-height-571-comparison-v1/terminal-height-comparison.png)은 별도 두 비행을 각각 첫 접촉에 맞춰 그린 오프라인 평가다. 상단 원실패는 추정높이가 S uppergate26.5mm를 넘어서0.25s 지지 확인을 못 채웠고, 하단 새 비행은 추정높이가 범위 내에 머문다. 원 서로다른 영상fix/센서/궤적을 같다고 가정하지 않는다. 모델·센서·실기 미확정은 여전히 남는다.

## 독립피드백에따른 v16 입력시각수정

v14실제training12서명과holdout2접촉/3실패감사완료·원평가binding검증. 원snapshot함수는 now.055에IMU.06을허용하여futureacc0→1 변화가currentfit−.020→−.015m/s로들어가는반례가있었다. 현source는it[-1]>now를거부하며current동시stamp는허용한다. zero/nonzerofuture와다음floatfuture경계시험을추가,204tests통과. [반례전후](../../artifacts/mission3/acquisition/20261009-vertical-causality-regression-v1/after.json).

원v14imu12sameconfig/physics spent571619613의3회source회귀에서원2접촉/1metric0timeout가유지됐다. 전3의commands/truth/observations/motors/sensors/deck_samples/estimates/rpyt_packets/camera_frames/camera.avi 30개SHA가각baseline과같다. [회귀비교](../../artifacts/mission3/acquisition/20261009-causal-imu-regression-training-v16/same-input-source-regression-comparison.json), [전3summary](../../artifacts/mission3/acquisition/20261009-causal-imu-regression-training-v16/summary.json). 새source의원평가/서명결합은별도이며독립담당이새검토중,기존v14승인을복제하지않는다. 전체물리121을독립성능분모로합치지않는다.

이검사는scalarfit의futureIMU입력만수정했다. legacyfallback및높이갱신의전체packet거부정책,producer자세/lever20ms정렬/actualtimestamp동기화/실기quality·에너지·조립spec U는그대로다. Qualified/formal/hardware0이다.
