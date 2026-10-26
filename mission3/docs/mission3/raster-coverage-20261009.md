# 진입 후 탐색 범위 연구 — 2026-10-09

v14 imu12의 신규 미사용5개 중3개는 decoded ID가 있거나 없더라도 metric pose가0인 탐색 실패였다. 접촉/높이 gate를 넓혀 해결할 실패가 아니다. v13 실패521·557·569의 원 차량 시간/위치를 오프라인 평가 대상으로만 사용해, 알려진 진입 위치와 경기장 경계에서 만든 연속 경로를 비교했다. 차량 진실 상태는 경로 생성·제어 입력으로 쓰지 않았다.

[PAMPC 원 논문](https://arxiv.org/html/2302.04708)은 목표 가시성과 구동 한계를 함께 다루며 MATLAB 폐루프 시험을 제시한다. 이 작업에는 논문의 NMPC를 구현하지 않았고, 미관측 차량의 상태를 가정해 최적화하지도 않았다. 가시성·시간·구동 한도를 함께 평가해야 한다는 설계 근거로만 참고했다.

현재 무왜곡 pinhole S 모델에서 native320/H87°로 f≈168.605px이다(D). 정면15cm 코드의 투영 폭은 f·0.15/depth이므로18px를 만족하는 이상적 정면 깊이는 약1.405m 이내다(D). 이는 제조사 인식 거리 보장이 아니다. 비스듬한 판의 짧은 변, 높이/자세로 인한 frame 탈락, 실제 K/D/PSF·quietzone·픽셀 alias·노출·배경·검출기가 유효 범위를 더 줄인다. native320을 확대하거나 문턱18px를 낮추지 않았다.

첫 연구는 ideal self-position/yaw와 direct velocity following을 가정한16개 범위/속도 경로를 만들었다. 원6DOF·모터·바이어스·Flow/ToF·영상 관측 및 occlusion이 없으며, 어떤 경로에도 decoded observation을 주입하지 않았다. 7px/18px는 투영 기회이며 실제 인식·pose·착륙이 아니다. 실제569 원기하 최대7.43px와 이상 경로의 원8/.75/.15 설정22.28px가 달라 이 proxy의 낙관성을 확인했다.

두 번째 연구는18개 범위/시선 proxy를 비교했다. leg 방향 시선으로 바꾸는 것이 개선을 보장하지 않아 실제 제어기에 넣지 않았다. 다음 표의 선택 후보는 기존 entry-forward 시선을 유지하며 탐색 속도.15m/s 및 뒤방향.05m/s도 유지한다.

| spent seed | forward6 / lateral.75 첫18px 기회(s) | forward10 / lateral3 첫18px 기회(s) |
|---|---:|---:|
| 521 | 290.9 | 277.3 |
| 557 | 284.7 | 272.5 |
| 569 | 288.4 | 275.4 |

시각은 Mission3 시작 기준이고 모든 표의 기회는 늦다. 현재300초는 미발견 탐색 제한이고, 별도 전체 Mission3 제한은360초다. 정렬·고도변경·접근·접촉 확인이 전체360초 내에 가능한지는 원 동역학·영상·센서 시험으로 검증해야 한다. 원경로의 실제 odometry drift 때문에 알려진 기하상의 기회가 실제 pixel 인식과 같지 않다.

v15 사전 고정 물리18회는 두 범위 각각9회다. 기존 검색 실패521557569601613617과 성공 유지571607619를 모두 포함한다. 원 v14imu12/scan15·gaze·속도·height1·net·scene·원 seeded 차량 운동·320mono·18px·contact/prediction/명령 포화는 유지한다. 경로는 진입 기준이며 track rendezvous/대기점이 아니고 모서리 간 계속 이동한다. 양 정책 전체9개가 끝난 뒤 metric 획득과 실제 접촉+caps 및3개 기존 성공 유지를 함께 판단한다. source65/config 동결이고 새 독립 agent를 만들지 않았다.

[첫 투영16개](../../artifacts/mission3/acquisition/20261009-entry-raster-coverage-projection-v1/summary.json) / [시선·범위 proxy18개](../../artifacts/mission3/acquisition/20261009-entry-raster-heading-coverage-projection-v2/summary.json) / [v15 사전 계획](../../artifacts/mission3/acquisition/20261009-raster-extent-training-v15/plan.json)

후속 검토 지침: v15가 늦은 metric 획득 이후 시간 부족으로 실패한다면 실제 단계별 시간/남은 직선 margin을 먼저 분석한다. 원 prediction6초·접촉.25초를 늘려 성공으로 만들지 않는다. 초기1m는 유지하되 향후 탐색 고도 변경은 카메라의 수직 framing과 하강 준비시간을 줄일 수 있는 S 후보로만 새 소스/시험을 사전 고정해 검토할 수 있다. 현재 제어기에 넣지 않았다. native 화각/카메라 장착·기체 개조 없음. 제조사 일치·실기/qualified/formal0, 진입2 진행0이다.

07:20 KST 독립 피드백: v14training12 실제 독립 검토와 원평가 binding12를 확인했다. 미래 IMU history 반례는 현재 source API 결함이며 실행 로그 해당 입력0과 구별한다. v15 전체18/aggregate source동결 종료 후 수정·회귀가 필요하다. 진행 중 partial 결과를 최종 정책 선정에 사용하지 않는다.

## 추가 1차 자료와 적용 범위

[Keipour et al., 2104.01272v2](https://arxiv.org/html/2104.01272v2) §3.1/3.3은 영상 특징 오차 피드백에 차량 속도 FF를 더하고, 짧은 직선 구간의 속도 추정을 이용하는 근거다. 본 구현의 센서 기반 속도 FF+상대 위치 피드백과 공통 원리가 있지만 같은 알고리즘은 아니다. 논문의 도로 위 hover prior/짐벌/자석/다중 point laser 및 다른 기체는 현 조건과 다르므로 채택하지 않았다. 이미지 Jacobian/IBVS로 바꾸려면 고정 전방 카메라의 가시성, 깊이·실제 K/mount·포화, 근접 마커 소실을 별도 검증해야 한다. 실패를 전체 분모에서 제외하는 문헌의 시험 범위를 우리 전체 미션 성공으로 대신하지 않는다. PMC는 captcha, 추측한 v3 HTML은404였으며 실제 arXiv 링크의 v2를 읽었다.

[Crazyflie2.1 공식 Rev3 데이터시트](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf) p1/2/3의 stock battery 공칭7분, 기체29g,250mAh와 [제조사 FAQ](https://www.bitcraze.io/support/f-a-q/)의 사용 조건 의존을 재확인(P). [AI-deck1.1 Rev2](https://www.bitcraze.io/documentation/hardware/ai_deck_1_1/ai_deck_1_1-datasheet.pdf) p1의4.4g/320mono/전원최대300mA도 재확인(P). 최대 전류를 실제 평균 부하로 쓰지 않는다. 덱을 모두 단 상태의 endurance와 Mission1/2 이후 SOC는 U다. 현 plant의 상수4.2V/35g S 설정은 이를 검증하지 않는다. 343초 모델 착륙을7분 공칭 아래라는 이유로 장비 가능 판정하지 않는다.

## v15 전18 물리 결과 — 기존 대안 채택 없음

forward6/.75는2/9,wide10/3는4/9 조건부 contact+normalcaps다. 원3positive571607619 중619가 양쪽에서 실패해 사전 retention 조건을 충족하지 않는다. 521·569의 새 착륙은 일부개선이지만 전체9 기준을 대체하지 않는다. v14 원8/.75 구성은 그대로 남긴다. [전체18 summary](../../artifacts/mission3/acquisition/20261009-raster-extent-training-v15/summary.json), [선정 거절](../../artifacts/mission3/acquisition/20261009-raster-extent-training-v15/selection.json), [단계·추정 진단](../../artifacts/mission3/acquisition/20261009-raster-extent-training-v15-phase-analysis/summary.json).

전체18에 정상명령 포화가 유지됐고89,782개 native320 FFV1 pixel SHA가 원입력과 일치했다. 전체 physics 원장100→118이며 서로다른source/split/재사용실험을 일반 성공률로 합산하지 않는다. main 개발평가는 무서명이며 기존 독립담당에게 이새18별도검토 요청, 완료전 새승인0.

forward6의521557569601613은simulation292.14~300.70s에 metric으로 전환했으나그후~8~10초안에 conditional own-position std가1m 상한을 넘어abort였다. 이는 실제 위치오차1m라는 뜻이아니며 잡음/편향/yaw 공분산S 가정이다. 617은 coarse 단계에서 같은abort,619는metric0 timeout. wide10/3의521569는metric285.28/287.74s 이후착륙했지만557601613은metric0 timeout,617619는 불확실성abort다. 양쪽late-acquisition·공통position gauge·가시성·시간 문제가 남는다. cov gate를풀거나truth로공분산을줄이지 않았다.

v16은 탐색대안의 개선실험과 분리된 입력검증 소스 회귀다. 미래IMU 반례를 원snapshot에서재현한 후 fitter가 it[-1]>now를거부하도록고쳤고204tests통과. 원v14imu12 config/기본경로8/.75/scene/문턱으로 spent571619613의3회 계획. 현재회귀완료전결과/서명은미확정이다. range 높이갱신/legacyfallback전체 timestampgate는 별도U라 이함수guard만으로 외부adapter가검증됐다고 하지않는다.

## 다음 paired 후보 v17 — 결과 미확정

S 가설은 COARSE_ACQUIRE에서 카메라 yaw 정렬을 기다리는 동안, 최신 확정 decoded-ID의 세계 방위로 수평 이동하는 것이다. 속도 FF는 유효 자세/영상속도 추정 이후에만 이용하며 차량 설정속도.25와truth는 입력하지 않는다. 후보는 기존 코드의 turnerror40°/turnspeed.35m/s/freshdecodedbearing 옵션이다. 기존 forward 목표.35와 global.4/.15/yaw10°/s 포화는 같다. age.15s/base1.5m/total4m/episode20s/pixel근접36px/거친방위7px/metric18px도 유지한다. 제한된 방위 기반 접근이며 IBVS 전체식·차속·자세 정확도를 보장하지 않는다.

저장 입력5건 재생에서 baseline COARSE 속도/단계가 완전히 재현됐고, 일부 정지 구간48/64개 표본(50Hz)에 후보 접근 명령이 나타났다. 궤적/영상/물리를 갱신하지 않은 계산 검증이므로 착륙 증거는 아니다. v10의 이전scan45/다른시선/.4turnspeed 시험 실패를 보존하고 동일 성과라고 하지 않는다. v17은 현재 causal-IMU source의 같은14조건×기존/후보=28재사용 개발 시험이다. 기존 성공8개 모두유지+전체14접촉/포화가 기존 방식보다 엄격히 좋아질 때만 채택한다. 전체 종료전 성공 부분집합으로 선택하지 않는다. config/preplan/활성실행ID는 work-log를 따른다.

## v17 전체28회 비교 결과

2026-10-09 07:53 KST 전체동결집계: baseline8 8/14, fresh decoded bearing40deg/.35m/s 8/14, 각정책positive8 retention통과지만 strictimprovement0으로후보채택안함. normalcap28/28, native116183 frames exact, qualified/formal/hardware0. 실패12를보존하며 후보601은std1m초과로실패형태가변함. 픽셀방향접근의부분지표개선은전체접촉성공증가가아니다. 기존raster8/.75·baselinecoarse정책유지, 거절후보unusedholdout0. 전체원계획·평가·selection은 acquisition/20261009-coarse-bearing-paired-training-v17/에, sensor/estimate-only phase진단은 v17-phase-analysis/summary.json에있다. 독립검토진행중이며자체서명없음.
