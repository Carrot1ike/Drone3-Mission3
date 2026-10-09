# v11 unused holdout5 독립 접촉 검토 — 2026-10-09

Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. **503·509·523의 새 원로그 접촉을 개별 승인했다. 전체5분모 중 조건부 모델 contact+command-envelope3, 실패521·541 두 개를 유지한다. 제조사 qualified·실기·formal 승인0.** 별도 training3/4와 합산해 일반 성공률 또는 진입5성공으로 표현하지 않는다. 서명은 본인 입력 SHA·실행 source/runtime/config·원 평가에 새로 묶었으며 v9/training 서명을 복제하지 않았다.

| seed | 실제 첫접촉(s) | PWMoff → 3초 끝(s) | 내부 CoM오차 / 점수 | 독립 판정 | 근거 |
|---|---:|---:|---:|---|---|
| 503 | 181.00 | 181.94 → 184.94 | 15.532mm / 9 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-holdout-v11-entry1-seed503/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed503/signedreview.json) |
| 509 | 190.24 | 191.12 → 194.12 | 23.845mm / 8 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-holdout-v11-entry1-seed509/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed509/signedreview.json) |
| 521 | 없음 | 없음 | — | ID0·search timeout 실패 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-holdout-v11-entry1-seed521/offline-evaluation.json) · [실패 감사](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed521/failure-denominator-audit.json) |
| 523 | 279.76 | 280.62 → 283.62 | 29.721mm / 8 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-holdout-v11-entry1-seed523/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed523/signedreview.json) |
| 541 | 없음 | 없음 | — | ID0·search timeout 실패 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-holdout-v11-entry1-seed541/offline-evaluation.json) · [실패 감사](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed541/failure-denominator-audit.json) |

[동결5개 전체 분모·identity](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/full-five-denominator-and-frozen-identity.json) · [개별 서명 색인](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/signed-review-index.json)

## 원 실행과 검토 범위

사전plan과각 run의 seed/entry/split/series·trialmetadata·frozen 설정·source-snapshot를 대조했다. 모든 실행은 training과 같은59source/test/C+10runtime/buildmanifest byte hash와 설정이다. plan의62개 core 기록도 확인했다. unused 선정은 training 이전 후보 목록/holdout 사전plan과split을 근거로 하며, 이 seed집합이 실제 환경 모집단이나 모든 진입을 대표한다고 추론하지 않는다. 결과를 후속 튜닝에 사용하면 이제 spent data다.

| 근거 | SHA256 |
|---|---|
| holdout plan | `c355e83fe097bfb035443b6f0b601c99254862b811b7b29f979716ac0865e8c7` |
| 보존 controller | `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180` |
| 보존 physical_estimator | `d8eab7c2e126307b3970f54f3db9cd0a1835c3fd8f5c37b6ff3747b900ee054c` |
| frozen 설정 | `58632e6b7291f27d5dd9d61bddcff199fe7567167478817330c126ea00e5b603` |
| evaluator | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |

현행 v12의 `onboard_tof_min_raw_m=.04` 변경은 이번v11 원snapshot에 없다. v11 onboard fullEKF는new range에 `0<=distance<5`를 적용하며PC의range_accurate와별개로 근접ToF를 융합하는 우려가 남는다. 현행 source를 과거 run으로 대체하지 않았다. [source62·delta 근거](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/source-delta-and-core62-crosscheck.json).

원evaluator로각5평가를 전필드 동일하게 재현했다. 성공세개는 sensor/estimate-only controller 단계, rawIMU·heldPWM 기반 freeflight gain/잔차·contact candidate 시각,100Hz RPYT 14byte 부호/정수thrust·센서 cadence/newflag를 별도 재생했다. 숨은native PID/EKF 전 내부 상태나 모든 compiler transitive header를 재현한 검토는 아니다. 변하지 않은 전체펌웨어·truth 경계는 같은 SHA인 이전독립 검토를 유지한다. known initialheading 기반ENTRY_SEARCH yaw가기존 reverse-speed schedule에도영향을주므로 pure광학시선 효과로만 분해하지 않는다.

## 세 실제 접촉과 명령 한계

| seed | 네발 접촉 표본 | HOLD 네발 표본 / PWM0 표본 | HOLD 발당 최소법선력(N) | 실제 최대3D속력(m/s) | off잔류추력(N) → .3s후(N) |
|---|---:|---:|---:|---:|---|
| 503 | 199 | 151 / 3001 | 0.032174 | 0.411033 | 0.201017 → 6.149e-08 |
| 509 | 196 | 151 / 3001 | 0.031394 | 0.406344 | 0.202232 → 6.186e-08 |
| 523 | 196 | 151 / 3001 | 0.031755 | 0.411917 | 0.202678 → 6.200e-08 |

세 HOLD 전체의 네 발은 pad에 닿아 법선력이 양수였고, 비발·차체·marker board·그물·바닥 접촉0,접촉/HOLD의실제turn0이었다. 차량은전체로그에서처음부터끝까지 .25m/s 연속이며3초간패드.75m이동했다. 각3,001개1kHz모터 표본의PWM/출력/staticforce0과151개50Hz실제지지를 확인했다. settle된총법선력은약.34335N으로 S35g과일치한다. 최대3D 상대속력7.27~7.35e-5m/s,최대HOLD XY슬립7.53~14.42µm는조건부마찰/접촉/lag모델결과이며실기접촉정밀도가아니다.

명령속력.4/정상3D가속도.15/yaw10/20/상승.10/하강.06은세개모두통과했다. off 긴급속도명령 변화13.395/12.870/12.783m/s²는 별도 예외다. 실제 최대3D속력 .406~.412는명령.4를넘었다. main요약의horizontal speed와3D speed도다른양이다(523 horizontal.408673 vs3D.411917). **명령cap을 실제강체운동cap으로해석하면안된다.** PWM0때실제추력이약.20N 남고20ms S응답으로감쇠하므로즉시기계정지/RPM0증명도아니다.

| seed | off 추정 pad위 body높이(mm) | off영상age(s) | off보정age(s) | 최종world오차(mm) / relativeXY추정오차(mm) |
|---|---:|---:|---:|---:|
| 503 | 22.415 | 5.12 | 20.13 | 213.946 / 69.180 |
| 509 | 13.096 | 5.18 | 20.09 | 219.963 / 80.002 |
| 523 | 9.189 | 4.94 | 20.24 | 164.553 / 78.689 |

실제 off body높이는약13.565mm다.503은추정22.415mm로S높이gate상한26.5mm까지4.085mm만여유있다. true높이·firstcontact CoM오차·worldgauge오차·relative추정오차를구분한다. 두위치가공유하는translation gauge는relative차에서취소될수있으므로작은착지오차가절대오도메트리정확도를증명하지않는다.

Flow100/ToF40의모든stamp/newflag/raw packet 대조오류0이며heldToF를새관측으로재융합하지않았다. 세 HOLD의100Hz301표본은정확ToF0/유효Flow0/usableFlow0·surfaceambiguous다.50Hz150개IMUobserver안정표본과151개truth지지표본을구분하며HOLD뒤영상age8.12/8.18/7.94s는비행prediction예산연장이아니다. IMU의near-g/gyro/tilt안정은pad정체성/네발지지를센서만으로확인하지못한다.

523은239.48s coarse→244.44s metric획득→259.38s직선LOWER_ALIGN였다. LINEUP의곡선358표본에는LOWER_ALIGN전환이없었다. 준비13.90/13.76/14.16s는계획12s를넘었으므로12초하드실행상한이라고부르면안된다. 실제세접촉과HOLD는직선에서완료했다.

## 523 replay의 실행 순서 차이

첫raw보정replay는로그된센서snapshot뒤실행되는`visual_anchor`를빠뜨려마지막보정시각10ms,예측가속도최대1.852e-4m/s²차이를보였다.260.38s센서기록z .6505137m은영상보정뒤내부.6492746m로바뀌며.65mfreeflight문턱아래가된다. archived estimator의accepted raw relative pose/높이회귀를실행순서대로추가한v2는gain·lastvalidstamp·specificforce·잔차·expectedaccel전부오차0이다. 이수정은검토helper에만적용했고첫실패JSON/log를보존했다. 원run·원평가·근거문턱에는변경없음. [523 보완된 정확 replay](../../../artifacts/mission3/reviews/20261009-v11-holdout-independent/seed523/full-support-calibration-replay-v2.json).

원 `development-review-unsigned.json`의공통접촉/HOLD/오차필드를독립재계산한뒤대조했으며세개불일치0이었다. 그파일의승인문구를본인판정으로전환한것은아니다.

## 두 실패와 영상·실기 한계

521·541은각각총309.32/308.96s,1mhover인계뒤300.02s에`search_timeout_without_metric_pose` ABORT다. 모든전달관측은marker_missing,decoded/metric/실제접촉/off0이다. 원300s검색기준과18pxpose/불확실성1m를유지했다. 두실패를표본분모에서제거하거나나중성공으로바꾸지않는다. 관측0이라실패원인을하나의시선/PSF/거리요인으로확정할수없다.

FFV1 원프레임5033304/5093487/5216001/5235258/5416001,전체24,051장을전부decode해controller-input SHA와byte동일·mono320uint8임을확인했다. 자체native그림을열어확인했다. AVI20FPS는결손렌더를제외한보관순서시계이며실제capturestamp를써야한다. S.05s전송은50Hz전달경계에서.06s로그다. 원픽셀보관일치는AIdeck전송·exposure·PSF·품질검증이아니다.

유지되는critical위험:idealray기반quality/nearest5×5 ToF·genericFlowtexture는실제component/deck응답이아니다. range_accurate=true인데raw<4cm가503/509/523에8/6/4센서표본있으며onboardfullEKF도근접metricgate를공유하지않는다. 기체·camera/Flow mount·K/D·CoM·mass·dynamics·PSF·gyro잔류·평균화/capture/RTOS/radio timing·접촉재료가S/U다. plant와observer의공유추력·질량/lag가정은groundeffect/동력변화에서거짓support를배제하지못한다. 고정4.2V는SOC/전류/배터리잔여·payload/M1M2소비를포함하지않는다.1mhover인계는actualM2경로인계가아니다.

다음검증은센서실제입력계약·전경로rangegate/중복시각·모델반례를분리하고새source/config/seed를선언해야한다. v11 signature는v12근접gate효과나실기성능/안전성/진입확대를사전승인하지않는다. 기존상세source경계는 [v9 검토](20261009-v9-contact-review.md)와 [별도v11 training 검토](20261009-v11-training-contact-review.md)를참고한다.

원자료·원평가·기존서명·공용source/config/libs/controller/test/work-log를수정하지않았다. 신규검토doc/지정독립folder에만결과를보존했다. 실패와split분모를유지하고최종서명해시/원입력불변검사후소유권을반환한다.
