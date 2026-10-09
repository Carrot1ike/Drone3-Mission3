# v13 scan15/scan30 spent-training 독립 접촉 검토

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. training 각6건을 별도로 검토한 결과 scan15는433·457·467·479·541의5건, scan30은433·457·467·479의4건을 조건부 모델 접촉으로 승인한다. scan15/521와 scan30/521·541의 검색 실패3건을 분모에 보존한다. 제조사·실기·qualified·formal 승인은0, 진입2 승인0이다. 재사용 training이며 일반 성공률·새 holdout·v11/v12 성공과 혼합하지 않는다.

원 [계획](../../../artifacts/mission3/acquisition/20261009-entry-scan-duration-training-v13/plan.json), 각 실행의 source-snapshot·frozen settings·trial metadata·모든 원로그/평가를 직접 확인했다. [본인 결과](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/examined-result-summary.json)와 seed별 원파일 SHA 및 새 감사에 판정을 결합했다. 평가12건 모두 실행 당시 evaluator로 재계산해 원평가 모든 필드와 일치했다. 원 run_dir 상대 경로와 파일SHA/canonical digest를 구분했다.

| 정책 / seed | 첫 metric 수신(s) | 실제 접촉(s) | PWM off / 실행 끝(s) | 내부 점수 / XY(mm) | 판정 |
|---|---:|---:|---:|---:|---|
| scan15 / 433 | 157.04 | 184.64 | 185.48 / 188.50 | 10 / 5.262 | 조건부 접촉 승인 |
| scan15 / 457 | 154.82 | 184.76 | 185.62 / 188.64 | 10 / 3.746 | 조건부 접촉 승인 |
| scan15 / 467 | 123.16 | 152.18 | 153.08 / 156.10 | 9 / 14.530 | 조건부 접촉 승인 |
| scan15 / 479 | 180.96 | 216.50 | 217.36 / 220.38 | 9 / 12.038 | 조건부 접촉 승인 |
| scan15 / 521 | 없음 | 없음 | 없음 / 309.32 | 없음 | 검색 실패 / 승인 없음 |
| scan15 / 541 | 72.40 | 102.52 | 103.46 / 106.48 | 8 / 20.783 | 조건부 접촉 승인 |
| scan30 / 433 | 159.68 | 187.16 | 188.08 / 191.10 | 9 / 17.602 | 조건부 접촉 승인 |
| scan30 / 457 | 157.32 | 186.06 | 187.02 / 190.04 | 8 / 23.441 | 조건부 접촉 승인 |
| scan30 / 467 | 125.86 | 155.66 | 156.64 / 159.66 | 9 / 15.649 | 조건부 접촉 승인 |
| scan30 / 479 | 183.36 | 215.86 | 216.80 / 219.82 | 8 / 20.159 | 조건부 접촉 승인 |
| scan30 / 521 | 없음 | 없음 | 없음 / 309.32 | 없음 | 검색 실패 / 승인 없음 |
| scan30 / 541 | 없음 | 없음 | 없음 / 308.96 | 없음 | 검색 실패 / 승인 없음 |

code60/runtime10은12건 모두 동일하고 계획 core63과 대조됐다. frozen config는 scan15 `6f7e1dd85d6c9d79d0484db30d2095dab847c267b47b82ea110b4d2f8c4761c9`, scan30 `5d35432d01cc159aa54cefb695761ae2147d1f128500c6455e8ca891302b7704`로 정책별 고정이다. 추가 config 원파일은 스냅샷 밖에서 계획 SHA와 맞는 bytes만 확인했다. 실제 실행 설정은 원 frozen settings와 public controller-settings를 분리해 검사했다. public 설정에는 물리 profile의 controller/observer/optics 소프트웨어 override가 적용된다. raw mission JSON을 실제 public 설정과 같다고 가정하지 않았다.

v11 대비 활성 public 설정 차이는 `search_pretranslate_scan_s=45→15/30` 하나다. WAIT15·진입 높이1·entry-forward gaze·원 pose/std/contact/명령 한도·장면·차량 phase·물리/센서 profile은 유지됐다. 현행 스냅샷에는 v12 optional range gate와 두 검사가 있어 원 v11 source bytes와는 다르다. `physical_estimator.py` SHA `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5`, controller SHA `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180`, evaluator SHA `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0`. profile에서 raw range 하한은 기본0이다. v12의4cm 시험이 적용된 실행으로 해석하지 않는다. [전체 source/config/phase 검사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/source-series-verification.json).

초기 scan 관측 길이는 제어tick 이산화 때문에 각15.02/30.02s였다. WAIT15 뒤 ENTRY_SCAN, 이후 ENTRY_SEARCH 순서를 원 명령과 sensor-only FSM 재생으로 확인했다. WAIT/SCAN의 `.6*(진입 목표-추정 위치)` 보정은 작은 비영 속도를 낸다. “이동 전 완전한 속도0”나 실제 기체 위치 불변을 주장하지 않는다. 의도한 raster 이동은 ENTRY_SEARCH 이후다. 진입 heading은 알려진 초기 방향이며 차량 live heading을 시선 목표로 주입하지 않는다.

승인9건은 각각 hold 참값151/151 모두 네 발 pad 접촉과 양의 법선력, 1kHz PWM/output/static-force3001개가0, off 이후3초를 확인했다. 모든 접촉/hold는 직선이고 차량 속력은 전체 .25m/s다. 다른 표면 또는 비발 접촉은 없다. 발당 hold 최소 법선력의 전체 최솟값은 0.030261N, 최대 hold 미끄럼은 11.368µm이다. actual motor force는 off 첫step에도 약.20N 남아20ms 가정 지연으로 감쇠한다. PWM0를 즉시 rotor/force0로 해석하지 않는다.

명령12건은 정상 .4m/s·.15m/s²·yaw10°/s·20°/s²·상승.10/하강.06m/s를 통과했다. 긴급 off/recovery/ABORT 예외는 별도로 보존했다. 승인9건 실제 최대 속력은 0.402217~0.412683m/s다. .4는 실제 운동의 하드 cap이 아니다. native FFV1 총46,801장을 직접 mono320 bytes/controller-input SHA와 대조해 모두 일치했다. capture/delivery stamp, Flow100·ToF40/new flag와 RPYT100Hz 14byte payload·부호·stop packet을 확인했다. 이 기록은 실제 radio/HAL/RTOS·AI-deck 전송·광학 PSF 검증이 아니다. 양성 개발 unsigned review와 대조한 hold/센서/world오차 필드는 불일치0이지만 독립 승인은 직접 재계산으로 결정했다.

raw IMU/PWM에서 전체 support gain·specific-force·residual 재생은9건 모두 scalar오차0과 calibration stamp 일치였다. sensor-only 후보 재생은 실제 off와 맞고 calibration은 모두 fresh다. sensor-log 뒤 visual_anchor가 내부 높이를 갱신하는 실행 순서를 다음 calibration 판정에 반영했다. 이는 같은 조건부 모델의 재현이며 실제 질량·motor gain/lag·ground effect·장착 IMU 정확도 보장이 아니다. .0145±.012m 높이 gate와 near-g/probe 조건도S이다. HOLD의 정확 ToF/유효 Flow는 모두0이고 ambiguous 추론을 유지한다. IMU 안정성은 네 발·pad 신원을 확인하지 못한다.

scan15/541은 첫 metric capture72.34/receipt72.40s, 실제 접촉102.52/off103.46/hold끝106.46s다. scan30/541과 v11/541의ID0 실패를 같은 조건의 성공으로 바꾸지 않는다. scan30/521는4개 decoded corner record가 있지만 metric0이고 코드 짧은 변5.33~8.25px였다. scan15/521는ID0/metric0이다. 세 실패는300s 미션 검색 budget 종료이며 pad/off 없음이다. 낮은 decoder/pose gate나 searchdeadline 연장을 권고하지 않는다. [실패 원로그 감사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/scan30-seed521/failure-audit.json).

457은 두 scan에서 새 trajectory로 접촉을 완료했지만 v11의 같은 마지막 visual fix/IMU 전파 오류를 수정한 실험이 아니다. off 추정 높이는 scan15 7.034mm, scan30 23.922mm로 원 v11/457에서 height gate가191.16s에26.5mm를 벗어난 상황과 다르다. trajectory/최종 영상 기준/센서 입력이 함께 바뀌므로 단일 원인 치료로 승인하지 않는다. scan30/467 off 추정 높이24.871mm는 상한26.5mm에 약1.63mm 여유밖에 없다. noisy propagated height/support 동시성의 민감도는 남는다.

착륙 준비 실측은13.72~17.02s로 계획 가정12s를 넘었다. 실제 접촉/hold는 전부 직선이며 이후 단계의 기존 지도·straight/prediction 검사는 유지됐다. 이 결과를12s가 보장된다고 쓰면 안 된다. 17.02s 사례의 추가5.02s는 환경 차속.25에서 약1.26m 이동에 해당하는 오프라인 비교다. 다음 설계 검토는 준비시간을 상수 보장으로 취급하지 않고 관측 속도/실제 단계 경과시간과 남은 straight margin의 일관성을 시험하는 것이 타당하다. 이미 있는 후속 margin 검사를 제거하거나 terminal 예산을 늘리라는 제안이 아니다.

range 하한0에서는4cm 미만 새 ToF도 onboard에 들어간다. [근거리/기준계 감사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/scan15-seed541/near-range-and-state-frame-audit.json)는 full onboard Z, mission PC Z, 실제 body Z를 오프라인으로 구분했다. 공식 `mm_tof`의 Z-only H 시작은 crosscovariance를 통한 다른 state/attitude 영향을 배제하지 않는다. PC의 별도 IMU context는 상위 중력보정에 deck를 직접 넣지 않지만 하위 RPYT PID는 full onboard 자세를 사용한다. 실제 plant→다음IMU 경로까지 격리됐다고 주장하지 않는다. v12 분석에서 확인한 raw threshold/RangeStatus/표면 기준계 문제는 유지된다.

qpos/qvel aircraft는 초기화 뒤 강제 수정하지 않고 차량만 처방 연속 운동을 한다. contacttruth는 오프라인 평가와 명시된 floor/tilt60 harness 중단에만 사용하며 support/controller 입력에 넣지 않았다. plant state를 이용한 합성 센서는 S/U 모델이다. K/D/mount/질량·CoM·관성/마찰/힘·공통 support 모델/IMU 잔류bias/PSF/texture/timing과 battery4.2V 고정·실제 M2 인계/RTOS gap은 계속 미확정이다. 각 실행 최종 world 위치 오차와 pad-relative XY 오차는 서로 다른 양이며 공통 위치 gauge가 상대 오차에서 상쇄될 수 있다. 이를 실제 위치 정확도 주장으로 바꾸지 않는다.

scan15 선택은 spent-training5/6 대 scan30 4/6의 개발 판단이며 이 작은 짝 비교로 일반 성공 확률을 추정하지 않는다. 새unused557/563/569/571/577는 별도 holdout 문서·분모·서명으로만 검토한다.541 전체1× replay 영상도 같은 training raw를 표시한 파생물이고 새 시행이 아니다. 원 자료·평가·source/config/work-log·과거 서명은 수정하지 않았다. 신규 [9개 개별 서명/3개 실패 색인](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/signed-review-index.json)과 [최종 불변 검사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-training-independent/final-verification.json)를 제공한다.
