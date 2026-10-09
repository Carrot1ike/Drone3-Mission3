# v16 causal IMU: 새 source 회귀 3회 독립 검토

검토자 **Heisenberg / Codex-Independent-Flight-Reviewer**, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. v15 전18회의 final-verification 이후 별도로 v16 원 run의 source-snapshot과 새 input/evaluation 결합을 검토했다. 새 에이전트·새 물리 시험·공용 수정·이전 서명 복제는 없다.

**spent training 3회:571/619 조건부 접촉 승인2,613 실패 거절1.** 유효한 원 sensor history에서 v14와 byte-identical인 source 회귀이며 새 착륙 능력·unused holdout·일반 성공률 근거가 아니다. 원장118→121은 원 fullphysics3회 추가의 장부 참조다. qualified/formal/제조사 일치/하드웨어/진입2 승인 모두0. 이전 v14 성공·실패와 v15 서로 다른 경로 분모를 합치지 않는다.

[원 사전계획](../../../artifacts/mission3/acquisition/20261009-causal-imu-regression-training-v16/plan.json) SHA256 `27d4270a8d2ce9988b037abd4d167761a53260c1bc4cab4534d2fd93d0f72887`; [새 개별 판정 인덱스](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/signed-review-index.json); [최종 binding](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/final-verification.json).

| seed | 원 baseline 및 새 run | metric receipt / accepted | first contact / PWMoff / 3s hold끝 | 원 평가·본인 판정 |
|---|---|---:|---|---|
| 571 | [v14 baseline](../../../artifacts/mission3/physical/20261009-imu12-v14-entry1-seed571) · [v16 run](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed571) | 72.50 / 584 | 106.26 / 107.16 / 110.16s | [조건부 승인 9점/14.585mm](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/seed571/signedreview.json) |
| 619 | [v14 baseline](../../../artifacts/mission3/physical/20261009-imu12-holdout-v14-entry1-seed619) · [v16 run](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed619) | 208.40 / 621 | 243.76 / 244.64 / 247.64s | [조건부 승인 10점/9.375mm](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/seed619/signedreview.json) |
| 613 | [v14 baseline](../../../artifacts/mission3/physical/20261009-imu12-holdout-v14-entry1-seed613) · [v16 run](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed613) | 없음 / 0 | contact0 / off없음 / hold없음 | [거절 search_timeout_without_metric_pose](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/seed613/failure-review.json) |

18회 v15와 다르게 v16은 기준 v14 imu12 원 config `8m/.75m`를 사용한다. 같은 seed의 frozen config와 runtime10개가 baseline과 동일하고, 모든 source62개를 원 manifest·사전계획65개(source62+config3)와 확인했다. 달라진 source는 `vertical_fit.py`에 `or it[-1] > now` 한 줄, `test_vertical_fit.py`의 경계 regression1개뿐이다. controller/scene/physical_estimator/camera/native320/pose18px/WAIT15/scan15/height1/gain.12/.08/6s prediction/.25s support/caps가 모두 보존됐다. source-snapshot과 원 frozen settings를 사용했고 현행 코드를 과거 run으로 대체하지 않았다. [원 source/config/trial/evaluation 재확인](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/source-series-verification.json).

| source | 이전 SHA256 | v16 원 SHA256 |
|---|---|---|
| vertical_fit.py | `35f6587922b0aaeded60d46d3ee1949a6ae7a9df677853ef3172d888e9d8f5e0` | `afba09e5003b5f77d13916854cd54f015f73e76345f74408e2646bb59c1f2153` |
| test_vertical_fit.py | `f742f37f4b6c502a95af968f8a496feb4dd7bf0e1a64cb7492d2841a81717155` | `2b20411de71eacce4bbcd675760aac666b29c08bf212f9ae38eee07f2c0fb42b` |

원 evaluator SHA `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0`는 유지됐다. 새 원 offline-evaluation 3개를 **각 run의 evaluator snapshot**으로 재계산해 전 필드 일치를 확인하고, 각 full evaluation-file SHA와 canonical digest를 별도로 저장했다. 기존 v14 서명은 새 source/evaluation 승인으로 복제하지 않았다. 204tests 원 log SHA `9d8595ffe85e3a7143d2215d804b5ce44a4911cf3fba1e30d077cc2c2e950f3a`가 사전계획과 맞았다. 본인은 별도 새 함수 반례/해석 fixture와 raw 산술·물리 감사를 실행했으며204개 원 suite를 재실행했다고 주장하지 않는다.

main comparison은 참고로 읽은 뒤 본인이 3×10개 파일 SHA를 다시 계산했다: `commands/truth/observations/motors/sensors/deck_samples/estimates/rpyt_packets/camera_frames.jsonl` 및 `camera.avi`가 각각 해당 v14 seed와 **30/30 byte exact**였다. 새 원 metadata/source/evaluation은 별도 binding한다. [본인 전체 SHA·source diff](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/independent-same-input-source-regression.json). 기록이 같다는 사실은 causal guard가 이번 정상 chronology에서 closed-loop 행동을 바꾸지 않았다는 source 회귀 근거이며, 새 성공 독립 표본이나 안전/착륙 성능 향상으로 세지 않는다.

3개 native FFV1을 모두 다시 decode하여 **12,180 frames**의 mono320 controller-input bytes/SHA가 일치했다(571=1645,619=4534,613=6001). archive exact는 AI-deck capture/송신 성능이나 렌즈 PSF 실측이 아니다. [619 원 픽셀 표본](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/seed619/native-camera-samples.png)을 눈으로 확인했고 전체 영상 재생 QA를 했다는 주장은 없다.

실제 접촉 승인2개를 새 원 logs에서 다시 검사했다. 각각 contact197/196표본이고 pad foot1–4만 접촉하며 chassis/marker/floor 충돌은 없다. PWMoff부터3초 동안151개50Hz truth 표본 모두 양의 네 발 법선 지지,3001개1kHz PWM/output/static-force0, 모든 contact/hold 직선, 전체 차량 속도.25m/s 연속을 확인했다. 최소 발법선은571 .033122N /619 .033200N, hold 최대 slip7.093μm /4.567μm다. RPYT100Hz·14byte/yaw sign, Flow100/ToF40 stamp/new/raw 전달, raw command-model support 및 전체 freeflight calibration 재생, sensor-only FSM, normalcaps가 모두 일치했고 자체 unsigned development 주요 필드와 불일치는 없었다. 이는 로그/source 재생 범위이며 전체 숨은 PID/EKF 상태나 실제 radio timing 재현이 아니다.

571/619의 실제 최대3D속도 .408150/.405305m/s는 명령 cap.4와 다르다. PWMoff 첫 step 실제 힘은 .201703/.202167N으로 남았다가 .3초 후6.170e−8/6.184e−8N으로 줄었다. 가정된20ms motor lag를 기록에서 재구성했고 실제 rotor 즉시 정지라고 쓰지 않는다. 준비시간14.88/13.50s는 계획12s를 초과한다. hold301개100Hz sensor 표본에서 정확한 ToF/valid Flow 모두0이며 ambiguous였다. IMU/model stable은 네 발/표면 identity의 실기 관측이 아니므로 pad 판정은 오프라인 물리 truth에서만 확인했다.

613은21 decoded/metric0, contact0/off없음, 원 handoff9.58s 뒤 search300s 경계309.60s에 timeout했다. std.638269m라 이번 실패를 std 초과로 바꾸지 않는다. coarse decoded bearing은 metric range/상대 pose가 아니며 원 strict gate를 낮추지 않았다. 마지막 ABORT 명령 뒤 wrapper가 wire/physics 전달 전에 종료되어 마지막 motor PWM은 nonzero/stop packet0이다. 안전한 실기 회복·정지를 확인한 실패가 아니다. [613 원실패 독립 감사](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/seed613/failure-audit.json).

**API 수정의 실제 범위:** 본인이 원35f658과 새afba09 snapshot을 직접 호출했다. now.055/range끝.05/IMU끝.06에서 미래 acceleration0과1 모두 새 함수에서 ValueError였다. 이전 함수는 각각−.020/−.015m/s를 반환했다. current IMU끝=now=.06은−.020m/s를 유지했고 `np.nextafter(.06,+inf)` 및1ms 미래는 거부했다. origin0/123s×acceleration−.04/0/.06의6개 해석 fixture 오차 최대2.92e−16m/s였다. [본인 before/after·현재 경계 반례](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/scalar-fit-independent-fixtures.json).

실제3회는 append-current IMU path이며 미래/중복 packet0, fallback0, 기록된 surface/image acceptance를 고정한 Z/vZ 산술 재생 오차0이었다. producer ToF40Hz와100Hz consumer 사이 range age0/5/10/15/20ms는 여전히 존재한다. body-IMU 누적 평균·현재 PC attitude·producer 시각 tilt 정렬은 합성 packet 가정이다. 실제 clock·capture/sample-average·전송지연은 U이며, API current coverage 요구와 외부 sample delay의 적합성을 검증하지 않았다. [3개 chronology/산술 재생](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/vertical-fit-all-cases.json).

**전체 융합의 future gate가 아니다.** 원 snapshot SelfMotion에 now.3/range끝.4/IMU끝.4를 제공하자 scalar fitter는 거부했지만 SelfMotion은 ValueError를 catch하고 legacy 선형 fallback−.030m/s를 반환했다(fallback1). raw height 보정은 fit 이전의 별도 경로다. 그러므로 새 `it[-1] > now`는 scalar function의 미래 IMU 의존 결함을 고친 것이며, invalid chronology를 observer 전체에서 거절하거나 외부 telemetry를 안전하게 만들었다는 보장은 아니다. 향후 입력 유효성 단계에서 future/nonfinite/out-of-order/clock mismatch를 분리 거절하고, 정확한 height 기록에 IMU가 부족한 경우만 명시적 fallback으로 허용하는 계약을 검증할 것을 권고한다. 공용 구현을 수정하지 않았고 이 API fixture를 실제 flight failure로 세지 않는다.

원 근접 ToF default0은 유지되어 full onboard EKF에4cm 아래 raw ToF가 들어간다. 독립 PC IMU context는 deck 업데이트를 받지 않지만 실제 자세 PID의 onboard EKF 경로와 합성 mount/mass/thrust는 남는다. 본인 near-range 기록, RPYT/command 모델 및 AST source 검사로 aircraft qpos/qvel 초기화 뒤 처방 쓰기가 없음을 다시 확인했다. truth는 controller/support live branch에 공급되지 않고, 합성 sensor 생산과 오프라인 접촉 평가·floor/tilt harness stop에 쓰인다. [원 source boundary·근접 range](../../../artifacts/mission3/reviews/20261009-v16-causal-imu-regression-independent/source-boundary-and-near-range.json).

range/Flow quality는 실제 RangeStatus/SQUAL 대신 합성 모델이며, body/camera/Flow mount,K/D,CoM/inertia,lag,contact/friction,RTOS/radio timing은 S/U다. 공급 전압4.2V 고정에 SOC/energy/mission1·2 reserve가 없고,613의 >5분 비행 실패는 장착 실기 endurance 근거가 아니다. Scalar accel displacement fit은 Forster manifold VIO나 bias 식별기가 아니다. 새 guard가 근접 정확도/동역학/배터리/하드웨어 미확정을 해결했다는 주장은 승인하지 않는다.

본인 초기 helper의 review_folder 누락과 fixture의 내부 deque 이름 오기는 원 snapshot을 읽어 수정했고 최초 실패 로그와 새 재개/v2 로그를 보존했다. 원 source/evaluation/physics를 바꾸지 않았다. 원자료320개·기존 서명45개를 마지막 SHA로 확인하고, 새2positive 서명 및1negative audit를 각각 본인 agentID·실행source·runtime·frozen config·trial·full evaluation SHA·실제 audit evidence에 묶어 final-verification/index를 저장한다. 전체3분모와 source 회귀 범위는 그대로 유지한다.

수정과 통합 경로의 직접 원본: [새 fitter snapshot](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed571/source-snapshot/scripts/mission3/vertical_fit.py:19), [변하지 않은 fallback 호출부](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed571/source-snapshot/scripts/mission3/estimator.py:261), [raw height 보정 경로](../../../artifacts/mission3/physical/20261009-causal-imu-v16-entry1-seed571/source-snapshot/scripts/mission3/estimator.py:204).
