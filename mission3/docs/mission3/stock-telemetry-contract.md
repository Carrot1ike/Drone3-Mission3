# 기본 로그 telemetry의 단위·축·시각 계약

Heisenberg, 2026-10-08. [standalone parser](../../scripts/mission3_stock_telemetry.py)와 [top-level 시험](../../scripts/test_mission3_stock_telemetry.py)은 CPU에서 기본 CFLib logger callback 또는 변수 payload를 검증·정규화한다. **37/37 시험 통과이며 무선·기체·미션 runtime 연결은 실행하지 않았다.** 실제 로그를 현재 `PhysicalMotion`에 바로 넣을 수 있다는 주장은 금지한다. 공유 미션 소스·평가기·원 실행 로그를 수정하지 않았다.

선행 [PC telemetry 간극 문서](pc-telemetry-gap.md)의 측정시각·평균 IMU·Flow dt·ToF identity 간극을 유지한다. 부품 FoV·센서 정확도·장착을 추정하여 채우는 모듈도 아니다.

## 고정 제조사 원문과 identity

공식 firmware Git commit은 `f6e0f0a3b526861328caf3296a7ecf00bb915d39`, 공식 CFLib Git commit은 `3c1cb0d024fef6f45d08f0ad236a22ac395dd924`다. 사용한 원문15개는 각 파일을 해당 commit의 `git show` blob과 byte 비교해 보존했다. [원문·전체 SHA·snapshot index](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-evidence/source-evidence.json).

기존 [RPYT 원문 manifest](../../artifacts/mission3/rpyt-protocol/20261008-054746-273897-tests/source-manifest.json)도 같은 두 Git commit이다. `33a000502846668583447859d5df03dc800b8edfea59e35ea2f4e02cd6ec345b`는 `commander.py` **파일 SHA-256**이며 Git commit이 아니다. 현재 그 파일도 동일하다. 두 검증은 실행하는 vendor 파일 집합이 다르지만 Git revision은 같다. 이 대조는 새 evidence에 보존했고 공유 RPYT·PC 간극 문서는 수정하지 않았다.

| 원문과 실제 근거 | 계약 |
|---|---|
| [stabilizer.c:604](../../src/bitcraze-mission3-firmware/src/modules/src/stabilizer.c:604), [:617](../../src/bitcraze-mission3-firmware/src/modules/src/stabilizer.c:617), [:677](../../src/bitcraze-mission3-firmware/src/modules/src/stabilizer.c:677) | `acc.x/y/z`는 body-frame sensorData float g, startup gravity 정렬 처리. `gyro.x/y/z`는 필터 뒤 float deg/s. `stateEstimate`나 raw ADC가 아니다. |
| [physicalConstants.h:29](../../src/bitcraze-mission3-firmware/src/utils/interface/physicalConstants.h:29) | firmware의 nominal gravity 9.81을 단위 환산에 사용. 지역 중력 실측이나 IMU 재보정 주장이 아니다. |
| [BMI088 driver:326](../../src/bitcraze-mission3-firmware/src/hal/src/sensors_bmi088_bmp3xx.c:326) | gyro startup bias 차감→airframe 정렬→LPF, accel scale/gravity 정렬→LPF. driver 구현 근거이며 실제 배정 기체의 센서·잔여 편향을 확정하지 않는다. 가상 0.03deg/s 잔여 편향은 S다. |
| [motors.c:1061](../../src/bitcraze-mission3-firmware/src/drivers/src/motors.c:1061) | `motor.m1..m4`는 uint16 commanded PWM ratio. `m*req`, rotor 힘·RPM과 구분. |
| [flowdeck_v1v2.c:94](../../src/bitcraze-mission3-firmware/src/deck/drivers/src/flowdeck_v1v2.c:94), [:262](../../src/bitcraze-mission3-firmware/src/deck/drivers/src/flowdeck_v1v2.c:262) | native int16 deltaX/Y를 기체측 `(-deltaY,-deltaX)`로 변환. motion/squal uint8, shutter uint16. 원시 metadata를 유지하고 SQUAL 유효성 gate를 만들지 않는다. |
| [flowdeck_v1v2.c:121](../../src/bitcraze-mission3-firmware/src/deck/drivers/src/flowdeck_v1v2.c:121) | 공식 task의 `flowData.dt`는 usec clock 차이이며 이 두 log block에 없다. raw count를 metric/angular 속도로 바꾸지 않는다. |
| [range.c:40](../../src/bitcraze-mission3-firmware/src/modules/src/range.c:40), [:91](../../src/bitcraze-mission3-firmware/src/modules/src/range.c:91) | rangeSet metre×1000 저장, `range.zrange` uint16mm. 로그 시각이 ToF capture 시각이라고 하지 않는다. |
| [pm_stm32f4.c:511](../../src/bitcraze-mission3-firmware/src/hal/src/pm_stm32f4.c:511) | `pm.vbat` floatV. 비행 시험의 고정4.2V 가정을 주입하지 않는다. |
| [log.c:568](../../src/bitcraze-mission3-firmware/src/modules/src/log.c:568), [CFLib log.py:623](../../src/crazyflie-lib-python/cflib/crazyflie/log.py:623) | block 획득 시점의 low24bitms logger stamp. 변수가 차례로 읽히는 snapshot이며 센서별 capture 시각이나 한 센서 epoch를 보장하지 않는다. |

현재 checkout에 대한 링크는 향후 바뀔 수 있다. 역사 검토에는 [고정 source snapshot](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-evidence/source-snapshot)과 파일 SHA를 사용한다. 공식 upstream 원문은 [firmware 고정 commit](https://github.com/bitcraze/crazyflie-firmware/tree/f6e0f0a3b526861328caf3296a7ecf00bb915d39), [CFLib 고정 log.py](https://github.com/bitcraze/crazyflie-lib-python/blob/3c1cb0d024fef6f45d08f0ad236a22ac395dd924/cflib/crazyflie/log.py)다.

## 두 block의 payload와 단위

`make_log_configs(LogConfig)`는 공식 class를 주입받아 아래 두 config를 만든다. 모든 field의 native fetch type을 명시하고 nominal10ms를 요청한다. 함수는 연결하거나 logging을 시작하지 않는다. 실제 접속 시 device TOC의 필드 존재·저장 type을 별도로 대조해야 한다.

| block | field 순서 / native type | variable payload |
|---|---|---:|
| `imu` | acc.x/y/z float, gyro.x/y/z float | 24B |
| `deck_power` | motor.m1..m4 uint16, motion.deltaX/Y int16, motion.motion uint8, motion.squal uint8, motion.shutter uint16, range.zrange uint16, pm.vbat float | 22B |

공식 [LogConfig.MAX_LEN](../../src/crazyflie-lib-python/cflib/crazyflie/log.py:136)=26B, [firmware LOG_MAX_LEN](../../src/bitcraze-mission3-firmware/src/modules/src/log.c:88)=26B, [CRTP data 한도](../../src/bitcraze-mission3-firmware/src/modules/interface/crtp.h:35)=30B다. ID1B+stamp3B를 포함한 data는28B·26B다. CRTP header1B를 더하면29B·27B이며 무선 ACK/재전송/공유채널 overhead 계산이나100Hz 성능 검증은 포함하지 않는다. [실제 CFLib class의 구성·size 계산](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-evidence/logconfig-demonstration.json).

정규화는 acc×9.81→m/s², gyro×π/180→rad/s, zrange/1000→m만 수행한다. 축은 firmware body 규약을 보존하며 추가 pitch/yaw 부호 반전을 하지 않는다. PWM·vbat는 원 값이고 gravity를 빼거나 observer bias를 보정하지 않는다. Flow count의 음수 변환은 Python의 넓은 정수에서 계산한다. int16 최솟값의 C narrowing/overflow, 공식 driver outlier rejection·선택적 MA/LPF를 재현하는 결과로 주장하지 않는다.

`decode_payload`는 ID/stamp를 제외한24B 또는22B에만 적용한다. callback mapping은 정확한 field 집합, Python float/int 및 native 범위를 요구한다. bool·문자열·누락/추가 field·정수형 field에 float·NaN/Inf·float32 초과·길이 불일치는 명시적 error로 거부한다. native uint16 range0/65535, motion/squal0 등을 임의 sentinel 또는 유효 측정으로 분류하지 않는다.

## normalized record의 제한

각 출력은 `record_kind="logger_snapshot"`, `physical_motion_ready=false`다. `captured_dt_s`, `measurement_capture_time_s`, `imu_average_window_s`는 null이고 각각 status는 `UNKNOWN`이다. Flow·range 측정 유효성과 measurement sample identity도 `UNKNOWN`이다. 낮은 SQUAL 또는 motion bit가 결과 metadata에 있지만 이를 generic texture score나 임의 valid gate로 바꾸지 않는다.

기본100Hz callback과 `logger_snapshot_interval_s=.01`은 측정 Flow dt·10개 IMU 평균·새 ToF sample을 뜻하지 않는다. 값이 같고 logger stamp만 증가한 packet은 새로운 **logger snapshot**으로 받으며, 동일 sensor sample인지 판정하지 않는다. 한 block 내부에서도 raw deltaX와 deltaY를 읽는 사이 task 갱신 가능성이 있어 atomic sensor burst 복원을 주장하지 않는다. [공식 CFLib decode와 standalone decode가 일치한 fixture](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-evidence/payload-fixtures.json).

## clock·age·block skew

wrap 주기는2^24ms=16777.216s다. 두 block은 하나의 명시적 boot session clock을 공유한다. 첫 logger snapshot을0으로 둔 session-relative 시각이며 부팅 epoch는 모른다. wrap 직후 처음 들어온 다른 block, wrap 전의 늦은 다른 block도 같은 상대 epoch로 계산한다.

- 같은 block stamp 반복: `duplicate_block_logger_timestamp`.
- 작은 역전: `out_of_order_block_packet`; 더 큰 역점프: `clock_backjump_requires_reset`. 둘 다 거부하며 cache·clock을 바꾸지 않는다.
- 너무 긴 forward/host gap은 caller policy error, 반주기 이상의 host 침묵 또는 정확 반주기 차이는 wrap ambiguity error다.
- PC monotonic clock 역점프는 device clock 문제와 별도 error다.
- `reset_session(new_id)`는 모든 block cache·unwrap 상태를 지운다. 기존 세션의 IMU와 새 세션 deck를 합치지 않는다.

24bit stamp만으로 wrap 직전의 재부팅과 정상 wrap을 구분할 수 없다. accepted wrap event는 `forward_wrap_assuming_same_boot`이고 boot-reset detection은 명시적으로 불가다. 실제 재접속·재부팅을 아는 caller가 새 session을 선언해야 한다. 작은 reset도 순서 역전과 구별 불가하므로 자동 재부팅 판정이나 clock 안정 보장을 하지 않는다.

`snapshot_view`는 block별 원 snapshot, PC callback receipt 이후 age, logger block skew, host receipt skew를 따로 반환한다. missing/stale/skew status와 caller-selected budget을 보존한다. receipt age는 sensor measurement age나 radio latency가 아니다. 동일 logger 시각 또는 `within_logger_policy`여도 `physical_motion_ready=false`다. PC/device clock offset·drift, 영상 정렬, 정확 sample freshness는 별도 미해결이다.

`ClockPolicy`와 stale/skew 한도는 호출자가 명시하는 진단 정책 S다. fixture의50ms reorder/1000ms forward/10s host gap, stale.05/.1s와 skew.001/.01s를 제조사 spec이나 착륙 안전 한도로 사용하지 않는다. [wrap/reset/order/stale/skew·UNKNOWN 예시](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-evidence/clock-and-metadata-fixtures.json).

## 실행과 검증

```bash
cd .
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python scripts/test_mission3_stock_telemetry.py
```

새 evidence directory를 보존하려면 `--out artifacts/mission3/stock-telemetry/<new-name>`을 추가한다. 기존 directory는 덮어쓰지 않는다. import/discovery만으로 artifact를 만들지 않는다. 공식 log.py/toc.py/crtpstack.py/callbacks.py만 격리된 package namespace에서 실행했으며 radio driver·Crazyflie connection을 만들지 않았다.

[최종 37개 시험 보고서](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-final-tests/report.json), [시험 출력](../../artifacts/mission3/stock-telemetry/20261008-native-v1-heisenberg-final-tests/test-output.txt). unit/axis/native type·payload·공식 callback 비교, unknown 측정 metadata, 원 값 반복, copy isolation, malformed input의 상태 보존, shared-clock wrap, explicit reset, out-of-order/backjump, stale/skew를 확인했다. 초기 pass 보고서도 보존했고, 큰 Python int host 시각의 overflow를 명시적 error로 처리한 뒤 최종37개를 다시 통과했다.

```text
scripts/mission3_stock_telemetry.py SHA-256:
67a2464ec45be2c1a80826929fff8ebd31c6ddf8e441bdd1d0c6948efb6bde12
scripts/test_mission3_stock_telemetry.py SHA-256:
7d6d1eae679d499f270423bd8082498b822610f8e337226d0d98d05d7bbff678
official cflib/crazyflie/log.py SHA-256:
b54bc69b16cda9b0fdeb1c742d690b0ab2e8d75ae6a6d1086ca8a08c7a21f0e5
official flowdeck_v1v2.c SHA-256:
6e81daa5eeeeda576e291efabc92e949a41f7f5f9f1d45d2c77dd213997bcabe
```

이 결과는 기본 로그 해석의 소프트웨어 계약이다. 실제100Hz 전달, startup calibration 잔여 오차, Flow/ToF capture freshness, 미션2 연속 경로, RTOS/supervisor·실기 제어·물리 접촉 안전성을 검증한 결과는 아니다.
