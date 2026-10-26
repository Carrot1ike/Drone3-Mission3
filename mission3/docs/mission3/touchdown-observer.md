# 근거리 센서 이후 접촉 후보와 지지 유지

2026-10-08 개발 기록. 실제 기체 검증이나 제조사 일치 프로파일이 아니다. 아래 최초 설정·음성 대조는 당시 조건으로 보존하고, 후속 설정과 반례는 별도로 기록한다.

## 센서 사용 경계

[ST VL53L1X DS12385 Rev8 §7.3](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)은 최소 정확 거리를4cm로 제시하고, 아래 거리에서는 검출 가능과 거리 정확도를 구분한다. [PixArt PMW3901MB-TXQT L214-ZSZ v1.40 Table3](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.40-280119.pdf)은 권장 working range 최소80mm를 제시한다. 이는 해당 부품 문서 P이며, 지급되는 Flow deck의 정확한 광학계·렌즈 revision·조립 FoV·센서 품질 응답을 보정한 값은 아니다.

현재 조건부 센서는4cm 아래 ToF의 target-return과 metric-accuracy를 별도 플래그로 남기고, 정확도 플래그가 false이면 높이·vz 융합을 차단한다. Flow는8cm 아래 working-range 밖으로 분류해 속도 융합을 차단한다. 시뮬레이터의 raw 값이 여전히 정밀해 보여도 이를 근거리 측정 보장으로 사용하지 않는다. 실제 센서의 불연속·바이어스·SQUAL을 그대로 재현했다는 뜻도 아니다.

상위 BLIND_DESCEND의 geometry handoff 높이는 상판 위11cm로 설정했다. 이후 별도 TOUCHDOWN_DESCEND에서 IMU 적분과 마지막 상대 상태의 제한된 예측을 사용한다. 이 높이는 센서가 끊기기 전 인계할 실험값이며 제조사 착륙 높이 사양이 아니다. 기본 단계의12초 예측 제한과 별개로, 현재 texture-inertial profile은 마지막 영상 또는 물리 인계 후6초가 지나면 후퇴·상승 복구를 시도한다. 부분 지지 뒤 복구가 기체를 기울인 반례가 있어 안전한 복구 완료를 보장하지 않는다.

## 접촉 후보

가정한 CoM에 있는 IMU가 읽는 body-Z specific force는 수평에 가까운 비행에서 다음 관계를 갖는다.

`a_specific_z = rotor_force / actual_mass + support_force_z / actual_mass + unmodelled_force_z / actual_mass`

중력은 specific force에 직접 더하지 않는다. 따라서 호버 중 약9.81m/s²가 보이는 것만으로 지지 접촉을 판단할 수 없다. `TouchdownObserver`는 **명령 PWM**을 공식 소프트웨어 추력 곡선과 가정한 모터 응답으로 전파하고, 그 예측 가속도와 IMU 측정의 차이를 지지력 후보로 사용한다. 실제 plant의 모터 힘·접촉·위치·발 개수는 입력이 아니다.

질량과 추력 gain 오차가 residual을 만들 수 있으므로, 바닥 기준 Flow/ToF가 사용할 수 있고 추정 고도가 판 상단보다15cm 이상 높은 비행에서 force-model→specific-acceleration 비율을 추정한다.1초 이상 관측 span과2초 창을 사용하고, 잔차 RMS .3m/s² 아래의 보정만 받아들인다. 비율은 최종 접근에서 고정한다. 이 값은 센서 기반 식별이며 실제 조립 질량을 측정한 제조사 값이 아니다.

최초 시험의 최종 하강 명령은 .025m/s다. IMU로 전파한 상판 위 높이가2.5cm 아래면 collective multiplier를1초에 걸쳐1→.65로 낮췄다. 당시 후보는 사전 보정 유효, 예측 rotor 가속도≤7.3m/s², IMU residual≥1.2m/s², specific-force-Z가9.81±1.5m/s², 추정 tilt<10°, gyro norm<.3rad/s, 추정 상대 수평거리<3.5cm를 연속 .25초 만족하는 조건이었다.150ms 측정 창과 임계값은 모두 S인 소프트웨어 실험 설정이다. 후속 반례 때문에 이 최초 조건을 현재 권장 정책으로 부르지 않는다.

현재 `mission3-rpyt-support-constrained-sensitivity.json` 및 연속 인계 profile은 probe하한.60, 예측 rotor≤6.5m/s², specific force9.81±.2m/s²와 마지막 유효 보정45초 이내를 요구한다. 알려진 패드 높이.30m에 조건부 contact CoM clearance.0145m를 더한 추정 높이±.012m도 요구한다. 높이 밴드는 잘못된 차량 몸체 지지를 거부하기 위한 추정 기반 제약이며 접촉 측정이나 실제 조립 clearance가 아니다. IMU만으로 네 발·패드 정체성·slip를 알 수 없는 한계는 그대로다.

공중 보정 뒤 gain이1.5배로 바뀐 별도 음성 대조는 강화된6.5/±.2 조건에서도 공중 motor-off를 재현했다. 후보 순간 예측6.49968m/s², IMU9.75307m/s², 보정age1.060s였고 사후 실제 높이는5.02388m였다. 이 fixture에는 추정 높이 gate가 없으므로 전체 미션의 동일 실패라고 합치지 않는다. 최근 보정·강한 임계값만으로 충분하다는 주장에 대한 반례다. [전체 대조·한계·truth-free 재생](calibration-change-negative-controls.md)을 확인한다.

특정 오차 조합·접지 마찰·지면 효과·IMU 장착 오차가 같은 관측을 만들 수 있다. 사전 보정과 probe가 이 위험을 줄여도 네 발의 접촉을 식별하거나 실기 착륙을 보장하지 않는다. 접촉/모터off/네 발3초 판정은 원 로그를 읽는 별도 평가기와 독립 리뷰가 확인한다.

## 모터 정지 후

motoroff 뒤3초 동안 IMU tilt/gyro/specific-force 안정성을 확인한다. 거리 범위 밖 Flow를0속도 증거로 처리하거나 정밀 ToF를 요구하지 않는다. 관측 불안정이 .25초 이상 지속되면 sensor support unconfirmed로 종료한다. 자동 모터 재시동 정책은 구현하지 않았다. 정지 상태의 센서 안정과 이동 패드 네 발 지지는 서로 다른 증거다.

## 검증 기록

- 단위 반례: hover는 접촉 아님, freefall은 지지 아님, 단발 충격/과도 tilt/비허용 하강은 후보 아님, 미보정 질량·gain 오차는 거짓 지지 residual을 만들 수 있음. strict 정책에서는 미보정 모델 정지를 차단하고, 보정한 공중 호버 반례를 차단한다.
- `contact-observer-negative/20261008-042212-474802`: 실제 조건부 plant 질량29/35/41g, thrust gain .8/1/1.2, 모터 lag20/50ms의18개 조합.16개는 공중 hover→collective probe를 환경 접촉 없이 완료했고 거짓 접촉 후보0이었다.41g/gain.8 두 경우는 환경 접촉 때문에 시험 실패로 남겼다. 보정 실패 조합도 motoroff 후보로 승격하지 않았다. 이 시험은 전체 미션 성공률이 아니다.
- `physical/20261008-041621-455777-entry1-seed1`: 부품 거리 gate와 IMU residual 경로 최초 전체 시험. 접촉164.94s/off165.50s/종료168.52s, 내부9점/1.4275cm, 조건부 접촉 평가 pass. 당시 사전 식별은 없었으므로 새 strict 정책의 증거로 대체하지 않는다. HOLD151표본 모두 ToF accurate=false/Flow valid=false이고 IMU stable=true. 독립 검토 대기.
- `physical/20261008-042051-360247-entry1-seed8`: 사전 식별과 rotor 가속도 gate를 포함한 전체 시험. off210.54s/종료213.56s, 원 평가/독립 검토를 별도 보존한다. 제조사 일치 성공0이다.

재현:

```bash
PYTHONNOUSERSITE=1 MUJOCO_GL=egl src/crazyflie-ros2-sil/.venv/bin/python scripts/benchmark_mission3_contact_observer.py
bash scripts/run_mission3_physical.sh --entry 1 --seed 8 --video --respect-session-deadline
```
