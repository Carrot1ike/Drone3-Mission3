# 미션3 구현과 실행

2026-10-08 개발 진행 기록. 마감09:30까지 새 결과를 갱신한다. 현재 결과는 가정이 명시된 조건부 시뮬레이션이며 실제 기체 사양 일치·실기 착륙 검증은 끝나지 않았다.

PC는 AI-deck 영상과 기체 센서로 차량과 자기 운동을 추정한다. 추정 차량 속도를 피드포워드로 사용하고, 상대 위치·속도·방향·높이 오차를 피드백으로 보정한다. 영상으로 차량을 발견하기 전에는 차량 위치·속도를 사용하지 않는다. 실제 차량 상태·설정된 .25m/s·곡선 여부는 제어기에 전달하지 않는다.

## 현재 권장 후보 경로

`AI 영상·body IMU·Flow·ToF → PC 자기/차량 추정 → 단계 제어 → FF+FB 속도·yaw 명령 → PC 공식 velocity PID → RPYT wire packet → 기체 공식 decoder → 기체 자세/각속도 PID → mixer/PWM → 자유 기체 물리`다. 실제 radio·ROS 전체 물리 폐루프·RTOS supervisor를 재현한 것은 아니다.

PC의 IMU 자세 추정과 기체의 전체 Flow/ToF EKF는 별도 상태다. 기체는 기존 공식 전체 EKF를 유지하고, PC는 평균 body IMU100Hz로 독립 자세를 전파한다. 이동·높은 패드를 기체 EKF가 바닥으로 해석하더라도 그 위치·속도·자세 보정이 PC의 중력 제거와 영상 pose에 곧바로 되먹임되지 않는다. 이는 초기 수평·잔여 gyro bias가 작다는 조건부 가정을 포함하며 장시간 자세 정확도를 보장하지 않는다.

바닥 Flow 속도가 유효한 동안 수평 IMU/자세 투영 잔차를 추정해 이후 짧은 관성 전파에서 차감하는 선택적 observer도 추가했다. 이는 gravity 투영·센서·모델 오차를 묶은 소프트웨어 추정값이며 제조사 IMU calibration은 아니다. 이동 패드·혼합 표면·무효 Flow에서는 갱신하지 않는다. 구현의 현재 경계는 [편집 가능한 다이어그램](../diagrams/mission3-rpyt-control.svg), 실패 진단과 수정 근거는 [독립 RPYT 검토](rpyt-full-estimator-failure-review.md)에 있다.

구형 `host_velocity_pid` 경로는 표면 보정한 PC 상태를 하위 velocity PID에 직접 공급했다. 이 경로는 `send_velocity_world_setpoint` 하나만으로 실제 기체에서 재현되지 않는다. 이 차이를 발견해 별도 RPYT 경로를 추가했으며 역사적 실행은 보존한다. 자세 부호·thrust unlock·zero stop 근거는 [RPYT 계약](rpyt-command-contract.md), 실제 ROS topic 경계 검증은 [ROS 인터페이스](ros-command-interface.md)에 있다.

| 모듈 | 실제 역할·검증 범위 |
| --- | --- |
| settings/track | 알려진 경기장·진입점·정적 지도와 제어 공개 설정. 트랙 영상 인식은 가정하지 않음 |
| vision/yaw_core | native mono8 320×320, ArUco ID·IPPE 두 후보·capture 시각 자세 제약. 합성 Brown K/D 일치와 실제 카메라 calibration은 다름 |
| estimator/uncertainty | 영상+자기 운동으로 차량 위치·방향·속도 추정. 바닥/이동 패드/혼합 표면 분리. 공분산·yaw floor는 조건부 오차 예산 |
| controller | 15초 WAIT→EXPLORE/WATCH→발견→후방 정렬→고도 조정→접근/추종→마지막 영상/제한 예측→물리 adapter 인계. GEOMETRY_READY는 접촉 아님 |
| physical_scene/flow_texture | 자유 기체·네 발·상판 물리, 가상 body IMU·유한 ray 표면 모델. 단색 패드 Flow 무늬/범위 gate는 일반 영상 모델이며 실제 PMW3901 SQUAL 재현 아님 |
| physical_estimator/firmware_host | 공식 Kalman core의 독립 onboard/PC 문맥. PC에는 추정 센서만, onboard 전체 EKF에는 raw 가상 Flow/ToF. 제조사 알고리즘과 센서/시간 모델의 동일성은 별개 |
| position_bridge/flight_controller | PC velocity PID100Hz→RPYT100Hz→공식 decoder→기체 자세/각속도 PID500Hz→mixer1kHz. 공식 C 원문 host 실행, 실제 firmware 이미지·통신은 아님 |
| touchdown | 공중 명령 추력/IMU 비율 식별→제한된 collective probe→지지 잔차 후보→PWM0→IMU 안정성3초. 온라인에는 접촉 참값이 없음 |
| evaluation/independent-review | 실행 종료 후 참 접촉·발별 법선력·PWM0·3초·직선·실패 분모·점수·source SHA 검토. 제어 입력 금지 |

## 08:15 현재 설정과 검증

현재 선택 설정은 `config/mission3-rpyt-tof27-depth-guard-only.json`이다. ToF 부품의 공칭 수광 대각27°를 반영한 각도 모델, 가까운 정적 관측점, 70cm 직선 준비 추종, 40cm 접근, 추정 마커 깊이 .32m 경계 검사를 사용한다. 깊이 경계에서 새 pose를 거부하면 이전 신뢰 영상 시각도 유지한다. 이 값은 실제 렌즈 보정에 근거한 오차 보장이 아니라 명시된 민감도 가정이다. [거리·높이 경계 비교](metric-depth-and-height.md).

같은 소스·설정·라이브러리를 고정한 새 지점1 시험10회가 모두 끝났고 **조건부 패드 접촉9/10, 개별 독립 승인5회 이상**으로 지점2 연구 시험을 시작했다. 실패1회는 가장자리 몸체 접촉·모터 미정지이며 그대로 분모에 포함한다. [원 계획과 검증 보고서](../../artifacts/mission3/frozen-series/tof27-depthguard-entry1-ten-v5/conditional-series-reviewed-report-0815.json). 제조사 사양 일치 qualified 집계는 여전히0이다. 지점별 진행은 [작업 기록](work-log.md)에 갱신한다.

미션 검사149개, 실제 logger 단위·축·시각 parser37개, 동결 실행 관리자35개, 보고서의 실패 거부 검사34개가 통과했다. 각 검사의 범위는 다르며 합계가 실제 기체 검증 횟수는 아니다. 보고서의 과거 파일 변조·시드 재사용 누락 반례도 보존했다. [독립 보고서 검토](conditional-series-report.md).

## 이전 구성의 비교 기록

이전 IMU texture bridge 고정 모델의 새 seed17/23/31은 각각 내부6/4/10점, 중심오차40.60/67.84/6.23mm로 접촉·모터0·3초 지지를 통과했고 각 독립 검토도 끝났다. [동결 계획](../../artifacts/mission3/frozen-series/inertial-texture-v1-entry1-frozen-0527/plan.json)과 원 로그·실패를 유지한다. 이 결과는 실제 RPYT 경로를 사용한 결과와 구분한다.

동일 training seed1에서 네 구성을 사전에 고정해 비교했다. [paired 계획/원 결과](../../artifacts/mission3/paired-rpyt/20261008-060137-plan/completed-identity-check.json):

| 구성 | 결과 |
| --- | --- |
| RPYT + 기체 full EKF 자세를 PC에 그대로 사용 | 마커 소실·ABORT, 접촉0 |
| PC 독립 IMU 자세만 추가 | 접촉162.40s·off163.14s·3초 지지, 내부8점/27.53mm, 접촉만 독립 승인 |
| 마지막 영상 대기거리만 .08→.30m | 접촉했으나 off 없음, 예측 만료 후 복구 중 기울기60° 시험 중단 |
| PC 독립 자세 + 대기거리 .30m | off163.60s·166.62s 종료, 조건부 접촉 평가pass·내부7점/36.52mm·접촉만 독립 승인 |

이 비교는 한 가상 조건에서 PC 자세 경로의 기여를 지지한다. 일반 성공률·실기 개선량은 아니다. `.30m` 선택은 조건부 Flow cone/추종 여유로 마지막 신뢰 바닥 속도를 확보하고, 공칭 카메라 깊이 약.53m를 남기기 위한 값이다. 실제 deck cone·mount는 U여서 실제 거리 최적값으로 확정하지 않는다.

새 [5개 동결 시험 계획](../../artifacts/mission3/frozen-series/rpyt-independent-entry1-five-v1/plan.json)의 unused seed43/47/53/59/61을 진행한다. 해당5회는 평가상47/59만 패드 접촉pass,43/61 실패 및53 차량몸체접촉 오판으로 실패했다. 따라서 다음 진입점 연구 확대는 하지 않았다. 후속 동일 코드/설정에서 5개 전체 물리 접촉이 개별 독립 검토를 통과하면, 가정을 표시한 연구용 모델에서 다른 진입 방향을 이어서 시험한다. 이는 immutable evaluation-contract/1의 제조사 일치 qualified 승격과 별도인 연구 범위 확대다. 제조사 프로파일 U가 있는 동안 공식 qualified 집계/승격은0·entry1이다. 실패를 제외하거나 과거 다른 코드의 성공을 5개에 섞지 않는다.

실패 수정에 재사용한 training43/61/1은 effective floor residual 추가 후 각8/9/8점,25.66/17.21/28.67mm로 접촉만 독립 승인됐다. 새로운 holdout 성공으로 세지 않는다.

추정 상판 높이·최종 보정 시각·더 엄격한 IMU/예측 rotor 조건을 추가한 뒤 재사용 training43/53/1은 각8/9/8점,25.57/10.82/28.64mm로 패드 평가를 통과했다. 개별 독립 검토는 접촉을 승인했고 제조사 일치 승인은 하지 않았다. 원래 잘못된 움직임을 유지한 negative53에서는 차량 몸체 위 모터off가 차단됐지만, 후퇴·상승 복구 중 기체가 기울어 시험이 중단됐다. 오판 차단과 안전한 복구는 별도로 검증해야 한다.

## 센서·접촉 한계

공식 부품 문서에 근거해 Flow80mm 아래 정밀 융합, ToF40mm 아래 정밀 높이 융합을 차단한다. 실제 드라이버 자체가 이 gate를 구현한다는 뜻은 아니다. 가상 Flow motion 수치는 유한 ray 평균이고 영상 correlation은 품질 gate에만 사용한다. plain white 패드·단일 edge에서는 유효 Flow가 없을 수 있다. [표면 무늬와 IMU 인계](flow-texture-and-inertial-bridge.md)에 36개 fixture와 보수적 제한을 기록했다.

마지막 영상 뒤 위치는 추정 속도·body IMU로 제한 시간 동안 전파한다. IMU/명령 rotor model이 보정되고 상대 오차·직선 여유·최근 영상 조건을 만족할 때만 최종 물리 adapter로 인계한다. 마지막 이미지/인계 후6초가 넘으면 제한된 후퇴·상승 복구를 시도한다. 모터off 후 재시작 정책은 없다. IMU 안정성은 네 발 접촉이나 slip를 직접 관측하지 못한다. [접촉 관찰기](touchdown-observer.md)의 모델 오차·ground effect·gain 변화에 의한 오판 반례를 계속 시험한다.

native320×320 영상 계약을 PC resize/색변환으로 보정하지 않는다. AI-deck 기본 streamer324 포맷과 제공320 포맷의 firmware ROI·촬영 timestamp·실기 지연은 미확정이다. K/D는 공개 H/V87°·대각115°에서 만든 합성 후보이며 실제 보정값이 아니다. DOF30cm 제한은 보수적 시험 정책이고 실제 blur/rolling shutter 모델은 미구현이다.

현재35g은 CF2.1 29 + AI1.1 4.4 + Flowv2 1.6g의 단순 합이다. 실제 revision·헤더·조립 질량/CoM/관성·camera/Flow/IMU 위치·접촉 형상은 U 또는 명시된 S다. 공식 PID tuning·모터 curve·ARM_LENGTH도 소프트웨어 상수이며 기체 실측값이 아니다. [제조사 센서 근거](hardware-sensor-evidence.md), [제조사 장착 CAD와 공백](manufacturer-mount-gap.md), [제공 기체 제약](../../config/provided-hardware.json)을 참조한다.

전압4.2V 일정 가정은 배터리 방전·덱 소비전력·미션1/2 소모를 재현하지 않는다. 수분의 탐색을 포함한 착륙 성공은 실제 남은 비행시간의 증거가 아니다. [탐색 시간과 배터리 공백](flight-time-gap.md)을 별도로 기록한다.

## 실행

프로젝트 루트 `.`에서 팀 `.venv`로 실행한다. 시스템 Python·vendor 원본·외부 인터넷 자료는 바꾸지 않는다.

```bash
PYTHONNOUSERSITE=1 MUJOCO_GL=egl src/crazyflie-ros2-sil/.venv/bin/python scripts/run_mission3_physical.py --entry 1 --seed 1 --profile config/mission3-rpyt-tof27-depth-guard-only.json --video
PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/evaluate_mission3.py run RUN_DIR
PYTHONNOUSERSITE=1 MUJOCO_GL=egl src/crazyflie-ros2-sil/.venv/bin/python scripts/review_mission3_physical.py RUN_DIR --video
```

`--respect-session-deadline`를 추가하면 이번09:30마감에서 중단한다. 재현할 때 training 실행과 사전 동결된 holdout 계획을 구분한다. 실제 ROS gateway는 별도 simulation topic의 영상·추정 sensor packet·명령만 검증했고 실제 Crazyradio 명령을 전송하지 않는다.

`config/mission3-rpyt-continuous-handoff-sensitivity.json`은 센서 기준1m 안정 호버를2초 유지한 뒤 모터·PID·추정기 상태를 유지한 채 미션3 시계만 시작한다. seed1/35초 smoke에서 engine8.66s 인계·23.66s 탐색·43.68s 미션 timeout을 확인했다. 사후 실제 높이는 인계 시.99368m였다. 이 준비 호버는 미션2 경로 자체를 구현하거나 실제 비행 인계를 재현했다는 뜻은 아니다. 전체 착륙 시험과 통신/telemetry rate·delay·dropout 검증은 진행 중이다. 후속 상태는 [work-log.md](work-log.md)에 기록한다.


## 07:33 당시 탐색과 단계 전환 결과

안정 호버 인계의 generic floor 품질 순간 저하를100ms 이내 조건부로 잇는 설정에서 지점1의 미사용 seed107/109/113/127/131이 모두 같은 소스·설정·라이브러리로 패드 접촉·모터 정지·3초 유지 평가와 각각의 독립 검토를 통과했다. 이5회는 8/9점, 오차16.747~23.973mm이며 실제 기체 사양과 완전히 일치한 성공으로 세지 않는다. [5회 원자료와 서명 검증](../../artifacts/mission3/frozen-series/rpyt-handoff-quality-grace-entry1-five-v2/five-reviewed-ledger.json).

지점2의 기존 관측점까지 긴 이동은 마커를 획득하기 전 불확실성 제한으로5회 모두 중단됐다. [가까운 정적 관측점](nearest-static-search.md) 수정은 마커를 획득하고 곡선을 따라 다음 직선으로 추종했지만, 곡선에 앞서 높이를 낮춘 채 오래 추종해 접촉 판정 보정값이 만료되는 별도 실패가 발생했다. 현재 `mission3-rpyt-straight-preparation-sensitivity.json`은 알려진 진입 위치에서 가까운 관측점을 고르고, 일직선 추종 높이70cm를 유지하다가 관측된 직선에 준비·최종 접근·정지 후 유지 거리가 남았을 때40cm로 낮춘다. 설정된 차량 속도나 live 위치를 제어기로 전달하지 않는다.

이 당시 설정의 지점2 training seed137/139/149는 이후 접촉 뒤 모터 정지·3초 유지 평가와 각각의 독립 검토를 통과했다. 다른 모델의 과거 실패를 남겼으며 training 성공을 새 holdout 성공 수에 섞지 않는다. 해당 단계의 미션 검사는145개였고 이후149개로 늘었다.

기본 firmware/CFLib 로그를 그대로 해석하는 [telemetry 단위·축·시각 계약](stock-telemetry-contract.md)은37개 검사를 통과했다. 실제 logger snapshot은 현재 가상100Hz IMU 평균이나 실제 Flow dt/ToF capture 시각과 같지 않다. [ToF 수광 대각 FoV 근거](tof-fov-reference.md)도 별도 sensitivity 설정으로 분리했다. 이 공백 때문에 현재 물리 시험은 계속 조건부 L2로 분류한다.
