# Flow·ToF 입력 계약과 독립 검토 피드백

2026-10-09. 제조사값 P, 고정 공식 소프트웨어 C, 계산 D, 실험 가정 S, 미확정 U를 구별한다. 최신 제어 시험은 v11이지만 여기서의 관측 재분류는 보존 v9 원자료를 읽기만 한 별도 진단이다. 원 성공/실패나 평가기를 바꾸지 않았다.

[Bitcraze Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/)는 아래에 장착하며 PMW3901 광학 흐름과 VL53L1X 거리 관측을 EKF에 융합한다. 제조사는 무광 표면과 조명·표면에 따른 거리 성능을 설명한다. 이 설명은 20cm 움직이는 패드, 근접 구간, 우리 가상 품질 flag에 대한 검증이 아니다. 실제 지급 deck revision은 U다.

공식 firmware commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`의 [보존 원본·SHA](../references/bitcraze-flow-validity-contract-20261009/source-manifest.json)를 읽었다. 현재 웹의 최신 firmware를 고정 실행 코드와 섞지 않는다.

| 입력 | 공식 소프트웨어 C 계약 | 현재 가상 입력 / 미확정 |
|---|---|---|
| Flow | motion burst의 delta, motion, shutter, squal 등을 읽는다. 두 delta 절댓값<100이며 `motion==0xB0`, disable=false일 때 enqueue. 원 delta를 장착 좌표로 부호·축 변환한다. | generic 64px texture probe·ideal surface ray로 flow_valid를 생성한다. 실제 motion/squal/shutter를 생성·검증하지 않았다. |
| Flow 잡음 | 기본 std2 counts. adaptive=false 기본; 켜면 shutter 선형식을 사용한다. 계수는 해당 공식 코드의 측정 fit이며 센서 전체 전달함수나 FoV가 아니다. | 고정 속도잡음·가상 영상 quality 설정은 S. generic OpenCV texture 판단을 실제 PMW 품질로 바꾸어 부르지 않는다. |
| ToF | zranger2는 SDK의 RangeMilliMeter를 읽어 uint16 반환·25ms task·5m 미만 enqueue·거리별 std 식을 사용한다. 읽은 SDK status를 통과/거부 조건에 쓰지 않는다. | ideal ray 거리로 range_valid/accurate를 만들고 그 뒤에 잡음을 추가한다. 정확도 flag=true라도 측정값이4cm 미만일 수 있다. |
| ToF 품질 | 이 driver 경로는 SDK RangeStatus를 외부 입력 계약으로 보존하지 않는다. | signal strength/sigma/reflectance/crosstalk/실제 RangeStatus 응답은 U. 가상 정확도 신호를 실기 로그에서 그대로 받는다고 가정하면 안 된다. |
| 시간 | Flow task는 실제 usecTimestamp 차이를 dt로 사용한다. ToF는 별도 task/측정 준비 및 enqueue 시간이다. | 동기 100Hz PC IMU/PWM·Flow100/ToF40·지연0은 S이며 hardware/RTOS/radio timestamp 계약 U. |

## 읽기 전용 측정 경계 검사

[scripts/audit_mission3_range_contract.py](../../scripts/audit_mission3_range_contract.py)는 truth 없이 raw range와 producer stamp만 읽고, S 후보 `0.04<=raw<=4m`, stamp age<=.05s를 현재 onboard `0<=raw<5m`과 비교한다. 4cm는 기존 보수적 metric 정책 S이고 실제 RangeStatus·패드 정체성을 보증하지 않는다. candidate를 제어기/추정기에 연결하지 않았다.

[전체 원자료 결과](../../artifacts/mission3/acquisition/20261009-v9-measured-range-contract-v1/summary.json): v9 seed421/433/457의 새40Hz 거리 표본9483/7769/7837개 중 가상 accuracy=true지만 raw<4cm는5/3/4개다. onboard raw eligibility에서4cm 미만이196/195/189개다. raw-only 후보가 남기는 표본은9285/7573/7642개다. 기존 독립 검토의12/8/10은 held 값 소비 표본에 관한 것이므로 새 producer 표본 분모와 섞지 않는다. 세 source 모두 producer stamps가 중복되지 않았다. 이것만으로 live 통신 stamp가 확보됐다는 뜻은 아니다.

Flow 각23707/19423/19593 표본에는 실제 motion/squal/shutter 필드가 모두 없고, ToF RangeStatus도 없다. 없는 품질 값을 임의로 생성해 제조사 계약을 충족했다고 하지 않는다. 이 분석은 관측 eligibility만 비교했고 공식 full EKF의 자세/PID 경로 또는 비행을 재생한 결과가 아니다.

## 다음 수정 순서

1. 가상 flag를 API에서 실기 계약처럼 취급하는 지점을 구분하고, raw 값·producer 시간·실제로 받을 수 있는 status의 명세를 작성한다. 없는 status는 U로 유지한다.
2. 현행 전체 시험 소스가 동결 해제된 뒤, onboard ToF의 근접 raw gate를 별도 opt-in으로 비교한다. 동일 원 seed와 별도 source snapshot으로 자세 추정·실제 속력·접촉·실패 전분모를 검사한다. 공식 firmware 전체 수정/실기 flash가 이미 검증됐다는 뜻은 아니다.
3. IMU support 후보와 실제 패드 접촉 평가는 계속 분리한다. 공유 추력 model, 지면 효과, 잘못된 지지면, stale timestamp, motor gain 변화 반례를 보존하며 조건부 성과를 실기 qualified로 격상하지 않는다.

[v9 독립 검토](reviews/20261009-v9-contact-review.md)는 세 조건부 모델 접촉만 승인했다. 마지막 관측 결손·onboard 근접 ToF 융합·shared thrust model·초점 범위 아래에서 전체 frame을 제거하는 가상 카메라 정책·준비시간/에너지 미검증을 후속 과제로 남겼다. 세 판정의 원평가 결합을 main이 다시 확인한 [binding 결과](../../artifacts/mission3/acquisition/20261009-search-reverse-flow-slew-v9/independent-review-binding-validation.json)가 있다. 제조사/실기/형식 성공 집계는0이다.
