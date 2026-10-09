# Mission3 제조사 장착·CAD·gyro 보정 근거와 공백

독립 검토자 Heisenberg / Codex-Independent-Flight-Reviewer, 2026-10-08 KST. 조건부 대상은 **표준 Crazyflie 2.1 + AI-deck 1.1 C1 단색 카메라 + Flow deck v2 C**이며 실제 배정 revision·장착 순서는 U다. 기존 접촉 서명과 제조사 적합 판정은 그대로 유지한다. 기계 판독 요약은 [새 JSON](../../config/mission3-manufacturer-mount-evidence.json), 원본·전체 SHA-256·조회 URL은 [증거 manifest](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/source-manifest.json)에 보존했다.

**P** 제조사 공개 사실/명목 CAD, **C** 공식 소프트웨어 상수·처리, **D** 근거에서 계산한 조건부 도출, **S** 시험 가정, **U** 미확인. 제조사 운영 저장소의 시뮬레이터 상수도 실물 측정 사양으로 승격하지 않는다. 수치가 없는 조립 CoM·광학 중심·장착값은 추정 scalar로 채우지 않았다.

## 1. 유도 가능한 것과 남는 값

| 항목 | 판정 | 가능한 범위 / 공백 |
| --- | --- | --- |
| 덱 설치 위치·방향 | P | Flow v2 아래, AI-deck 위/아래; FRONT와 UP/DOWN 표시를 따라 설치. 실제 조합·순서 U |
| 확장 헤더의 평면 결합 기준 | P→D | 공식 KiCad 템플릿의 헤더 중심 간격22mm, 핀 피치2mm, 첫/끝 핀 중심 간격18mm. 실제 센서 중심 좌표는 별도 |
| 덱 기계 축의 명목 회전 | D | 기체와 덱 축을 FRONT/LEFT/UP으로 정의하고 표시대로 이상적으로 조립하면 축 회전은 I. 실제 오차·카메라 영상 축·렌즈 변환은 U |
| 카메라→body 전체 SE(3) | U | 홀더 3D datum, 모듈 착좌, 광학 투영 중심, 조립 stack 높이, 실제 body 원점이 모두 연결되지 않음 |
| Flow/ToF→body 전체 SE(3) | U | 아래 장착은 확인하지만 실제 PMW 광학 중심/ToF 발광·수광 기준점, PCB 내부 위치·깊이·장착 순서·body 원점이 미해결 |
| 최하단 높이·접촉 발/CoM 기준 거리 | U | 핀·커넥터 단품 치수만으로 전체 조립 위치/ground clearance를 확정할 수 없음 |
| BMI088 시작 시 gyro bias 차감 | C | 공식 드라이버에서 bias 추정 후 raw에서 차감. 실제 잔차 사양은 별도 U |
| 주입 gyro yaw residual0.03°/s | S | 프로젝트의 조건부 직접 IMU 입력 가정. 제조사 보정 후 잔차나 정확도 사양 아님 |

[공식 2.x 장착 안내](https://www.bitcraze.io/documentation/system/platform/cf2-expansiondecks/)와 [보존 orientation 그림](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/downloads/orientation-symbols.png)은 덱 방향과 설치 위치를 알려 준다. AI-deck와 함께 오는 15+4+6mm 핀은 위1/아래2 덱 구성용이며 넓은 spacer가 아래 AI-deck를 받치는 용도다. 이 설명만으로 특정 AI/Flow 조립 순서나 센서의 body z를 선택하지 않는다.

## 2. 공식 native CAD에서의 실제 계산

공식 [확장 템플릿](https://github.com/bitcraze/crazyflie2-exp-template-electronics/blob/d407886d21958346fb94a4e9fc28c5b41eb0f408/ecad/template.kicad_pcb)은 Crazyflie 2.0용 설계이며 위 2.x 공식 안내가 2.0/2.1 template으로 링크한다. docs는 Eco1.User의 본체 윤곽·주요 부품도 **2.0 참고 도형**이라고 명시한다. 실제 2.1 PCB/AI/Flow의 측정 layout로 대체하지 않는다.

사진 pixel을 재지 않고 native `(at ...)`와 through-hole pad를 [독립 추출 코드](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/derive_template.py)로 계산했다. [전체 좌표와 계산](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/derived-template-geometry.json)은 재검산 가능하다.

| native CAD 항목 | mm 값 / 기준 |
| --- | --- |
| P1(CF_EXP_LEFT) footprint 원점 | (139.001,86.00092), 회전180° |
| P2(CF_EXP_RIGHT) footprint 원점 | (139.001,108.00092), 회전180° |
| 새 H 평면 datum | 두 원점의 중점(139.001,97.00092); u=KiCad+X, v=KiCad+Y |
| H 좌표의 P1 pin1 / pin10 | (+9,−11) / (−9,−11) |
| H 좌표의 P2 pin1 / pin10 | (+9,+11) / (−9,+11) |
| 도출 | 행간22, 피치2, 중심간 총span18 |

EXP_ORIENTATION의 FRONT 화살표는 native +X, 앞면 circle-dot은 UP이다. 이 평면 보기에서 LEFT는 −KiCadY다. H는 **정의한 CAD datum**이며 CoM·IMU·현재 freejoint 원점이 아니다. 같은 커넥터에 올바르게 결합하는 덱의 헤더 datum은 명목상 XY 정합된다. 덱 기계 축 회전 I는 FRONT/LEFT/UP 축 정의와 이상적 설치에서만 나오는 D이며 실제 camera R_BC/Flow 렌즈 R로 넣지 않는다. 템플릿 thickness1.6mm도 실제 AI/Flow PCB 두께 사양으로 쓰지 않는다.

## 3. 단품 도면과 조립 stack의 한계

| 공개 자료 / 명목 치수(P) | 조립 변환으로 연결되지 않는 값(U) |
| --- | --- |
| AI-deck 1.1 연결 도면 PCB28.600×45.400mm, 홀더 나사 중심 간12.000mm | 나사 datum→홀더 pocket→카메라 광학 중심의 3D 위치·각도 |
| mono 모듈 단품 도면 holder5.00±.10mm, module height5.32±.30mm, OP FoV start 표기 | 단품 datum을 조립 홀더 datum에 연결하는 변환·투영 중심·영상 축. FoV 시작점은 calibrated pinhole center가 아님 |
| Flow v2 평면 도면27.940×21.079mm, 단품 외형21×28×4mm | 광학 중심/ToF 기준점 좌표와 전체 stack에서의 최하단 위치 |
| female connector Rev1 10핀,20×2×3mm(SMD pad 제외) | 납땜·착좌·핀 삽입 깊이를 포함한 실제 PCB 간격 |
| long15+4+6mm Rev1, 끝 구간15±.1/6±.1mm, 피치2mm | 전체 조립체의 board z·sensor-to-foot 거리 |

원본 도면: [AI](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/sources/hardware/src/products/ai-deck-1_1/datasheet/ai-deck-mechanics.png), [mono module](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/sources/hardware/src/products/ai-deck-mono-camera/datasheet/ai-deck-mono-camera-mechanics.png), [Flow](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/sources/hardware/src/products/flow-deck-2/datasheet/flow-deck-v2-mechanics.png), [pins 공식 PDF](https://www.bitcraze.io/documentation/hardware/long_pins_15_4_6mm/long_pins_15_4_6mm-datasheet.pdf). 도면/PDF의 숫자는 단품 datum에만 적용한다. 치수 항목을 합쳐 CoM offset을 만드는 데 필요한 조립 조건이 없다.

[공식 카메라 장착/교체 안내](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/guides/camera-replace/)는 두 나사·FPC·foam pocket 고정을 보여 준다. stock 전방 설치의 정성 근거이며 foam의 pose 허용오차나 holder CAD datum은 제시하지 않는다. 카메라를 변경하라는 제안으로 사용하지 않았다. 모듈의 mirror/flip 가능성도 실제 출력 영상 축 보정의 증거가 아니다.

변환은 `p_A = R_AB p_B + t_AB`, B=실제 body/조립 원점, H=본체 헤더 datum, D=AI-deck 헤더 datum, M=홀더 datum, C=광학 frame으로 정의하면 `T_BC = T_BH T_HD T_DM T_MC`다. H↔D의 명목 평면 정합·방향 일부는 D로 얻지만 **t_BH, stack z, T_DM, T_MC**가 미해결이므로 전체 R_BC/t_BC는 null이다. Flow도 `T_BF = T_BH T_HDf T_DfF`의 마지막 optical reference와 stack/body 원점이 미해결이다. `camera(20,0,15)mm`, `Flow(0,0,−5)mm`, `IMU(0,0,0)`는 기존 실험 S 좌표로 남는다.

조사 범위는 고정 hardware8207400d, mechanicsc70aa743, simulation7e93752d, aideck examplesd39d2ec5의 전체 tree, 신규 확장 template d407886d와 공식 장착 자료, 현재 Bitcraze 조직 공개 repository 목록78개다. [범위 inventory](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/inventories)에서 실제 AI holder/Flow optical assembly의 완전한 정량 native CAD를 확인하지 못했다. **존재하지 않는다는 결론은 아니다.** 공식 저폴리 simulation mesh/PROTO camera 상수는 software C이며 실기 extrinsic 근거에서 제외했다. 공개 [crazyflie-electronics](https://github.com/bitcraze/crazyflie-electronics/blob/5e8d0fcf343c9e1517c617a8fbd9576d63f45370/README.md)는 명시적으로1.0용이라2.1 장착 계산에서 제외했다. mechanics의 mockup/기존세대/출처가 실물2.1 조립 metrology로 확정되지 않은 파일도 이름만으로 승격하지 않는다.

## 4. BMI088: 실제 부품 사양과 공식 시작 보정

[Bosch 공식 BMI088 데이터시트](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf), **BST-BMI088-DS000-19 Rev1.9(2024-01), §1.3 Table5 p10**을 [보존 렌더](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/rendered/bmi088-page10.png)로 확인했다.

| 공개 부품 항목 | 값 / 조건 | 해석 |
| --- | --- | --- |
| zero-rate offset | Typ ±1°/s, nominal VDD/25°C, slow/fast offset cancellation off | 보정 전 부품의 typical. 보정 후 잔차·maximum 보장 아님 |
| offset 온도 변화 | Typ ±.015°/s/K,−40…85°C, ±2000°/s 범위 | gyro bias0.03°/s 잔차 근거로 환산하지 않음 |
| 출력 noise | Typ .1°/s RMS, BW47Hz(.014°/s/√Hz) | 다른 필터·ODR의 현재 주입noise0.08°/s와 동일하게 취급하지 않음 |
| sensitivity | Typ16.384LSB/(°/s),25°C,±2000°/s | 양자화 스케일과 bias 정확도는 별개 |

[공식 driver 고정 소스](https://github.com/bitcraze/crazyflie-firmware/blob/f6e0f0a3b526861328caf3296a7ecf00bb915d39/src/hal/src/sensors_bmi088_bmp3xx.c#L309)를 읽었다. 일반 branch는512 raw 표본의 평균/분산으로 각 축 분산<100(raw count²), 최소 tick 조건을 만족할 때 bias를 latch한다(L748–766,845–866). `GYRO_BIAS_LIGHT_WEIGHT` branch는 첫1000표본 평균이며 시작 시 안정 상태를 요구한다(L696–741). 이 매크로의 실제 기체 build 선택은 U이며 고정 보정 완료 시간을 보장하지 않는다.

raw에서 `gyroBias`를 먼저 차감하고 deg/LSB 스케일을 적용한 뒤 airframe 정렬·저역통과·estimator queue로 보낸다(L325–334). `AreCalibrated()`도 gyroBiasFound를 반환한다(L287–289). 이는 source의 처리 사실 C이며 실제 기체 보정 성공·온도/진동에 따른 잔차를 증명하지 않는다. 일반 bias는 최초 valid 뒤 재추정하지 않는 source 구조이고, temperature 추적 보정·잔차 한계가 이 코드에서 자동 보장되지 않는다.

±2000°/s 설정의 `(2×2000)/65536 = .06103515625°/s/rawLSB`는 C→D다. 분산100의 표준편차 환산 `.6103515625°/s`는 **보정 허용 안정성 threshold의 단위 환산**이며 bias mean 오차 bound가 아니다. 평균 bias가 float이므로0.03°/s가1LSB보다 작다는 사실만으로 불가능하다고 판단하지 않는다. driver 설정 BW116/ODR1000Hz도 C이며 Bosch BW47Hz noise 조건과 다르다.

공식 `imu_sensors.imuPhi/imuTheta/imuPsi`는 persistent 회전 파라미터이고 공통 기본값0°다. `sensorsAlignToAirframe`의 축 회전은 확인했으나 실제 장치의 parameter 값·PCB/CoM 위치·camera/Flow 변환을 증명하지 않는다. 공식 Flow driver의 `accpx=−deltaY`, `accpy=−deltaX`는 장착 convention에 맞춘 **2D motion 축 처리 C**다. 이 부호만으로 센서의 3D 위치를 유도할 수 없다. `FLOWDECK_POS_X/Y/Z` 공통 기본0 및 gyro lever-arm 항은 C이며 실제 장착점0의 공개 측정 사양이 아니다.

현재 검토한 [051833 S 입력 근거](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/project-sensitivity-context.json)는 simulated IMU에 yaw bias0.03°/s를 직접 더한 뒤 host Kalman으로 보내며 실제 RTOS BMI088 startup driver를 실행하지 않는다. 따라서 이 값은 **소프트웨어 전처리 뒤 남는 효과를 시험하기 위한 S 가정 또는 미모델링 전처리의 민감도**로만 해석한다. 제조사 정규 보정이 이 잔차를 남긴다고 주장하지 않는다. 새 조건부 gyro front-end fixture가 필요하면 startup subtraction·raw 스케일·안정 시작/불안정 시작·보정 후 변화의 영향을 별도 hash/seed에서 확인할 수 있다. 실물 spec 숫자로 기존 run을 소급 보정하지 않는다.

## 5. 메인에 넘기는 구체 공백

현재 자료로 문서화할 수 있는 것은 **stock 설치 가능한 위치·방향, connector 기준의 명목 평면 정합, 단품 치수, 공식 gyro startup 차감**이다. 완전한 제조사 mount profile을 닫으려면 같은 revision의 본체/덱/holder CAD datum, 실제 stack 순서와 fully seated 기준, optical center·영상 축, body 원점/조립 CoM 관계가 필요하다. 비하드웨어 장착 추론은 S/U로 유지한다. 지금 장비가 없으므로 사용자에게 실물 측정이나 장착 변경을 요구하지 않는다.

near 센서4/8cm 정책은 **부품 문서를 근거로 메인 개발자가 선택한 보수적 시험 정책**이다. 단품 operating envelope와 실제 덱의 lens/quality/mount 검증은 별도다. 본 조사에서 해당 정책·모델·controller·Flow 코드·기존 평가/서명은 변경하지 않았다. 이전 조건부 접촉의 물리 계약 pass를 취소하거나 manufacturer qualified로 승격하지 않는다.

| 핵심 보존 원본 | 전체 SHA-256 |
| --- | --- |
| template.kicad_pcb | `d9cc1d5ae5c1acc74a2fba0001a07eca7476acef5cb93ca89994769e53e1a74f` |
| AI-deck 평면 도면 | `bb06f51e030552a862cdd7d5a4b1fe7a9367ed1b26ca179ac4163eae00f6398f` |
| mono 모듈 도면 | `1326b5c77def881cca199a32999fddba67742b5e386b04e915ed6518500ef1e0` |
| Flow v2 평면 도면 | `46ecf9211f0da47988ca0b6d16f426a5a6afa8539ac1c5a7cee09786b08db2ff` |
| 15+4+6mm 핀 PDF | `db8c35069d44c059e722e757952e07ebb321e5972594077dc8a300c6dda7db18` |
| Bosch BMI088 Rev1.9 PDF | `53ab734dba49ac202fa6182fc2b545df6c6d7459c9fec6e851a1209e6bac417e` |
| 공식 gyro driver f6e0f0a3 | `c3093cde69776c84e8ee5b89600b108e8e94daae32722ac2e22048853a4f339a` |

[최종 검증](../../artifacts/mission3/manufacturer-mount-evidence/20261008-heisenberg/final-verification.json)은 모든 보존 source hash·파생 좌표·미확정 null·기존 서명 불변을 검사한다.
