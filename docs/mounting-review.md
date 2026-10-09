# AI-deck / Flow deck 장착·카메라 변환 검토

출처 확인일: **2026-09-30 (Asia/Seoul)**. 근거는 Bitcraze 공식 문서·제품 사진·공식 저장소만 사용했다.

## 적용 범위

대상은 **표준 Crazyflie 2.1 + AI-deck + Flow deck**이다. 지급될 덱 버전·하드웨어 개정·적층 위치는 미정이며, **AI-deck 1.1 + Flow deck v2는 시뮬레이션 작업용 조건부 프로파일**이다. 하드웨어 개조 금지, 제조사 방법으로 장착, 소프트웨어 변경은 허용된 것으로 이해한다. 기체와 Crazyradio가 없어 이번에는 실물 장착·보정을 확인하지 않는다.

이번 단계는 **기체동역학 없이 가상 카메라 광학중심의 위치·방향 경로를 직접 지정하는 MuJoCo 시야·ArUco 검출 기하시험**이다. 결과는 해당 가상 관측 조건에만 적용하며 하드웨어 일치, 자율비행, 접촉 착륙 또는 전체 착륙 시뮬레이션 통과를 보장하지 않는다.

## 제조사가 허용하는 장착

| 항목 | 확인 결과 |
| --- | --- |
| AI-deck | **위와 아래 모두 장착 가능**하다. 제공된 긴 헤더를 쓰며, JTAG 개발 편의에는 위쪽, 충돌 시 안테나 보호에는 아래쪽이 유리하다는 안내다. 어느 하나를 유일한 공식 기본 위치로 지정하지 않는다. [AI-deck 장착 안내](https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/#attach-the-ai-deck), [제품 설명](https://www.bitcraze.io/products/ai-deck/) |
| Flow deck v2 | **기체 아래 장착용**이며 광류·거리 센서는 아래 표면을 관측한다. AI-deck 카메라와 별도 센서다. [Flow deck v2 제품 설명](https://www.bitcraze.io/products/flow-deck-v2/) |
| 방향 | 전원을 끄고 장착한다. 덱의 점은 위, 십자는 아래, 화살표는 기체 전방에 맞춘다. 아래 장착을 덱 뒤집기나 카메라 하향 재장착으로 해석하지 않는다. [확장 덱 장착 안내](https://www.bitcraze.io/documentation/tutorials/getting-started-with-expansion-decks/#orientation) |
| 조합·핀 | 공식 호환표에서 AI-deck와 Flow deck 조합은 `yes`다. AI-deck 제공 핀은 **15 mm + 플라스틱 4 mm + 6 mm**이며 위 1덱·아래 2덱 구성에 맞춘 핀으로 설명한다. 위에는 덱 또는 배터리 홀더로 배터리를 고정한다. 이 핀 치수를 광학중심 높이로 대입하지 않는다. [공식 기계 구조·호환표](https://www.bitcraze.io/documentation/system/platform/cf2-expansiondecks/) |

AI-deck를 아래 선택할 경우 Flow의 하향 시야를 가리지 않는 적층 순서·간격을 확인해야 한다. 공식 호환표는 임의 적층의 시야·착륙 간섭까지 검증한 자료가 아니다. 제품 설명에 있는 핫글루 보강·카메라 교체도 이번 하드웨어 개조 금지 조건에서는 채택하지 않는다.

## 공식 도면과 카메라 방향

아래 PDF의 해당 도면을 시각적으로 확인했다. 문서 개정 번호와 실물 하드웨어 개정은 구분한다.

| 공식 자료 | 도면에서 확인한 범위·한계 |
| --- | --- |
| [Crazyflie 2.1 데이터시트 Rev 3](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf), §9, 3쪽 | 기판·헤더·전방 표시. 이 도면만으로 덱 포함 무게중심이나 카메라 광학중심을 정의할 수 없다. 문서의 하드웨어 개정 표는 B1이다. |
| [AI-deck 1.1 데이터시트 Rev 2](https://www.bitcraze.io/documentation/hardware/ai_deck_1_1/ai_deck_1_1-datasheet.pdf), §5, 2쪽 | `FRONT/UP` 표시, 앞쪽 카메라 커넥터 P3, 평면 치수 28.600 × 45.400 mm. 이는 §4 제품 전체 크기 30 × 52 × 8 mm와 범위가 다르며 렌즈 오프셋이 아니다. 하드웨어 개정 표는 C1이다. |
| [Flow deck v2 데이터시트 Rev 1](https://www.bitcraze.io/documentation/hardware/flow_deck_2/flow_deck_2-datasheet.pdf), §5, 2쪽 | `FRONT/UP` 표시와 기판 평면 치수. 하드웨어 개정 표는 C이다. 센서 외형과 광학·거리 측정 원점을 동일시하지 않는다. |
| [단색 카메라 모듈 데이터시트 Rev 1](https://www.bitcraze.io/documentation/hardware/ai_deck_mono_camera/ai_deck_mono_camera-datasheet.pdf), §6, 2쪽 | 모듈·FPCB·커넥터·렌즈 외형 도면. 기체에 조립된 지지대·모듈 위치 및 광학중심까지 이어지는 완전한 치수 관계를 확보하지 못했다. |

**방향에 대한 해석:** [공식 제품 사진](https://www.bitcraze.io/images/products/ai-deck-side-585px.jpg)은 카메라 렌즈가 덱 앞쪽에서 기판 면을 따라 바깥을 향하는 구성을 보여준다. 위 도면의 `FRONT/UP`와 장착 지시를 함께 보면 기본 구성은 **전방 관측, 영상 평면은 대략 전방에 수직**인 것으로 해석할 근거가 있다. 이는 사진·도면에 따른 정성적 해석이며, 장착 피치 0° 또는 정확한 회전행렬을 제조사 사양으로 확정한 것은 아니다. 위·아래 장착 허용을 하향 카메라의 근거로 사용하지 않는다.

## 카메라 → 기체 변환의 채택 여부

제조사 기체 축은 오른손 좌표계 **x 전방, y 왼쪽, z 위쪽**이다. 영상 좌표와 기체 축을 같은 것으로 취급하지 않는다. [Bitcraze 좌표계 문서](https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/)

검출용 광학좌표 `C`를 x 영상 오른쪽, y 아래쪽, z 시선 전방으로 정의할 때, 기체좌표 `B`로의 변환은 `p_B = R_BC p_C + t_BC`다. **실제 `R_BC`, `t_BC`는 모두 미확정**이다. 렌즈 외관 중심, 덱 중심, 기체 원점·무게중심은 정의가 다르므로 동일시하지 않는다. 공개된 전체 크기·평면도·핀 길이를 합쳐 임의 오프셋을 만들거나 미확정을 0으로 대체하지 않는다.

영상 방향도 소프트웨어에 의존한다. 확인한 [공식 WiFi 스트리머 소스](https://github.com/bitcraze/aideck-gap8-examples/blob/5fd95cac9c088b8ec7e2e011008548a34bf9406e/examples/other/wifi-img-streamer/wifi-img-streamer.c#L37-L81)는 `IMG_ORIENTATION = 0x0101` 레지스터에 `3`을 쓰며, 프레임 크기는 324 × 244다. 확인 커밋은 `5fd95cac9c088b8ec7e2e011008548a34bf9406e`다. 센서의 공칭 320 × 320과 수신 영상의 회전·반전·크롭을 혼동하지 않는다. **실제 수신 영상의 축·내부 파라미터·왜곡을 확인하기 전에는 외부 변환을 확정하지 않는다.**

이번 기하시험에서는 세계 → 가상 광학좌표 변환만 사용한다. 공칭 FOV로 만든 이상 투영과 직접 지정한 광학중심 경로는 시험 가정으로 표기한다. 이를 기체 위치·자세·무게중심 경로나 하드웨어 카메라 보정값으로 바꾸어 부르지 않는다. 질량·관성·공력·추력·렌즈 오프셋의 임의값도 이 검토에서 제공하지 않는다.

## 차량·마커 시험 변수와 주의사항

다음 값은 사용자 요청에 따른 **임시 차량 실험 변수**이며 제조사 기체 사양이 아니다. 차량 축을 x 전방, y 왼쪽, z 위쪽으로 정하고, 착륙장 중심에서 마커 중심까지의 벡터를 차량 축으로 표현한다.

| 변수 | 이번 배치 |
| --- | --- |
| 착륙장 | 0.30 × 0.30 m, 상면 높이 지면 기준 0.20 m |
| 마커 인쇄 한 변 | **0.10 m**, 검은 외곽 테두리 포함, 바깥 흰 quiet zone 제외 |
| 마커 중심 위치 | 착륙장 중심 + **(0.40, 0, 0.15) m**; 지면 기준 높이 0.35 m |
| 마커 면 방향 | 차량 앞쪽의 수직 면, 인쇄 정면은 뒤쪽 착륙장을 향함(법선 -x) |
| 차량 운동 | 초기 조건은 0.3 m/s 등속 직진 |

- 마커 중심과 착륙점은 다르다. **차량 축에서 착륙점 = 마커 중심 - (0.40, 0, 0.15) m**다. 검출 결과는 카메라·마커 축으로 표현되므로 이 벡터를 직접 더하거나 빼기 전에 축 회전을 함께 적용한다.
- 10 cm 정사각형의 인쇄 영역은 높이 0.30~0.40 m에 놓인다. 30 cm 착륙장의 앞 가장자리보다 마커 면이 0.25 m 앞선다는 것은 배치로부터 계산한 값이다. 흰 여백·실제 판·지지대의 크기와 간섭은 아직 확정되지 않았다.
- 검출·자세 추정에는 검은 외곽까지의 0.10 m를 사용한다. 흰 여백을 포함한 종이 길이와 혼동하면 거리 추정이 달라진다. 여백이 프레임 밖으로 잘리거나 지지대가 가리지 않도록 모델링한다.
- 하강·기체 피치 변화에 따른 네 모서리의 가시성, 근거리 초점, 픽셀 수·왜곡·흔들림·전송 지연을 별도 확인해야 한다. 공식 단색 모듈은 초점 대상 거리 35 cm, 피사계 심도 30 cm~무한대로 안내한다. 이는 ArUco 검출이나 착륙 성공 보장이 아니다. [단색 카메라 광학 사양](https://www.bitcraze.io/documentation/hardware/ai_deck_mono_camera/ai_deck_mono_camera-datasheet.pdf)
- Flow가 정지 바닥에서 움직이는 차량 상판으로 넘어가면 광류의 기준 표면도 바뀐다. 깃발·상판 경계는 거리 센서 시야에도 영향을 줄 수 있다. 이는 공식 센서 동작 설명으로부터의 설계상 주의이며, 이번 광학중심 시험에서는 Flow/ToF 추정과 접촉을 검증하지 않는다. [Flow 측정 설명](https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/#measurement-details)
