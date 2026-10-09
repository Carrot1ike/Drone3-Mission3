# 작은 마커의 상대 거리와 늦은 관측: 전체 14건 진단

2026-10-09. v17의 사전 고정 baseline14 전체를 기존 원영상·모서리·센서 상태로 분석했다. 4CPU 독립 worker가 끝난 뒤에만 truth를 오프라인 오차 평가에 읽었다. 새로운 비행·제어 입력·인식 문턱 변경은 없다. baseline 조건부 접촉8/14와 실패6/14는 그대로이며 manufacturer/qualified 성공은0이다.

## 어떤 정보를 더 얻을 수 있는가

사용자 지정 마커 중심 높이0.40m, 바닥 ToF로 얻은 기체 높이, IMU roll/pitch, native decoded quad가 있다면 마커 중심의 상대 거리점은 조건부로 구할 수 있다. 왜곡 제거한 네 모서리의 대각선 교점으로 광선 r를 만들고, body 기준 camera offset o를 포함해 `λ=(marker_height-own_height-o_z)/r_z`, `q=o+λr`를 쓴다(D). 영상 확대는 하지 않는다. 바닥 거리 분류·자세·K/D/mount·고정 마커 높이가 정확하다는 조건(S/U)이 필요하다. near-horizon6°와 원 coarse7px 검사도 유지했다.

전체 decoded4,526표본 중 조건부 상대점2,539개가 남았다. 표본은 시간상 상관되므로 개별 독립 시험으로 세지 않는다. 아래 실패6건은 원 v17 판정으로 정한 전체 실패 분모다. seed는 다른 차량 초기 조건을 재현하는 식별자이며 좋은 결과를 골라 추가하지 않았다.

| 원 실패 seed | decoded / 조건부 상대점 | 상대 XY 오차 중앙값 | 기존 own pose로 world 변환한 오차 중앙값 | 상대 민감도 std(S) | 첫 상대점: handoff 후 | 가장 긴 관측 episode(S) |
|---|---:|---:|---:|---:|---:|---:|
| 521 | 0 / 0 | 없음 | 없음 | 없음 | 없음 | 0s |
| 557 | 23 / 23 | 2.32cm | 61.13cm | 22.47cm | 291.00s | 1.56s |
| 569 | 0 / 0 | 없음 | 없음 | 없음 | 없음 | 0s |
| 601 | 20 / 20 | 3.10cm | 45.64cm | 21.21cm | 284.76s | 1.50s |
| 613 | 21 / 21 | 3.69cm | 49.93cm | 27.87cm | 281.26s | 0.64s |
| 617 | 0 / 0 | 없음 | 없음 | 없음 | 없음 | 0s |

중간 민감도는 corner1px·height3cm·tilt1°를 독립 perturbation으로 가정한 1차 계산(S)이다. 제조사 정확도·보정 확률·상관된 world posterior가 아니다. episode는 관측 간격0.3초 이하라는 진단 설정(S)으로 묶었다. 300초는 원 탐색 한도이며 전체 비행 제한360초와 다르다. handoff의 센서 안정 준비시간도 실제 비행에 포함된다.

![원 실패 조건의 상대 정보와 늦은 관측](../../artifacts/mission3/acquisition/20261009-v17-baseline14-height-bearing-diagnostic-v1/relative-geometry-and-late-observation-v2.png)

이 조건부 모델에서는 작은 quad가 상대 접근점 정보를 제공한 세 사례가 있다. 그러나 그 점을 기존 own world pose로 바꾸면 큰 누적 오차가 남는다. 마커 상대점으로부터 차량 heading·pad pose·차속·turn 여부가 자동으로 생기지 않는다. 다른 세 실패는 decoded marker 자체가0이므로 이 계산을 추가하는 것으로 해결되지 않는다.

## 제어 설계에 적용하는 범위

현재 채택된 controller에는 연결하지 않았다. 후속 후보는 기존 ID 확인·기하·신선도 검사와 명령 포화 안에서 상대점과 영상 중심을 이용해 조기 접근 방향을 만들고, 원 native18px pose/heading 및 기존 uncertainty 조건을 확보한 뒤 LINEUP으로 넘기는 구조다. 잘못된 바닥 분류·near-horizon·관측 소실이면 해당 거리점은 사용할 수 없다. full pose 공분산을 임의로 줄이거나 차량 속도0.25m/s를 feedforward에 입력하는 구조는 아니다. 이동 표적 속도는 측정 이력에서 추정해야 한다.

연속 관측 확보와 초기 탐색이 함께 필요하다. 위 세 사례의 첫 정보는 원 탐색 한도까지 약9.0/15.24/18.74초만 남긴다. 현재 결과만으로 관측 시간 연장·접근 예산 확대·native pose 문턱 완화를 채택할 근거는 없다. 상대점 후보의 이동 표적 추정·상관·geometry loss·충돌 여유를 먼저 검증하고, 원 성공 유지 및 전 실패를 포함한 새 사전 paired 비행 비교가 필요하다.

[Keipour 등 visual servoing 논문](https://arxiv.org/abs/2104.01272)은 영상공간의 관측으로 이동 차량 접근 속도 명령을 만드는 방법을 제시한다. 여기서는 영상에 기반한 상대 제어를 검토하는 근거로만 쓴다. 해당 논문의 장비·표적·성능이나 전체 제어법을 우리 stock 기체에 검증 없이 적용한 것이 아니다. [Falanga 등 PAMPC](https://arxiv.org/abs/1804.04811)은 입력 한계와 표적 가시성·영상 속도를 함께 고려한다. 이번 height-bearing 진단은 그 NMPC/VIO 구현이 아니며 시야 유지와 동역학 제약을 함께 평가해야 한다는 원리만 참고한다. 두 원 논문 초록을2026-10-09 다시 확인했다.

P 제조사 광학 사양과 실제 모델의 S/U 차이는 [하드웨어 근거](hardware-sensor-evidence.md), [인식 연구](acquisition-research-20261009.md)에 있다. native mono320, 공칭 H/V87° 및 electronic rolling shutter를 유지하되 현재 순간 프레임·K/D/mount·PSF/readout·센서 전달은 조건부다. 원 worker는 capture 이전30ms 이내 command 상태를 사용하고 truth 평가는 capture 이후 첫 보존 표본을 쓴다. 이 시각 근사와 calibration/world correlation을 제거한 정확도 주장은 하지 않는다.

전체 [계획·14건 수치](../../artifacts/mission3/acquisition/20261009-v17-baseline14-height-bearing-diagnostic-v1/summary.json), [episode·원 입력 SHA·오프라인 truth SHA](../../artifacts/mission3/acquisition/20261009-v17-baseline14-height-bearing-diagnostic-v1/episode-provenance.json), copied helper를 보존했다. 원본 파일·원 실패·기존 서명은 바꾸지 않았다. 그림 v1은 범례가 수치 하나를 가려 QA 거부했고, 원 v1과 거부 기록을 남긴 뒤 v2에서 수정했다.
