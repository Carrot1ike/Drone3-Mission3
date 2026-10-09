# Flow deck의 표면 무늬와 마지막 접근

2026-10-08 05:18 KST 현재. 실제 기체가 없는 조건부 개발 결과다. 모든 시험은 entry1의 training이며, 제조사 일치 성공·다른 진입점 승격으로 집계하지 않는다.

## 실제 덱과 현재 모델의 차이

[Bitcraze Flow deck v2 설명](https://www.bitcraze.io/products/flow-deck-v2/)은 바닥을 향하는 광학 흐름 센서로 상대 운동을 측정하며 matte 표면이 적합하다고 설명한다. 실제 덱은 앞을 보는 AI-deck 영상과 별개다. 마커 영상은 계속 native mono8 320×320만 받는다.

로컬 공식 `flowdeck_v1v2.c`의 `flowdeckTask`는 약10ms마다 PMW3901 motion burst를 읽는다. 장착 축에 따라 `accpx=-deltaY`, `accpy=-deltaX`로 바꾸고, outlier 검사 뒤 motion register가 `0xB0`일 때 estimator에 보낸다. 기본 `useAdaptiveStd=false`, `flowStdFixed=2.0`이며, adaptive 옵션을 켜면 shutter 기반 표준편차 회귀를 쓴다. SQUAL 회귀 코드는 주석 처리돼 있다. 이 값은 소프트웨어 동작이며, 검증된 센서 정확도나 실제 덱의 FoV를 뜻하지 않는다.

기존 수학 모델은 여러 ray가 본 바닥/패드의 각속도를 평균한다. 따라서 완전히 흰 패드도 정확한 이동을 반환하는 낙관적인 부분이 있었다. 새 `flow_texture.py`는 별도의 가상 하향 영상으로 이 위험을 드러낸다. 실제 PMW3901의 이미지 센서 배열, lens revision, shutter, 조명, SQUAL, 내부 correlation을 재현했다는 뜻은 아니다.

가상 probe의 64×64, pinhole 42°, 무늬·gradient·correlation 임계값은 모두 S(민감도 설정)다. PMW3901의 공식 firmware 전환 상수 `35/.71674/.1`도 실제 이미지 해상도라고 해석하지 않는다. [OpenCV phaseCorrelate 설명](https://docs.opencv.org/4.13.0/d7/df3/group__imgproc__motion.html)은 두 영상 사이 translation을 구하는 일반 알고리즘의 근거다. phase response가 높더라도 반복 타일에서 다른 주기에 맞출 수 있으므로 정확한 이동을 보장하지 않는다.

## 가상 영상 반례

`scripts/benchmark_mission3_flow_texture.py`는 바닥/패드 중심/패드 모서리 × 높이6개 × 상대속도2개를 비교한다. 두 프레임의 기체 위치를 명시적으로 지정하는 센서 fixture이며, 물리 비행이나 착륙 시도가 아니다. 해석을 위한 수학적 표면 각속도는 제어기 입력으로 쓰지 않는다.

수정된 renderer의 결과는 [report.json](../../artifacts/mission3/flow-texture/20261008-051302-274056/report.json)에 있다. 36개 중 부품 거리 범위에 들어온28개, 가상 영상 품질 usable19개, 거리와 영상 품질을 모두 만족한17개다. 거리 범위는 맞아도 무늬/상관이 부족한11개가 있다. 균일한 흰 패드 중심은 특히 문제다. 단일 경계는 한 방향 운동만 제한할 수 있고, 반복 타일은 aliasing을 만든다.

MuJoCo `map.znear`는 metre가 아니라 `stat.extent`의 비율이다. 원래 .001은 이 경기장에서70mm clip이었으며, 너무 가까운 패드가 영상에서 사라지는 문제를 확인했다. 현재 물리 adapter는 실제1mm가 되도록 설정하고 scene XML에도 같은 값을 저장한다. 첫 fixture `050953`은 원본과 별도 clarification을 보존했다. 범위 밖의 근거리 영상 해석을 새 fixture로 대체한다.

![가상 하향 probe: 표시만 확대, 계산은 native64](../../artifacts/mission3/flow-texture/20261008-051302-274056/native-probe-display-montage.png)

## 전체 루프의 실패와 대안

`physical/20261008-051046-496887-entry1-seed1`은 가상 무늬 품질 gate를 적용한 전체 비행이다. 마커 획득133.26s, 정렬·접근 후159.74s에 relative Flow 불가로 BLIND_RECOVER,161.74s BLIND_RISE,162.38s ABORT였다. 실제 패드 접촉0, 모터off 없음이다. 흰 패드에서 Flow를 사용할 수 있다고 계속 주장하지 않는다. 이 실행의 renderer는 clip 수정 전 source snapshot이며, 근거리 clip 한계도 함께 보존한다.

별도 `mission3-inertial-texture-bridge-sensitivity.json`은 다음 조건으로 마지막 물리 support adapter로 인계하는 대안이다.

- 마지막 정상 영상으로 후방 정렬·패드 상대 위치·차량 속도를 추정했고, 직선·지도 여유·world odometry 검사에 통과한다.
- 패드 근처의 기하 조건, 상대 속도 조건, 마지막 영상 크기·focus 범위를 만족한다. 카메라를 놓친 뒤3초 이내에만 인계 후보를 만든다.
- 바닥 위 공중 구간에서 IMU와 명령 추력 모델의 gain을 식별했고, 현재 IMU 시각과 tilt가 유효하다. 이 capability는 접촉 신호가 아니다.
- 위 조건을 .20s 유지하면 GEOMETRY_READY를 반환한다. 이것은 기하 인계이며 착륙 성공이나 모터off가 아니다.
- 이후 마지막 영상 age와 support 인계 후 시간 모두6초로 제한한다. 시간이 지나거나 추정·경계·직선 조건이 깨지면 하강 중단·상승 복구한다.
- 모터off는 별도 IMU/명령 추력 차이의 지지 후보가 지속된 경우에만 수행한다. off 뒤3초는 IMU의 지지·tilt·gyro 안정 검사와 별도의 오프라인 접촉 평가로 확인한다.

6초, tilt, 추정 오차 임계값과 추력/IMU 모델은 조건부 설계 정책이다. 센서 bias·지면 효과·mount 오차를 실제로 측정하기 전에는 해당 시간 동안 실기 위치 오차가 패드 안에 들어온다고 보장할 수 없다. 기존 기본 경로는 relative Flow를 요구한다. 대안은 별도 opt-in 실험이며, 실패 기록을 지우거나 부품 범위 밖 Flow를 다시 유효화하지 않는다.

## 실행

```bash
cd .
bash scripts/run_mission3_physical.sh --entry 1 --seed 1 --profile config/mission3-flow-texture-sensitivity.json --video
bash scripts/run_mission3_physical.sh --entry 1 --seed 1 --profile config/mission3-inertial-texture-bridge-sensitivity.json --video
```

무늬 gate만 적용한 실패, bounded IMU 대안, 기존 낙관적 수학 Flow 결과를 서로 구분한다. 모든 실험의 source와 설정은 실행 전에 snapshot으로 보존된다.
