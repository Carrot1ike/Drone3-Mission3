# 미션3 간단 제어 블록선도

[미리보기 PNG](../diagrams/mission3-simple-control.png), [편집 가능한 SVG](../diagrams/mission3-simple-control.svg), [Mermaid 원본](../diagrams/mission3-simple-control.mmd).

센서 → 상태 추정 → FF+FB 착륙 제어 → 기체 내부 제어 → 기체 운동의 폐루프를 표시한다. 단계 판단은 추정 상태를 받아 현재 단계의 목표와 전환 조건을 상위 제어에 전달한다. 기체 움직임에 따라 영상·센서 측정이 바뀌면서 루프가 이어진다.

FF는 영상과 자기 운동으로 **추정한 차량 속도**를 사용한다. FB는 착륙 목표에 대한 상대 위치·속도·yaw 오차를 보정한다. 설정된 차량 속도나 차량 참값을 제어기에 직접 넣는 구조가 아니다.

간단 선도의 상위 제어 블록에는 PC 속도 PID와 명령 제한·접촉 후보 판단을 함께 묶었다. 실제 현재 경로의 출력은 자세·yaw 회전율·추력 RPYT 명령이며 기체 자세·각속도 제어가 모터 출력을 만든다. 정적 지도·영상 품질·표면 전환·예측 시간 제한·기체 추정기의 세부 연결은 [상세 선도](../diagrams/mission3-rpyt-control.svg)와 [구현 설명](implementation.md)에 남아 있다.

하단 성공 평가는 실행 종료 후 패드 접촉·모터0·3초 유지를 확인하는 기준이다. 온라인 IMU 접촉 후보와 실제 패드 접촉 확인은 구별한다. 이 선도는 제어 구조의 요약이며 실제 통신·제조사 사양 일치 검증 완료를 뜻하지 않는다.

재생성:

```bash
cd .
PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/render_mission3_simple_diagram.py
```
