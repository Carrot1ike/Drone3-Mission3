# 최신 직선 시험 16회

원배치 `20261009-193646-264501`, 종료 2026-10-09 19:41:31 KST.
30×5m 경기장, 동쪽 28m 직선, 호버 준비 후 차량 최고 0.3m/s 출발.
드론 높이 1m, 초기 방향 북쪽에서 동쪽 20°, 차량 주행선과 옆거리 1.25→0.50m를 5cm 간격으로 각 1회 시험했다.

| 항목 | 결과 |
|---|---:|
| 완료된 시험 | 16 |
| 목표 ArUco ID 인식 | 16 |
| 조건부 접촉·PWM0·직선 3초 유지 완료 | 14 |
| 실패 | 2 |
| 독립 승인·제조사 일치·실기 성공 | 0 |

1.15m는 접촉 뒤 `inertial_texture_bridge_prediction_expired`로 종료 조건을 충족하지 못했다.
0.50m는 `visual_loss_after_discovery`로 중단했다. 실패를 전체 분모에서 빼지 않는다.
동일 seed 1001의 개발 조건 비교이며 모집단 성공률·일반화·실기 검증이 아니다.
기존 스타디움 실험과 구조 변경의 재현 시험은 이 16회에 합산하지 않는다.

## 파일

- [전체 결과표 CSV](all-16-results.csv): 16회 모두 포함.
- [검토용 집계 JSON](reviewed-results.json): 거리별 인식·완료·오차·속도·원자료 SHA.
- `evidence/d*-r01/`: 원평가, 명령 포화 검사, 픽셀 일치 검사, 실행 설정·manifest의 변경 없는 사본.
- [사본 SHA](source-copy-manifest.json): 사본 116개의 원경로·크기·SHA·byte exact 확인.
- [원자료 전체 SHA](raw-archive-manifest.json): 원배치 2,140파일의 경로·SHA.
- [Release 첨부 안내](release-assets.json): 공유용/전체 원자료 ZIP 크기·SHA. 아직 업로드하지 않아 URL은 null.
- [카메라 시각 범위](camera-time-ranges.json): 녹화 시작·종료 시각과 프레임 공백.

32개 영상은 공유 ZIP에 있다. 제3자 영상은 1× 재생이다. 카메라 H.264 미리보기는 녹화 프레임을 20FPS로 묶었으므로 전체 시험 시간과 일치하지 않을 수 있다. 원 FFV1과 `camera_frames.jsonl` 시각 정보는 전체 원자료 ZIP에서 확인한다.

현재 저장소 코드는 관리자/하위 제어로 구조 분리한 이후 버전이다. 16회가 실행된 소스와 라이브러리·환경은 전체 원자료 ZIP의 frozen snapshot으로 확인한다. 구조 분리 회귀의 28,480명령/이벤트 일치 및 대표 물리 비교 2회는 별도 검증이며 새 착륙 성공으로 계상하지 않는다.

자료: [8쪽 보고서](../../../docs/mission3/mission3-team-share-20261009.pdf), [상세 결과 설명](../../../docs/mission3/straight-arena-results-20261009.md), [제어 구조](../../../docs/mission3/hierarchical-control-20261009.md).
