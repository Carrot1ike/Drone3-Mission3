#!/usr/bin/env python3
"""현재 코드 경로의 편집 가능한 SVG와 미리보기 PNG."""
from render_control_diagram import Drawing,BLUE,GREEN,AMBER,MUTED


def control():
    d=Drawing(1700,1400,"미션3 현재 제어 구조 · 조건부 물리 시험")
    d.text(55,30,"미션 3 · 현재 제어 구조",36,anchor="start")
    d.text(55,88,"영상으로 차량을 발견하고, 추정 차량 속도 FF + 상대 상태 FB로 정렬·추종·착륙",24,anchor="start")
    d.text(55,133,"고정 조건: mono8 320×320 · 20×20cm 패드 · 높이30cm · 차량0.25m/s (plant 전용)",22,MUTED,"start")
    d.rect(40,200,1100,950,"#f6f9fe","#c6d5e5")
    d.rect(1190,200,465,950,"#f6faf7","#c7d8cc")
    d.text(590,217,"상위 추정·단계 제어 · PC에서 계산",26)
    d.text(1422,217,"제조사 C + 조건부 물리",25)
    d.node(70,280,320,155,"AI-deck 가상 영상","native 단색 320×320\n공칭 FOV87° / K·왜곡 U\n촬영 시각·실기 지연 U",BLUE,23)
    d.node(430,280,310,155,"ArUco 상대 관측","ID · IPPE 두 후보\ncapture 시각 IMU 자세\ncoarse 방위 ≠ 정밀 pose",BLUE,23)
    d.node(780,280,325,155,"차량 운동 추정","마커 → 패드 고정 변환\n영상 회귀 + 자기 운동\n차량 위치·방향·속도 미지",BLUE,23)
    d.arrow([(390,350),(430,350)]);d.arrow([(740,350),(780,350)])
    d.node(70,500,320,155,"가상 IMU · Flow · ToF","IMU1kHz / Flow100Hz\nToF40Hz · 유한 표면 patch\n모델·잡음·장착은 S",GREEN,23)
    d.node(430,500,310,155,"자기 상태 · 표면 가설","바닥 / 패드 / 혼합 / 모호\nFlow는 관측 표면 상대 속도\n높이는 바닥·패드 기준 분리",BLUE,23)
    d.node(780,500,325,155,"센서 사용 경계","부품 권장 범위의 보수 gate\nFlow <8cm: 융합 제외\nToF <4cm: 높이 융합 제외",AMBER,23)
    d.arrow([(390,570),(430,570)]);d.arrow([(780,570),(740,570)])
    d.arrow([(585,500),(585,450),(942,450),(942,420)])
    d.node(70,715,1035,135,"단계·안전 게이트","15초 대기 → 탐색 → 인식 → 후방 정렬 → 접근 → 추종 → 제한된 하강\n영상 나이 · 직선 잔여 길이 · 상대 오차 · 센서 품질 · 실패 시 상승 복구",BLUE,25)
    d.arrow([(942,655),(942,715)]);d.arrow([(585,655),(585,715)])
    d.node(70,925,480,155,"FF + FB 속도·yaw 명령","FF = 추정 차량 속도\nFB = 상대 위치·속도·방향 오차\n속도·가속도·회전율 제한",BLUE,24)
    d.node(605,925,500,155,"접촉 후보 · 모터 종료","공중 명령 추력 / IMU 응답 비율 보정\ncollective probe + 지지 residual\n모터0 뒤 IMU 안정성3초",AMBER,24)
    d.arrow([(310,850),(310,925)]);d.arrow([(855,850),(855,925)])
    d.node(1220,280,405,170,"하위 상태 공급","공식 Kalman core IMU 자세 전파\n위치·속도는 프로젝트 표면 추정\nstock 전체 EKF 재현 아님",GREEN,24)
    d.node(1220,560,405,155,"공식 PID · mixer · 전압 보정","공식 C 원문 host 실행\n세계 속도 + yawrate → PWM\nRTOS · 통신 · 실기 구현 미완",GREEN,23)
    d.node(1220,840,405,160,"MuJoCo 자유 기체","PWM → 조건부 motor model\n힘·토크 → 운동 · 접촉\n기체 pose 강제 이동 없음",GREEN,24)
    d.arrow([(1422,450),(1422,560)]);d.arrow([(1422,715),(1422,840)])
    d.arrow([(550,995),(575,995),(575,1120),(1160,1120),(1160,640),(1220,640)])
    d.arrow([(1105,1005),(1180,1005),(1180,690),(1220,690)],dashed=True)
    d.arrow([(1220,925),(1175,925),(1175,475),(210,475),(210,500)],dashed=True)
    d.node(55,1200,1590,150,"독립 오프라인 평가 · 실행 후만 읽기","참 위치·접촉 쌍·네 발·모터0·3초 유지·직선·실패 분모 → 독립 검토 → 새 코드/설정으로 재시험\n제어기로 참 차량 상태·속도·접촉을 전달하지 않음 / 조건부 접촉 성공 ≠ 제조사 일치 성공\nP: 제조사 공개 사실 · C: 공식 소프트웨어 모델 · S: 민감도 가정 · U: 미확정",AMBER,25)
    d.arrow([(1422,1000),(1422,1200)])
    d.save("mission3-current-control")


def phases():
    d=Drawing(1700,1300,"미션3 단계 인터페이스 · 현재 구현")
    d.text(55,30,"미션 3 · 단계와 종료 조건",36,anchor="start")
    d.text(55,88,"공통 명령: phase · 세계 속도 · yawrate · motoroff · done · failure / 결과는 독립 평가",24,anchor="start")
    nodes=[
        (55,180,"HANDOFF_IN → WAIT","알려진 진입점·정면·높이1m\n15초 fixed gaze\n이전 미션 EKF 연속 인계 U"),
        (625,180,"EXPLORE → WATCH","정적 지도 감시점으로 이동\n작은 yaw sweep으로 기다림\n트랙 영상 인식 가정 없음"),
        (1195,180,"COARSE → ACQUIRE","ID/bearing은 탐색만 허용\n정밀 pose 검사 통과 필요\n차량 운동 추정 초기화"),
        (1195,450,"LINEUP → LOWER_ALIGN","차량 뒤 .65m 정렬\n속도·방향 오차 감소\n가시 높이로 낮춤"),
        (625,450,"APPROACH → TRACK","추정 차량 속도 FF + 상대 FB\n남은 직선·영상 품질 확인\n곡선에서는 착륙 보류"),
        (55,450,"BLIND_APPROACH/DESCEND","마지막 신뢰 영상부터 제한 예측\n순수 패드 Flow일 때만 하강\n물리 adapter 인계 높이11cm"),
        (55,720,"TOUCHDOWN_DESCEND","Flow/ToF 근거리 융합 차단\nIMU + 마지막 상대 상태 예측\n명령 추력/IMU 지지 후보"),
        (625,720,"MOTOR_OFF_HOLD","지지 후보 확인 후 PWM0\n3초 IMU 안정성 검사\n네 발 접촉은 온라인 입력 아님"),
        (1195,720,"RESULT_OUT","LANDED_PENDING_REVIEW\n오프라인 접촉·모터·3초 평가\n독립 리뷰/제조사 프로파일 검사")]
    for x,y,title,detail in nodes:d.node(x,y,450,175,title,detail,BLUE,24)
    d.arrow([(505,265),(625,265)]);d.arrow([(1075,265),(1195,265)])
    d.arrow([(1420,355),(1420,450)]);d.arrow([(1195,535),(1075,535)])
    d.arrow([(625,535),(505,535)]);d.arrow([(280,625),(280,720)])
    d.arrow([(505,805),(625,805)]);d.arrow([(1075,805),(1195,805)])
    d.node(55,1010,1590,225,"실패 복구와 집계",
        "낮은 terminal에서 영상 상한·센서 품질·직선 여유 위반 → 제한된 후퇴 FF + 상승 → FF 종료·제동 → 안전 추정 높이에서 ABORT\n모터off 뒤 지지 불확실 → sensor support unconfirmed / 재시동 정책 미구현\n광학점 GEOMETRY_READY는 접촉이 아님. 조건부 접촉과 제조사 일치 성공을 별도 집계하고 실패 분모를 유지\n진입1의 자격 있는 독립 성공5회 전에는 다음 진입점 승격하지 않음",AMBER,25)
    d.arrow([(280,895),(280,1010)],dashed=True)
    d.save("mission3-current-phases")


if __name__=="__main__":control();phases()
