# 마지막 접근 연속성: 10월9일 후속 시험

사용자가 중단을 요청할 때까지 이어지는 세션이다. 전진과 하강을 같은 quintic reference로 진행하고, 마지막 구간에서 기존 높이·횡방향 확인 1초를 함께 누적하는 후보를 구현했다. 정렬·영상·Flow/ToF·불확실성·직선·접촉 조건과 명령 포화를 유지했다. 실내35×12m, native mono320×320, 연속 이동 차량0.25m/s와 원 seeded phase도 유지했다.

두 재사용 training 조건에서 기존 방식과 후보를 같은 source로 각각 실행했다. v21과 v22에 각4회이며 이전165회와 합친 전체 완료 기록은173회다. 이 합계는 서로 다른 source·정책·split을 포함한 실행 원장이고 전체 성공률의 분모로 합산하지 않는다. 신규8회에서는 조건부 모델 접촉·PWM0·직선3초 유지와 일반 명령 제한이 모두 통과했다. v21 native12,334장과 v22 native12,294장의 무손실 원본 SHA가 일치했다. 제조사 사양 일치·실기·qualified 성공은0이고 진입2로 진행하지 않았다.

| 조건 | 기존 수직 정지 | v22 후보 수직 정지 | 감소 | 기존 착지 오차 | 후보 착지 오차 |
| --- | ---: | ---: | ---: | ---: | ---: |
| seed571 |10.78s|7.16s|33.58%|1.46cm|1.81cm|
| seed619 |9.44s|7.16s|24.15%|0.94cm|2.46cm|

정지 시간은 첫 LOWER_ALIGN부터 실제 첫 접촉 전까지의 원 물리 로그에서 `abs(actual vz)<.005m/s` 구간을 시간 적분했다. 명령의0이나 영상 프레임 수로 대체하지 않았다. 사전 기준은 두 조건 모두 정지 시간20% 이상 감소·접촉 보존·착지 오차 증가1cm 이하이다. v22는619의 오차 증가1.519cm로 탈락했다. v21도571 오차와619 정지 기준을 통과하지 못했다. 둘 다 원 baseline을 대체하지 않았다. 비행이 접촉에 성공했다는 판정과 제어 후보 채택 판정은 별개다.

실제 하강과 수평 접근이 함께 진행되어 경로의 큰 계단은 줄었지만, 약0.40m 구간의 확인/마지막 접근 대기는 남는다. 현재 영상은 이를 그대로 보여준다. 기체 센서 근거가 부족한 동안 무조건 계속 하강시키지 않는다. 후보 reference의 C2 연속성과 실제 기체 jerk 제한도 구분한다.

[v22 비교 그림](../../artifacts/mission3/acquisition/20261009-concurrent-confirm-approach-paired-training-v22-continuity/approach-continuity.png), [원 비교 수치·입력 SHA](../../artifacts/mission3/acquisition/20261009-concurrent-confirm-approach-paired-training-v22-continuity/summary.json), [사전 계획](../../artifacts/mission3/acquisition/20261009-concurrent-confirm-approach-paired-training-v22/plan.json)에 모든 조건을 남겼다. 실제 suite225개가 통과했다. 기존 reviewer에게 두 source의4회씩 새 독립 검토를 요청했으며 현재 결과는 main 평가/unsigned 비교다. 실제 서명·최종 보고가 오기 전에 검토 완료로 표시하지 않는다.

다음은 마지막 영상 이후 상대 상태와 기체 속도 오차를 분석하는 일이다. 6개 기존 궤적의 terminal 진입 시 차량 속도 오차에 남은 시간을 곱하면 약0.5~1.7cm 규모의 D 민감도가 나온다. 그러나 v22에서 그 방향과 실제 착지 오차 방향도 같지 않아 이를 단독 원인으로 주장할 수 없다. 기체 속도/자세·상대 위치 전파·접촉 응답을 함께 분해해야 한다. 현재 차량 추정은 world-position regression이며 기존 relative-kinematics 옵션이 이미 채택됐다는 잘못된 가정은 배제했다. 입력/FF/pad/physics truth의 경계를 그대로 유지한다.

활성 예약은 같은 채팅을5분 간격으로 이어가며 의미 있는 결과만 알린다. 사용자가 중단을 요청하면 새 실행을 시작하지 않고 본인이 시작한 실행을 정리하고 예약을 삭제한다. 실패·원자료·과거 사양 미확정과 독립 판정은 보존한다.


2026-10-09 10:55KST 추가: v21/v22실제독립검토를읽고원평가·snapshot·input/evidenceSHA와main결합을마쳤다. 각source의조건부접촉4개가승인됐으나두후보의사전개선기준탈락은유지한다. 새v23은perfect-reference의D상쇄수식만수정한선택정책이고기존baseline은raw20파일byte동일이다. 전체228시험통과후네재사용training 실행을완료했다. 원장173→177,명령포화4/4·원영상12,292프레임exact·두정책각조건부접촉2/2다. 독립담당의v23검토는진행중이며qualified/formal/hardware/entry2는0이다.

| v23재사용조건 | 기존→후보수직정지 | 감소/20%기준 | 기존→후보접촉오차 | 오차증가/+1cm기준 |
|---|---:|---|---:|---|
|571|10.78→6.76s|37.29%,통과|1.458→3.309cm|+1.850cm,실패|
|619|9.44→7.80s|17.37%,실패|.938→2.639cm|+1.701cm,실패|

수식수정과단독시험통과가실제정확도개선을보장하지않았다. 후보는채택하지않고기존정책을유지한다. 원실패와후보별표본을보존하며효능모집단성공률로합산하지않는다. [v23원계획](../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23/plan.json), [원결과](../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23/summary.json), [동일기준비교](../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23-continuity/summary.json)를사용한다. 실제하강속도의접촉직전값은명령포화와다르며571최소−.104m/s였다. 다음은마커소실전후의기체/차량속도추정과자세·중력보정잔차를시간정렬해분해한다. FF단독원인/실기검증으로단정하지않는다.

11:03KST독립검토추가: [v23실제보고서](reviews/20261009-v23-reference-relative-approach-independent-review.md)와final/index(11:00:07)를읽고 [main결합](../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23/main-independent-binding-validation.json)을완료했다. 네조건부접촉승인·원908/이전112/새226검토근거SHA불변이다. 수식CPU18fixture/원9접근시험재실행은확인됐지만전체228재실행은main범위다. 추정heading변화의forward_dot*standoff미분항은누락되며constant-heading한계도남는다. 후보거절·qualified/formal/hardware/entry2=0을유지한다. 말기기체속도오차분해의 [새접촉전기록](terminal-imu-budget-20261009.md)은기존6경로readonly자료로서새비행효능표본이아니다.
