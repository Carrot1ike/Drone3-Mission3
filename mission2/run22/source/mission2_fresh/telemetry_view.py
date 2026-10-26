"""Read-only shared live/video compositor. No flight commands or truth feedback."""
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .passages import level_image,F,CX,CY
from .exit_track import heading_image

W,H=1600,960
FONT='/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf'
FONTS={}
BG='#0c1420';FG='#e6eef7';MUTED='#a3b2c6';GREEN='#5ce1b5';AMBER='#ffcc77';BLUE='#78b8ff'
NAMES={'TAKEOFF':'이륙','ENTRY_FORWARD':'입구 준비 전진','FIND':'링 탐색','ALIGN':'링 정렬',
'SCAN_BRAKE':'출입구 후 정지','SCAN':'좌우 45° 스캔',
'EXIT_SCAN_BRAKE':'출구 관측 준비','EXIT_SCAN':'출구 기둥 좌우 관측',
'WALL_SCAN_BRAKE':'벽 앞 정지','WALL_SCAN':'벽 통로 좌우 45도 스캔',
'STRAIGHT':'링 직진 통과','BRAKE':'감속','POST_RING_DESCENT':'블록 구간 높이로 하강',
'BLOCK_GAP':'블록 사이 정렬','BLOCK_STRAIGHT':'블록 통과','WALL_1':'첫 번째 벽',
'WALL_2':'두 번째 벽','WALL_STRAIGHT':'선택한 벽 통로 직진','EXIT':'출구 정렬',
'EXIT_STRAIGHT':'출구 통과','LAND':'착륙 하강','DONE':'명령 종료'}


def font(n):
    if n not in FONTS:FONTS[n]=ImageFont.truetype(FONT,n)
    return FONTS[n]


def render(external,image,d,command,run_label,elapsed,age,fps,ended=False,note=''):
    canvas=Image.new('RGB',(W,H),BG);draw=ImageDraw.Draw(canvas)
    def text(x,y,s,n=21,color=FG):draw.text((x,y),str(s),font=font(n),fill=color)
    def line(x,y,s,n=20,color=MUTED,width=485):
        row=''
        for ch in str(s):
            if ch=='\n' or draw.textlength(row+ch,font=font(n))>width:
                text(x,y,row,n,color);y+=29;row='' if ch=='\n' else ch
            else:row+=ch
        text(x,y,row,n,color);return y+29
    stage=d.get('stage','CONNECTING');state=d.get('frame',{}).get('state') or {}
    decision=d.get('decision') or {};active=decision.get('perception_active',True)
    passage=d.get('passage') or {};target=d.get('target');sent=(command or {}).get('command') or {}
    text(18,12,f"{run_label}   {elapsed:6.1f}s   {NAMES.get(stage,stage)}",29,GREEN)
    text(1025,19,'종료된 기록' if ended else f'LIVE   입력 {fps:.1f} fps  |  영상 나이 {age:.2f}s',21,AMBER)
    for x in [12,544,1076]:draw.rounded_rectangle((x,61,x+512,475),10,fill='#172333')
    text(25,65,'외부 시점 · 기록된 실제 자세 재현',20)
    text(557,65,'제어에 사용한 기체 영상',20)
    text(1089,65,'검출 · 바닥 특징점 · 판단 목표',20)
    if external is not None:
        ex=Image.fromarray(cv2.cvtColor(external,cv2.COLOR_BGR2RGB));ex.thumbnail((512,385))
        canvas.paste(ex,(12+(512-ex.width)//2,96+(385-ex.height)//2))
    if image is not None and state:
        is_passage=stage in ('BLOCK_GAP','WALL_1','WALL_2','EXIT','EXIT_SCAN','WALL_SCAN')
        view=(heading_image(image,state) if stage=='WALL_SCAN' or stage in ('EXIT','EXIT_SCAN') and decision.get('exit_width_prior_m')
              else level_image(image,state) if is_passage else image.copy())
        raw=cv2.cvtColor(view,cv2.COLOR_GRAY2BGR);diag=raw.copy();diag=(diag*.38).astype(np.uint8)
        if stage=='SCAN':
            for candidate in decision.get('ring_candidates',[]):
                u,v=candidate['center'];radius=candidate['diameter']/2
                cv2.circle(raw,(round(u),round(v)),round(radius),(255,190,70),1)
        if target and active:
            u,v=target['center'];radius=target['diameter']/2
            color=(90,240,110) if target.get('observation_source','current_image')=='current_image' else (80,190,255)
            for im in [raw,diag]:
                cv2.circle(im,(round(u),round(v)),round(radius),color,1)
                cv2.circle(im,(round(u),round(v)),3,(80,235,255),-1)
                cv2.line(im,(round(CX),round(CY)),(round(u),round(v)),(255,160,80),1)
        if is_passage and active:
            if stage=='WALL_SCAN':
                wall_scan=decision.get('wall_scan') or {}
                for f in wall_scan.get('current_faces',[]):
                    x,y,w,h=f['box']
                    for im in [raw,diag]:cv2.rectangle(im,(x,y),(x+w,y+h),(95,230,120),1)
            if stage=='EXIT_SCAN' and not passage:
                for f in (decision.get('exit_scan') or {}).get('current_faces',[]):
                    x,y,w,h=f['box']
                    for im in [raw,diag]:cv2.rectangle(im,(x,y),(x+w,y+h),(95,230,120),1)
            for f in passage.get('all_faces',passage.get('faces',[])):
                x,y,w,h=f['box'];cv2.rectangle(diag,(x,y),(x+w,y+h),(105,105,105),1)
            for f in passage.get('faces',[])+passage.get('side_faces',[]):
                x,y,w,h=f['box'];cv2.rectangle(raw,(x,y),(x+w,y+h),(95,230,120),1)
                region=diag[y:y+h,x:x+w];region[:]=cv2.addWeighted(region,.3,raw[y:y+h,x:x+w],.7,0)
                cv2.rectangle(diag,(x,y),(x+w,y+h),(95,230,120),1)
                edge=f.get('floor_edge')
                if edge:
                    a,b=[tuple(np.rint(p).astype(int)) for p in edge]
                    cv2.line(diag,a,b,(80,210,255),2);cv2.line(raw,a,b,(80,210,255),1)
            if passage:
                u=round(CX-F*passage['left']/passage['depth'])
                for im in [raw,diag]:
                    cv2.line(im,(u,15),(u,235),(255,145,75),1)
                    cv2.circle(im,(u,round(CY)),4,(255,145,75),1)
            for choice in passage.get('choices',[]):
                if choice.get('valid') and not choice.get('historical'):
                    u=choice['edge_u'];cv2.line(diag,(u,40),(u,225),(0,190,255),1)
                    if 'floor_end_u' in choice:
                        row=choice['floor_row']
                        for im in [raw,diag]:cv2.line(im,(u,row),(choice['floor_end_u'],row),(210,230,90),3)
        # Motion tracks are on attitude-corrected images; label and display there.
        motion=decision.get('floor_motion') or {}
        if not is_passage:
            # Transform the annotation together with the image; ground tracks
            # are already in this attitude-corrected view.
            diag=level_image(raw,state)
            diag=(diag*.55).astype(np.uint8)
        for a,b,c,e in motion.get('tracks',[]):
            cv2.line(diag,(round(a),round(b)),(round(c),round(e)),(255,210,90),1)
            cv2.circle(diag,(round(c),round(e)),1,(255,220,120),-1)
        for im in [raw,diag]:cv2.drawMarker(im,(round(CX),round(CY)),(180,180,180),cv2.MARKER_CROSS,10,1)
        for x,im in [(544,raw),(1076,diag)]:
            canvas.paste(Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB)).resize((512,386)),(x,96))
    draw=ImageDraw.Draw(canvas)
    for x in [12,544,1076]:draw.rounded_rectangle((x,500,x+512,820),10,fill='#172333')
    text(25,510,'인지 · 현재 관측',23,GREEN)
    y=548
    text(25,y,f"링 {d.get('ring_index',0)}/3   벽 {decision.get('wall_index',0)}/2   {d.get('passage_phase','')}",21);y+=31
    if target:
        y=line(25,y,f"링 거리 {target['depth']:.2f} m / 좌우 {target['left']:+.2f} m / 상하 {target['up']:+.2f} m")
        predicted=target.get('observation_source')=='ego_prediction'
        y=line(25,y,'주황: 이전 관측 + 기체 이동량으로 추정한 링' if predicted else '초록: 현재 영상에서 관측한 링',20,AMBER if predicted else GREEN)
        if predicted:
            seconds=(decision.get('ring_tracking') or {}).get('last_observation_age_s',0)
            y=line(25,y,f'마지막 관측 {seconds:.2f}초 전 · 현재 검출/통과 증거 아님',19,AMBER)
    elif stage=='WALL_SCAN':
        ws=decision.get('wall_scan') or {}
        y=line(25,y,f"현재 벽 면 후보 {len(ws.get('current_faces',[]))}개",21,GREEN)
        y=line(25,y,'회전 영상의 벽 바닥선과 옆 바닥을 같은 로컬 좌표로 연결',20)
        y=line(25,y,'스캔이 끝난 뒤 관측 폭 비교 · 전진 명령 0',20,BLUE)
    elif stage=='EXIT_SCAN':
        scan_exit=decision.get('exit_scan') or {}
        y=line(25,y,f"현재 기둥 {scan_exit.get('current_posts',0)}개 / 누적 {len(scan_exit.get('posts',[]))}개",21,GREEN)
        y=line(25,y,f"허용된 출구 폭 {decision.get('exit_width_prior_m',0):.2f}m와 관측 간격 비교",20)
        y=line(25,y,'기둥의 일정한 밝기와 바닥 접점 검출 → 회전 전후 관측 연결',19)
        if passage:y=line(25,y,f"출구 중앙 추정: 전방 {passage['depth']:.2f}m / 좌측 {passage['left']:+.2f}m",19,BLUE)
    elif stage=='SCAN':
        candidates=decision.get('ring_candidates',[])
        y=line(25,y,f"현재 영상의 링 후보 {len(candidates)}개",21,GREEN)
        y=line(25,y,'영상의 원 테두리와 안팎 밝기 차이를 검사',19)
        y=line(25,y,'링 규격으로 거리 추정 → 기체 자세·이동량으로 좌우 관측 연결',19)
        y=line(25,y,'파랑 원: 이번 영상의 후보 / 오른쪽 목록: 스캔 중 누적 후보',19,BLUE)
    elif passage and active:
        y=line(25,y,f"거리 {passage['depth']:.2f} m / 목표 좌우 {passage['left']:+.2f} m")
        if 'gap_m' in passage:
            label='관측 바닥 폭' if passage.get('method')=='observed_floor_strip' else '추정 통로 폭'
            y=line(25,y,f"{label} {passage['gap_m']:.2f} m / 관측한 끝 {passage['rear_depth']:.2f} m")
        y=line(25,y,passage.get('reason',''))
        if passage.get('kind')=='wall' and not passage.get('rear_observed'):
            y=line(25,y,f"후면 미관측: 관측면 + 전진 여유 {decision.get('clearance_margin_m',.45):.2f}m",19,AMBER)
    else:
        y=line(25,y,'현재 장애물 인식 갱신 없음. 아래 직진 값은 진입 때 확정한 값.' if 'STRAIGHT' in stage else '현재 단계의 목표 관측 대기 또는 준비 동작',20,AMBER)
    line(25,max(y,750),'주황: 거리 계산에 쓴 바닥선\n파랑 세로선: 정렬 목표',19)
    text(557,510,'판단 · 방향 선택 근거',23,GREEN);y=550
    choices=passage.get('choices',[]) if active else []
    for side in [1,-1]:
        c=next((c for c in choices if c['side']==side),None)
        if c:
            label=f"관측 폭 {c['gap_m']:.2f}m"+(' (화면 끝에서 잘림)' if c.get('clipped') else '') if c.get('valid') else c['reason']
            text(557,y,('왼쪽' if side==1 else '오른쪽')+' 후보: '+label,21);y+=31
    if passage.get('side') and active:
        text(557,y,'선택: '+('왼쪽' if passage['side']==1 else '오른쪽')+' / 관측 폭이 큰 후보',22,AMBER);y+=34
    checks=decision.get('checks') or {}
    translations={'full':'링 전체 보임','lateral':'좌우 오차 < 9cm','vertical':'높이 오차 < 9cm',
       'depth':'정렬 거리 도달','velocity':'잔류 속도 감소','sideways_stopped':'좌우 속도 감소','forward_stopped':'전진 속도 감소'}
    for key,value in checks.items():
        text(557,y,('✓ ' if value else '· ')+translations.get(key,key),19,GREEN if value else AMBER);y+=26
    if checks:text(557,y,f"조건 유지 {decision.get('stable_s',0):.2f}s",19);y+=26
    if 'STRAIGHT' in stage:
        text(557,y,f"이동 {d.get('travel_m',0):.2f} / {d.get('straight_distance_m',0):.2f} m",24,BLUE);y+=36
        y=line(557,y,'정렬 때 확정한 방향과 높이로 직진')
    scan=(decision.get('wall_scan') if stage=='WALL_SCAN' else decision.get('exit_scan') if stage=='EXIT_SCAN' else decision.get('ring_scan')) or {}
    if stage in ('SCAN','EXIT_SCAN','WALL_SCAN') and scan:
        y=line(557,y,f"현재 {scan['relative_yaw_deg']:+.1f}° / 목표 {scan['target_offset_deg']:+.0f}°",22,BLUE)
        if stage=='WALL_SCAN':
            for choice in scan.get('choices',[]):
                y=line(557,y,f"{'왼쪽' if choice['side']==1 else '오른쪽'} 관측 폭 {choice['gap_m']:.2f}m / {choice['samples']}회",19)
        for candidate in scan.get('candidates',[])[:4]:
            y=line(557,y,f"후보 {candidate['id']}: 추정 {candidate['distance_m']:.2f}m · 관측 {candidate['samples']}회",19)
        y=line(557,y,'좌45° → 우45° → 정면 복귀 · 벽 통로 방향 결정' if stage=='WALL_SCAN' else '좌45° → 우45° → 정면 복귀 · 출구 폭과 맞는 기둥 쌍 선택' if stage=='EXIT_SCAN'
               else '좌45° → 우45° → 정면 복귀 후 가장 가까운 관측 후보 선택',18)
    if not choices and 'STRAIGHT' not in stage and stage not in ('SCAN','EXIT_SCAN','WALL_SCAN'):
        reason=decision.get('ring_selection_reason')
        if reason:line(557,y,reason,18)
        elif not checks:line(557,y,NAMES.get(stage,stage)+' 단계 실행 중')
    text(1089,510,'상태 추정 · 사용하는 사전정보',23,GREEN)
    motion=decision.get('floor_motion') or {};position=decision.get('arena_left_position')
    text(1089,551,f"바닥 추적점 {motion.get('inliers',0)}개 / {motion.get('source','기록 없음')}",20)
    if position is not None:text(1089,582,f"경기장 중앙 기준 좌측 위치 {position:+.2f}m (추정)",20)
    if decision.get('minimal_priors'):
        width=decision.get('exit_width_prior_m')
        line(1089,582,(f'규격: 링 + 출구 폭 {width:.2f}m / 기체 센서 사용\n지도·경기장 크기·위치 입력 없음' if width
                       else '규격 사전값: 링만 / 카메라·기체 센서 사용\n지도·출구 폭·경기장 크기·위치 입력 없음'),19)
        line(1089,636,'남은 가정: 구간 순서·개수, 입구 1.5m 전진\n바닥점 추적은 표시용; 벽 방향에 사용 안 함',19)
        ring_scan=decision.get('ring_scan') or {}
        if ring_scan.get('selected_id'):
            line(1089,697,f"초기 스캔: 후보 {ring_scan['selected_id']} / {ring_scan['selected_distance_m']:.2f}m",17,AMBER)
        line(1089,724,'벽 판단: 보이는 바닥 ≠ 그물 없는 공간',17,AMBER)
    else:
        line(1089,615,'경기장 폭 5m + 시작 위치 + 바닥 영상 이동\n블록·벽 치수와 통과 방향 사전 입력 없음',19)
        line(1089,685,'링 규격·출구 폭·카메라 사양 사용\n펜스/그물 모양은 위치 추정에 사용하지 않음',19)
    line(1089,759,'계산: 초기 방향의 로컬 좌표\n전송: 기체 기준 vx·vy / EKF 목표 Z',19,BLUE)
    draw.rounded_rectangle((12,838,1588,946),10,fill='#172333')
    text(25,849,f"실제 전송  전방 {sent.get('vx',0):+.3f} m/s    좌측 {sent.get('vy',0):+.3f} m/s    yaw {sent.get('yawrate',0):+.1f} °/s    목표 Z {sent.get('height_m',0):.3f} m",24,BLUE)
    tof_status=(f"링 ToF 제외 요청 {sent.get('ring_pass_mask',0)} / ACK {(command or {}).get('ring_pass_ack',0)}"
        if 'ring_pass_mask' in sent else 'ToF 융합 제외 요청 없음')
    text(25,884,f"EKF 높이 {state.get('ekf_height_m',0):.3f} m   ToF {state.get('tof_distance_m',0):.3f} m   {tof_status}",21)
    text(25,916,note or '원본 cflib → hoverDecoder → EKF/PID  |  외부 자세는 표시·독립 평가용이며 제어 입력으로 사용하지 않음',17,MUTED)
    return cv2.cvtColor(np.array(canvas),cv2.COLOR_RGB2BGR)
