#!/usr/bin/env python3
"""Meeting PDF from the same reviewed evidence and prose as the team report app."""
from pathlib import Path
import json, re, html, math
import xml.etree.ElementTree as ET
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Image, KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor, white
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.graphics.shapes import Drawing, Rect, Circle, Line, Polygon, String, Path as VectorPath

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/mission3/team-share-20261009'
content=json.loads((OUT/'report-content.json').read_text())
data=json.loads((OUT/'reviewed-results.json').read_text())
sections={s['id']:s for s in content['sections']}
for name,path in [('NanumGothic','/usr/share/fonts/truetype/nanum/NanumGothic.ttf'),('NanumGothicBold','/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf')]:
    pdfmetrics.registerFont(TTFont(name,path))
pdfmetrics.registerFontFamily('NanumGothic',normal='NanumGothic',bold='NanumGothicBold',italic='NanumGothic',boldItalic='NanumGothicBold')
INK=HexColor('#183348');TEAL=HexColor('#078099');MUTED=HexColor('#5d7283');LINE=HexColor('#d8e2e8')
styles={
    'body':ParagraphStyle('body',fontName='NanumGothic',fontSize=10,leading=16,textColor=INK,wordWrap='CJK',spaceAfter=9),
    'small':ParagraphStyle('small',fontName='NanumGothic',fontSize=8.4,leading=13,textColor=MUTED,wordWrap='CJK',spaceAfter=6),
    'title':ParagraphStyle('title',fontName='NanumGothicBold',fontSize=25,leading=34,textColor=INK,spaceAfter=18),
    'h1':ParagraphStyle('h1',fontName='NanumGothicBold',fontSize=18,leading=26,textColor=INK,spaceAfter=15),
    'h2':ParagraphStyle('h2',fontName='NanumGothicBold',fontSize=12,leading=18,textColor=TEAL,spaceBefore=11,spaceAfter=8),
    'cell':ParagraphStyle('cell',fontName='NanumGothic',fontSize=8.5,leading=12.5,textColor=INK,wordWrap='CJK'),
    'num':ParagraphStyle('num',fontName='NanumGothic',fontSize=9,leading=13,textColor=INK,alignment=2),
    'header':ParagraphStyle('header',fontName='NanumGothicBold',fontSize=8.5,leading=12,textColor=white,wordWrap='CJK'),
}
def markup(text):
    text=html.escape(text.replace('−','-'))
    text=re.sub(r'\*\*(.*?)\*\*',r'<b>\1</b>',text)
    text=re.sub(r'`([^`]+)`',r'<font color="#087d98">\1</font>',text)
    return text
def p(text,style='body'):return Paragraph(markup(text),styles[style])
def block(key):return [p(t) for t in sections[key]['body'].split('\n\n')]
def table(rows,widths,nums=(),failrows=()):
    rendered=[]
    for i,row in enumerate(rows):
        rendered.append([p(str(x),'header' if i==0 else 'num' if j in nums else 'cell') for j,x in enumerate(row)])
    t=Table(rendered,colWidths=widths,repeatRows=1,hAlign='LEFT')
    commands=[('BACKGROUND',(0,0),(-1,0),INK),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,0),(-1,-1),.4,LINE)]
    for i in range(1,len(rows)):
        if i%2==0:commands.append(('BACKGROUND',(0,i),(-1,i),HexColor('#f3f7fa')))
    for i in failrows:commands.append(('BACKGROUND',(0,i),(-1,i),HexColor('#fff0e9')))
    t.setStyle(TableStyle(commands));return t
def diagram(name,width=515):
    # Our authored SVG uses these explicit primitives; keep selectable vector labels.
    svg=ET.parse(OUT/'assets'/f'{name}.svg').getroot()
    d=Drawing(1100,520)
    def color(value):return None if not value or value=='none' else HexColor(value) if value.startswith('#') else white if value=='white' else INK
    for el in svg:
        tag=el.tag.split('}')[-1];a=el.attrib
        if tag in ('style','defs'):continue
        fill=color(a.get('fill'));stroke=color(a.get('stroke'));sw=float(a.get('stroke-width',1))
        common=dict(fillColor=fill,strokeColor=stroke,strokeWidth=sw)
        if tag=='rect':
            obj=Rect(float(a.get('x',0)),520-float(a.get('y',0))-float(a['height']),float(a['width']),float(a['height']),rx=float(a.get('rx',0)),ry=float(a.get('rx',0)),**common)
        elif tag=='circle':obj=Circle(float(a['cx']),520-float(a['cy']),float(a['r']),**common)
        elif tag=='line':
            x1,y1,x2,y2=float(a['x1']),520-float(a['y1']),float(a['x2']),520-float(a['y2'])
            obj=Line(x1,y1,x2,y2,strokeColor=stroke or INK,strokeWidth=sw)
            if a.get('stroke-dasharray'):obj.strokeDashArray=[5,4]
            if a.get('marker-end'):
                angle=math.atan2(y2-y1,x2-x1);dx,dy=math.cos(angle),math.sin(angle)
                d.add(Polygon([x2,y2,x2-8*dx+3*dy,y2-8*dy-3*dx,x2-8*dx-3*dy,y2-8*dy+3*dx],fillColor=stroke or INK,strokeColor=None))
        elif tag=='polygon':
            points=[float(v) for v in re.findall(r'[-\d.]+',a['points'])];obj=Polygon([v if i%2==0 else 520-v for i,v in enumerate(points)],**common)
        elif tag=='text':
            text=''.join(el.itertext()).replace('−','-');bold=a.get('class')=='b';size=float(a.get('font-size',21 if bold else 17))
            textfill=white if 'fill:white' in a.get('style','') else fill or INK
            obj=String(float(a['x']),520-float(a['y']),text,fontName='NanumGothicBold' if bold else 'NanumGothic',fontSize=size,fillColor=textfill,textAnchor=a.get('text-anchor','start'))
        elif tag=='path':
            obj=VectorPath(**common); tokens=re.findall(r'[MHV]|[-\d.]+',a['d']);i=0;x=y=0
            while i<len(tokens):
                cmd=tokens[i];i+=1
                if cmd=='M':x=float(tokens[i]);y=float(tokens[i+1]);i+=2;obj.moveTo(x,520-y)
                elif cmd=='H':x=float(tokens[i]);i+=1;obj.lineTo(x,520-y)
                elif cmd=='V':y=float(tokens[i]);i+=1;obj.lineTo(x,520-y)
        else:continue
        d.add(obj)
    scale=width/1100;d.scale(scale,scale);d.width=width;d.height=520*scale
    return d
def page(n,title):return [PageBreak(),p(f'{n:02d}  {title}','h1')]
story=[]
story += [p('DRONE3  /  MISSION 3  /  TEAM HANDOFF','small'),Spacer(1,18),p('미션3 개발·검증 자료','title'),p('최신 직선 트랙 16조건 전체','h2'),p('시험 종료: 2026-10-09 19:41 KST<br/>구조 분리 검증: 2026-10-09 20:11 KST'.replace('<br/>',' / '),'small')]
story += block('overview')
story += [Spacer(1,13),table([['공유 핵심','이번 기록'],['시나리오','30×5m 경기장 / 28m 직선 / 최고 0.30m/s'],['전체 조건','옆거리 1.25→0.50m, 0.05m 간격, 동일 seed 1001, 각 1회'],['결과','ID 인식 16 / 조건부 완료 14 / 실패 2'],['증거','제3자 1배속 16 + 기체 영상 16 / 원평가·로그·코드 SHA'],['해석','제조사 일치·실기·독립 승인 성공 0 / 성능 분모 16 유지']],[110,405]),Spacer(1,16),p('읽는 순서','h2'),p('02 환경 → 03 제어 계층 → 04 단계·센서 전환 → 05 전체 결과 → 06 실패·재현 검사 → 07 제조사 근거·한계 → 08 실행·공유 파일')]

story += page(2,'환경과 착륙 대상')
story += [diagram('arena',500)]
# Detailed source text is in MD/HTML; retain every fixed condition on the meeting pages.
story += [p('30×5m / 그물 5m / 실내 무풍. 차량 (1,2.5)→(29,2.5)m, 드론 (2,2.5−d,1)m. 북→동 20°는 동 기준 yaw 70°다. 호버 준비 후 출발하며 발견 전 고정 대기·시선을 유지한다. 회전 탐색이나 트랙 대기점 이동은 없다.'),p('차량 최고 0.30m/s. 출발·정지 가속 0.15m/s²는 S 시험 가정이다. 차량은 끝에서 정지한다. 미인식 화각 통과는 시험 장치 전용 참값 종료이며 제어 입력이 아니다.')]
story += [diagram('pad-marker',480),p('패드 20×20cm / 높이 30cm. 판 20×20cm / 코드 15×15cm. 패드 앞 가장자리→마커 면 15cm, 패드 중심→마커 중심 [앞 25cm, 위 10cm]. 판 바닥 z=0.30m, 중심 0.40m, 위 0.50m.','small')]

story += page(3,'상위 판단과 하위 제어')
story += [diagram('control-hierarchy',500)]
story += block('architecture')[:3]
story += [table([['책임','모듈'],['전역 관측·안전 검사 / 단계 선택','supervisor.py'],['대기·발견 / 정렬','acquisition_control.py / alignment_control.py'],['추종·접근 / 하강 / 복구','approach_control.py / landing_control.py / recovery_control.py'],['공통 입력·수식 / 명령 이력','guidance.py / command.py / command_limiter.py'],['하위 PID / 물리·접촉 연결','position_bridge.py / flight_controller.py / run_mission3_physical.py']],[180,335])]
story += [p('계층형 FSM 스타일이다. SMACC2/행동 트리는 레퍼런스이며 해당 라이브러리나 전체 Statecharts 의미론을 채택한 것은 아니다. 독립 시험 최대 4개 병렬화와 한 비행의 시간 순서는 별개다.','small')]

story += page(4,'단계·피드포워드·센서 전환')
story += [table([['단계','진행 조건·동작']]+content['phases'],[178,337])]
story += [p('차량 속도 FF + 상대 오차 FB','h2'),p('개념식: v_cmd = 추정 차량 속도 + Kp × 상대 위치 오차 + Kv × 상대 속도 오차. 정적 마커-패드 치수는 사용하지만 차량 실제 상태와 설정 속도 0.30m/s는 제어 입력에 쓰지 않는다.'),p('상위 명령 50Hz / PC 속도 PID·RPYT 100Hz / 고정 공식 자세·각속도 PID 경로. 단계 전환 시 PID·추정 상태와 공통 제한기의 속도/yaw 이력을 유지한다.'),p('정상 명령: 속력 0.40m/s · 가속 0.15m/s² · yaw 10°/s · yaw 가속 20°/s² · 상승 0.10m/s · 하강 0.06m/s. 말기 지원 하강 0.025m/s. 안전 override는 별도 기록.'),p('Flow/ToF가 패드 위로 넘어갈 때','h2'),p('Flow는 관측 표면 기준 상대 운동이고 ToF는 아래 표면까지의 거리다. z=1m를 유지해도 z=0.30m 패드 위에서 거리만 1.0→0.7m로 바뀔 수 있다. 바닥/패드/무효 표면과 센서 시각·품질을 구분하고 근접 무효 구간에는 제한된 IMU 예측을 사용한다. [2]')]

story += page(5,'직선 트랙 16회 전체 결과')
story += [p('동일 seed 1001 / 각 거리 1회 / ID 인식 16/16 / 조건부 완료 14/16. 완료는 실제 움직이는 패드 접촉·PWM 0·직선 지지 3초의 조합이며 근접 정지는 성공이 아니다.')]
rows=[['옆거리 m','ID','완료','접촉 오차 cm','내부 점수','비고']]
for r in data['rows']:
    note='관성 예측 만료' if r['distance_m']==1.15 else '인식 후 영상 소실' if r['distance_m']==.5 else '—'
    rows.append([f"{r['distance_m']:.2f}",'있음','완료' if r['contact_complete'] else '실패','—' if r['contact_error_cm'] is None else f"{r['contact_error_cm']:.3f}",'—' if r['points'] is None else r['points'],note])
story += [table(rows,[61,40,50,94,70,200],nums=(0,3,4),failrows=(3,16)),Spacer(1,12),p('접촉 오차는 조건부 기체 CoM/body 원점 기준. 내부 점수는 1cm 미만 10점, 다음 1cm마다 1점 감소, 10cm 이상 0점이다. 대회 공식 점수가 아니다. 실패 조건의 접촉 점수도 완료로 세지 않는다.','small'),p('16회 정상 명령 포화 PASS. native 11,084프레임의 FFV1/입력 SHA 일치. 실제 최고 속력의 조건별 범위는 0.35623~0.42569m/s여서 엄격한 실제 0.40m/s 상한을 검증한 것은 아니다.','small'),p('영상 32개와 원평가: videos.html. 제3자는 1배속, 기체는 고정 20FPS 기록 프레임 미리보기다. 1.15m의 약 7.2초 기록 공백 때문에 로그 t로 시간 정렬해야 한다. 무손실 원픽셀·원로그는 별도 전체 원자료 ZIP.','small')]

story += page(6,'실패·진행 시각·구조 변경 검증')
story += [p('실패 두 조건','h2')]+block('failures')[:2]
story += [p('1.25m 조건의 진행 시각','h2'),table([['실행 시작 기준 시각 s','기록된 단계'],['9.76','센서 호버 준비 / 차량 출발'],['15.12 / 20.04','첫 영상 guidance / LINEUP 전환'],['27.00 / 35.32 / 40.22','높이 준비 / APPROACH / TRACK'],['42.22 / 45.52 / 45.72','BLIND_APPROACH / BLIND_DESCEND / 지원 하강'],['49.26 / 50.14 / 53.18','첫 실제 접촉 / PWM 0 / 실행 종료']],[145,370]),p('시각은 d125-r01 원로그의 한 실행 예시다. 모든 조건의 공통 전환 시각을 뜻하지 않는다.','small'),p('현재 구조로 바꾼 뒤의 재현 검사','h2')]+block('validation')

story += page(7,'제조사 근거와 현실 차이')
story += [p('P 제조사 공개값 / C 사용자 확정 / D 계산 / S 시험 가정 / U 미확정','h2'),table([['항목','확정 내용과 검증 한계'],['C 입력·운영','Crazyflie 2.1 + AI-deck + Flow deck, 하드웨어 개조 금지, PC 계산, native mono 320×320. 실기/Crazyradio 미보유.'],['P 카메라 모듈 [1]','HM01B0 모노 유효 320×320, 롤링 셔터, 공칭 H/V 87°·D 115°, 초점 35cm. 지급 revision과 실제 보정은 U.'],['P Flow deck [2]','PMW3901 광류 + VL53L1x ToF / 공개 거리 최대 약 4m. 표면·광량·근접 상태의 정확도 보장과 구별.'],['D 대상 기하','마커 중심에서 패드 중심까지 정적 치수 변환. 공칭 화각은 가시성 후보이며 인식 보장이 아님.'],['S/U 센서·동역학','K/D, 기체-카메라/Flow 장착, 롤링 셔터·전송 지연, 질량/관성, 추력·모터·공력, 근접 센서와 접촉/마찰 미확정.'],['기록 검증과 현실','FFV1 픽셀 SHA 정확 일치는 기록 충실도다. 센서 현실성·제조사 일치·실기 성능의 검증과 다르다.']],[120,395])]
story += [p('공식 예제 창과 지급 영상 조건','h2'),p('공식 AI-deck 예제의 324×244/324×324 창 설정과 사용자 확정 320×320 원입력 사이의 펌웨어 창·오프셋·시각을 확인해야 한다. 실제 지급 덱과 모듈 revision은 장비 지급 후 대조한다. 현재 렌더·센서·동역학은 조건부이며 다른 해상도로 바꿔 성능을 맞추지 않았다.'),p('공식 문서·구조 레퍼런스','h2')]
for i,(title,url,note) in enumerate(content['references'],1):
    label=f'[{i}] {title}'
    story.append(Paragraph(f'<link href="{html.escape(url)}" color="#078099">{html.escape(label)}</link> — {html.escape(note)}',styles['small']))

story += page(8,'팀 공유 파일·실행·다음 작업')
story += [table([['파일·폴더','용도'],['미션3_팀공유.pdf / .md','회의용 보고서와 편집용 원문'],['team-report.html / videos.html','검색 가능한 보고서 / 16조건·32영상 비교'],['all-16-results.csv / reviewed-results.json','숫자·단위·null과 원자료 SHA'],['evidence/ / verification/','원평가·배치 계획 / 구조 변경 재현 검사'],['code-executed-batch/','16회 실행 당시 동결 코드·runtime'],['code-current-hierarchy/ / config/','현재 구조 코드 / 최신 시나리오·센서 프로필'],['별도 최신 원자료 ZIP','전체 JSONL·FFV1·동결 소스·평가, 원경로 보존']],[220,295]),p('확인과 실행','h2')]+block('usage')[:3]
story += [p('우선순위','h2'),p('1. 실패 두 조건의 영상·센서·상대 속도 오차를 분해한다. 2. 실제 속도 overshoot 보호와 말기 연속 접근·착지 정확도를 함께 시험한다. 3. 새 seed 반복/holdout을 별도 사전 계획한다. 4. 장비 지급 후 카메라·센서·장착·조립 동역학·통신 지연을 보정한다.'),p('이 공유 작업은 새 비행 시험·외부 업로드·팀원 전송을 수행하지 않았다. 과거 스타디움 결과는 이 자료의 결과 표와 원자료 ZIP에 포함하지 않았다.','small')]

def footer(c,doc):
    w,h=A4;c.saveState();c.setStrokeColor(LINE);c.line(40,40,w-40,40)
    c.setFont('NanumGothic',8);c.setFillColor(MUTED);c.drawString(40,27,'Drone3 · 미션3 / 최신 직선 16조건 / 2026-10-09')
    c.drawRightString(w-40,27,str(doc.page));c.restoreState()

pdf=OUT/'미션3_팀공유.pdf'
doc=SimpleDocTemplate(str(pdf),pagesize=A4,rightMargin=40,leftMargin=40,topMargin=43,bottomMargin=53,title='Drone3 미션3 팀 공유 · 최신 직선 16조건',author='Drone3 프로젝트',subject='제어 구조·센서·최신 직선 시험 전체·재현 자료')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(pdf)
