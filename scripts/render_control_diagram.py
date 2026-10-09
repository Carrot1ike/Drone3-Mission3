#!/usr/bin/env python3
"""Render project-owned static control diagrams; no simulator is started."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/diagrams"
FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
NS = "http://www.w3.org/2000/svg"
ET.register_namespace("", NS)
INK, MUTED, LINE = "#183047", "#536679", "#687e91"
BLUE, GREEN, AMBER = "#eaf2fc", "#eaf4ed", "#fff5e2"


class Drawing:
    def __init__(self, width, height, title):
        self.image = Image.new("RGB", (width, height), "white")
        self.draw = ImageDraw.Draw(self.image)
        self.svg = ET.Element(f"{{{NS}}}svg", {
            "width": str(width), "height": str(height),
            "viewBox": f"0 0 {width} {height}", "role": "img",
        })
        ET.SubElement(self.svg, f"{{{NS}}}title").text = title
        self.rect(0, 0, width, height, "white", None, 0)

    def element(self, tag, **attrs):
        return ET.SubElement(self.svg, f"{{{NS}}}{tag}", {
            key.replace("_", "-"): str(value) for key, value in attrs.items()
        })

    def rect(self, x, y, w, h, fill, stroke=LINE, radius=12):
        self.draw.rounded_rectangle((x, y, x+w, y+h), radius,
                                    fill=fill, outline=stroke, width=2)
        self.element("rect", x=x, y=y, width=w, height=h, rx=radius,
                     fill=fill, stroke=stroke or "none", stroke_width=2)

    def text(self, x, y, content, size=24, color=INK, anchor="middle"):
        font = ImageFont.truetype(str(FONT), size, index=1)
        for n, line in enumerate(content.split("\n")):
            cy = y + n * (size + 12)
            bbox = self.draw.textbbox((0, 0), line, font=font)
            width = self.draw.textlength(line, font=font)
            left = x-width/2 if anchor == "middle" else x
            self.draw.text((left, cy-bbox[1]), line, fill=color, font=font)
            node = self.element("text", x=x, y=cy,
                                font_family="Noto Sans CJK KR, sans-serif",
                                font_size=size, fill=color, text_anchor=anchor,
                                dominant_baseline="text-before-edge")
            node.text = line

    def arrow(self, points, color=LINE, dashed=False):
        if dashed:
            for a, b in zip(points, points[1:]):
                length = math.dist(a, b)
                if length == 0:
                    continue
                for offset in range(0, math.ceil(length), 15):
                    end = min(offset+8, length)
                    start_point = tuple(a[i]+(b[i]-a[i])*offset/length for i in (0, 1))
                    end_point = tuple(a[i]+(b[i]-a[i])*end/length for i in (0, 1))
                    self.draw.line((start_point, end_point), fill=color, width=3)
        else:
            self.draw.line(points, fill=color, width=3)
        attrs = {"points": " ".join(f"{x},{y}" for x, y in points),
                 "fill": "none", "stroke": color, "stroke_width": 3,
                 "stroke_linejoin": "round"}
        if dashed:
            attrs["stroke_dasharray"] = "8 7"
        self.element("polyline", **attrs)
        (ax, ay), (bx, by) = points[-2:]
        angle = math.atan2(by-ay, bx-ax)
        head = [(bx, by)] + [
            (bx-13*math.cos(angle+s), by-13*math.sin(angle+s))
            for s in (-0.45, 0.45)
        ]
        self.draw.polygon(head, fill=color)
        self.element("polygon", points=" ".join(f"{x:.2f},{y:.2f}" for x, y in head),
                     fill=color)

    def node(self, x, y, w, h, title, detail="", fill="white", size=25):
        self.rect(x, y, w, h, fill)
        self.text(x+w/2, y+17, title, size)
        if detail:
            self.text(x+w/2, y+58, detail, 21, MUTED)

    def sum(self, x, y):
        self.draw.ellipse((x-29, y-29, x+29, y+29),
                          fill=BLUE, outline=LINE, width=2)
        self.element("circle", cx=x, cy=y, r=29, fill=BLUE,
                     stroke=LINE, stroke_width=2)
        self.text(x, y-14, "+", 36)

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.image.save(OUT / f"{name}.png")
        ET.ElementTree(self.svg).write(OUT / f"{name}.svg",
                                      encoding="utf-8", xml_declaration=True)
        print(OUT / f"{name}.png")


def overview():
    d = Drawing(1600, 1360, "이동 차량 착륙 제어 구조")
    d.text(55, 27, "이동 차량 착륙 · 제어 구조", 37, anchor="start")
    d.text(55, 88, "Crazyflie 2.1 + AI-deck + Flow deck  /  하드웨어 개조 없이 PC에서 인식·상위 제어", 25, anchor="start")
    d.text(55, 136, "사용자 실험 조건: 30 × 30 cm 착륙장 · 0.3 m/s 등속 직진 · 차량 앞 수직 ArUco", 23, MUTED, "start")
    d.rect(355, 218, 700, 1015, "#f6f9fe", "#c6d5e5", 14)
    d.rect(1110, 218, 445, 1015, "#f6faf7", "#c7d8cc", 14)
    d.text(705, 237, "PC · 상위 착륙 제어기 (설계 대상)", 26)
    d.text(1332, 237, "기체 · 기존 펌웨어", 26)

    # Physical feedback is routed outside the computing lanes.
    d.arrow([(1500, 1150), (1580, 1150), (1580, 199), (25, 199), (25, 345), (65, 345)])
    d.text(790, 172, "기체 운동 → 영상 변화", 18, MUTED)
    d.arrow([(1500, 1125), (1527, 1125), (1527, 472), (1500, 472)])

    d.node(65, 290, 220, 105, "AI-deck 카메라", "기본 장착 유지", BLUE, 24)
    d.node(65, 460, 220, 210, "인식·좌표 설정", "카메라 보정값\n마커 크기\n마커 → 착륙점\n실측 고정 변환", AMBER, 24)
    d.node(65, 745, 220, 180, "이동 차량", "앞쪽 수직 마커\n뒤쪽 착륙장\n마커 ≠ 착륙점", AMBER, 24)
    d.node(65, 1005, 220, 190, "기체 사양 기준", "제조사 자료만 사용\n하드웨어 버전 확인\n미공개 값은 미확정\n실험 조건과 구분", AMBER, 24)

    d.node(390, 290, 630, 105, "ArUco 인식", "지정 ID · 네 모서리 · 상대 자세", BLUE)
    d.node(390, 460, 630, 140, "좌표 변환 · 상태 추정", "착륙점 상대 위치·속도·방향\n영상 시각·지연 보상 · 관측 신뢰도", BLUE)
    d.node(390, 680, 630, 105, "단계 관리 · 목표 생성", "인식 → 정렬 → 접근 → 속도 맞추기 → 하강 → 접촉", BLUE)
    d.node(390, 850, 630, 140, "피드포워드 + 피드백", "차량 운동 예측 + 착륙점 오차 보정\n수평 추종 · 상대 높이 · 방향 목표", BLUE)
    d.node(390, 1060, 630, 125, "명령 조건 확인 · 제한", "좌표 기준·측정 나이·품질 확인\n관측 손실: 하강 보류 → 재획득 또는 중단", BLUE)
    d.node(390, 1245, 630, 85, "CFLib · 목표값 지속 갱신", "세계 속도 API는 일관된 세계 상태 추정이 있을 때", BLUE, 25)

    d.node(1150, 405, 350, 130, "IMU · Flow deck · ToF", "자세·회전 / 광류 / 거리\n바닥 ↔ 이동 상판: 기준 변화", GREEN)
    d.node(1150, 680, 350, 105, "기체 상태 추정기", "내부 센서 융합", GREEN)
    d.node(1150, 895, 350, 125, "설정된 내부 제어기", "속도 · 자세 · 각속도\n모터 출력 계산", GREEN)
    d.node(1150, 1095, 350, 100, "모터 · 기체 운동", "관측값으로 다시 돌아오는 피드백", GREEN, 25)

    d.arrow([(285, 345), (390, 345)])
    d.text(337, 306, "WiFi", 19, MUTED)
    d.arrow([(285, 500), (315, 500), (315, 373), (390, 373)], dashed=True)
    d.arrow([(285, 590), (390, 550)], dashed=True)
    d.arrow([(65, 810), (44, 810), (44, 374), (65, 374)], dashed=True)
    for y1, y2 in ((395, 460), (600, 680), (785, 850), (990, 1060), (1185, 1245)):
        d.arrow([(705, y1), (705, y2)])
    d.text(722, 801, "단계별 목표", 19, MUTED, "start")
    d.arrow([(390, 550), (370, 550), (370, 928), (390, 928)])
    d.text(410, 810, "추정 상태 피드백", 19, MUTED, "start")
    d.arrow([(1020, 575), (1039, 575), (1039, 1130), (1020, 1130)], dashed=True)

    d.arrow([(1325, 535), (1325, 680)])
    d.arrow([(1325, 785), (1325, 895)])
    d.text(1340, 822, "내부 피드백", 19, MUTED, "start")
    d.arrow([(1325, 1020), (1325, 1095)])
    d.arrow([(1150, 732), (1085, 732), (1085, 527), (1020, 527)])
    d.text(1081, 610, "상태\n로그", 18, MUTED)
    d.arrow([(1020, 1288), (1082, 1288), (1082, 982), (1150, 982)])
    d.text(1234, 1248, "실기체: Crazyradio / CRTP", 22, MUTED)
    d.text(1234, 1283, "CrazySim: UDP", 22, MUTED)
    d.save("control-overview")


def states():
    d = Drawing(1600, 570, "착륙 단계와 관측 손실 처리")
    d.text(55, 27, "착륙 단계 · 상태에 따라 전환", 36, anchor="start")
    d.text(55, 91, "인식 → 일자 정렬 → 접근 → 속도 맞추기 → 하강 → 접촉 확인", 26, MUTED, "start")
    nodes = [
        ("인식", "지정 마커\n안정적 관측"),
        ("일자 정렬", "방향·중심선\n오차 감소"),
        ("접근", "착륙점 부근\n높이 유지"),
        ("속도 맞추기", "수평 오차 +\n상대 속도 감소"),
        ("하강", "수평·방향\n추종 계속"),
        ("접촉 확인", "접촉 검증 후\n모터 종료"),
    ]
    for i, (title, detail) in enumerate(nodes):
        x = 55 + i*252
        d.node(x, 177, 230, 143, title, detail, BLUE)
        if i < len(nodes)-1:
            d.arrow([(x+230, 247), (x+252, 247)])
    d.node(560, 400, 500, 125, "재획득 또는 중단", "관측 복구 후 단계·목표를 다시 확인", AMBER)
    for i in (1, 2, 3, 4):
        x = 170 + i*252
        d.arrow([(x, 320), (x, 353), (810, 353), (810, 400)], dashed=True)
    d.text(810, 362, "마커 손실 · 오래된 영상 · 추정 기준 불일치 → 하강 보류", 22, MUTED)
    d.text(55, 537, "설계안: 단계별 오차·지속 시간·접촉 조건은 검증 후 확정", 21, MUTED, "start")
    d.save("landing-states")


def control_law():
    d = Drawing(1600, 940, "차량 운동 피드포워드와 위치 오차 피드백")
    d.text(55, 27, "수평 제어 · 피드포워드 + 피드백", 36, anchor="start")
    d.text(55, 91, "설명용 기준식: 기체·차량의 일관된 세계 위치·속도 추정이 확보된 경우", 25, MUTED, "start")
    d.node(55, 195, 400, 155, "동일 좌표계의 상태 추정", "기체 위치 · 차량 위치·속도\n마커 → 착륙점의 고정 변환", BLUE)
    d.node(55, 470, 400, 140, "단계별 수평 목표", "접근 지점 → 착륙장 중심\np_target", BLUE)
    d.node(590, 205, 440, 120, "차량 운동 예측 · 피드포워드", "v_ff = v_vehicle_hat", AMBER)
    d.node(590, 470, 210, 140, "위치 오차", "e = p_target\n− p_drone_hat", BLUE)
    d.node(845, 470, 210, 140, "피드백 보정", "v_correction\n= Kp × e", BLUE)
    d.sum(1130, 387)
    d.node(1200, 325, 345, 150, "속도 목표", "v_command\n= v_ff + v_correction\n좌표 기준 확인 · 목표 제한", GREEN, 25)
    d.node(1200, 700, 345, 125, "CFLib → 기체 내부 제어", "목표 추종 → 기체 운동", GREEN, 24)

    d.arrow([(455, 265), (590, 265)])
    d.text(522, 228, "차량 속도 추정", 18, MUTED)
    d.arrow([(255, 350), (255, 470)])
    d.arrow([(455, 540), (590, 540)])
    d.text(550, 502, "p_target", 19, MUTED)
    d.arrow([(455, 303), (503, 303), (503, 658), (695, 658), (695, 610)])
    d.text(660, 670, "p_drone_hat", 20, MUTED)
    d.arrow([(800, 540), (845, 540)])
    d.arrow([(1030, 265), (1130, 265), (1130, 358)])
    d.arrow([(1055, 540), (1130, 540), (1130, 416)])
    d.arrow([(1159, 387), (1200, 387)])
    d.arrow([(1372, 475), (1372, 700)])
    d.text(1372, 561, "지원되는 목표값으로 전송\n세계 속도 API 사용은 조건부", 21, MUTED)
    d.arrow([(1372, 825), (1372, 865), (27, 865), (27, 271), (55, 271)])
    d.text(715, 833, "영상·센서 관측 → 상태 추정 → 오차 보정", 23, MUTED)
    d.text(55, 903, "수직 하강·yaw 추종·접촉 판단은 단계 관리와 함께 실행합니다. 이 그림은 완성된 제어 코드가 아닙니다.", 22, MUTED, "start")
    d.save("control-law")


if __name__ == "__main__":
    if not FONT.is_file():
        raise SystemExit(f"한국어 글꼴이 없습니다: {FONT}")
    overview()
    states()
    control_law()
