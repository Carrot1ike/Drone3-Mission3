"""Image geometry with no block/wall size, gap-width or direction priors.

Metric scale comes from the associated camera height and visible floor contacts.
All passage widths come from the current image; there is no arena origin or map.
"""
import cv2
import numpy as np
from .vision import F, CX, CY, attitude



def level_image(image, state):
    rotation, _ = attitude(state)
    pixel_to_body = np.array([[0., 0., 1.], [-1/F, 0., CX/F], [0., -1/F, CY/F]])
    homography = np.linalg.inv(pixel_to_body) @ rotation @ pixel_to_body
    return cv2.warpPerspective(image, homography, (324, 244))


def solid_faces(image, state):
    gray = level_image(image, state)
    histogram = np.bincount(gray[15:115].ravel(), minlength=256).astype(float)
    histogram[:32] = 0
    peaks = []
    for _ in range(6):
        peak = int(np.argmax(histogram))
        if histogram[peak] < 65:
            break
        peaks.append(peak)
        histogram[max(0, peak-9):peak+10] = 0
    result = []
    for peak in peaks:
        mask = (np.abs(gray.astype(np.int16)-peak) < 5).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        for label, (x, y, w, h, area) in enumerate(stats[1:], 1):
            if w < 6 or h < 35 or area < .72*w*h or y > CY-12 or y+h < CY+12:
                continue
            points = []
            for u in range(x+1, x+w-1):
                vv = np.flatnonzero(labels[:, u] == label)
                if len(vv) and CY+12 < vv[-1] < 240:
                    points.append((u, vv[-1]+1))
            face = dict(box=[int(x), int(y), int(w), int(h)], gray=int(peak),
                        depth=None, depth_method='floor_not_visible')
            if len(points) >= 4:
                pp = np.array(points, dtype=float)
                slope, intercept = np.polyfit(pp[:, 0], pp[:, 1], 1)
                residual = np.abs(pp[:, 1] - (slope*pp[:, 0]+intercept))
                pp = pp[residual < max(2., 3*np.median(residual))]
                if len(pp) >= 4:
                    slope, intercept = np.polyfit(pp[:, 0], pp[:, 1], 1)
                    us = np.array([x, x+w], dtype=float)
                    vs = slope*us+intercept
                    if np.all((vs > CY+12) & (vs < 241)):
                        depths = F*state['ekf_height_m']/(vs-CY)
                        lefts = (CX-us)*depths/F
                        face.update(depth=float(np.median(depths)), depth_method='floor',
                                    floor_edge=np.column_stack([us,vs]).tolist(),
                                    ground_edge=np.column_stack([depths,lefts]).tolist(),
                                    depth_span=float(np.ptp(depths)),
                                    near_depth=float(min(depths)), far_depth=float(max(depths)),
                                    measured_width=float(abs(lefts[1]-lefts[0])))
            result.append(face)
    return sorted(result, key=lambda f: f['box'][0])


def front_faces(faces):
    # A front face has a nearly constant ground depth; a side has a sloping floor line.
    return [f for f in faces if f['depth'] is not None
            and f['depth_span'] < .20*f['depth']
            and f['box'][2] > 10]


def rear_extent(front, faces):
    """Observed adjacent side endpoints, not a configured object length."""
    x,_,w,_ = front['box']; sides = []
    for face in faces:
        if face is front or face['depth'] is None:
            continue
        fx,_,fw,_ = face['box']
        adjacent = min(abs(fx-x-w), abs(fx+fw-x)) <= 3
        if adjacent and abs(face['near_depth']-front['depth']) < .25*front['depth']:
            if face['far_depth'] > front['far_depth'] and face['depth_span'] > .08:
                sides.append(face)
    rear = max([front['far_depth']] + [f['far_depth'] for f in sides])
    return float(rear), sides


def blocks(image, state):
    faces = solid_faces(image, state); fronts = front_faces(faces); pairs = []
    for i,a in enumerate(fronts):
        ax,ay,aw,ah = a['box']
        for b in fronts[i+1:]:
            bx,by,bw,bh = b['box']
            if bx-ax-aw < 16 or abs(a['gray']-b['gray']) > 8 or min(ah,bh) < 60:
                continue
            if abs(a['depth']-b['depth']) > .20*min(a['depth'],b['depth']):
                continue
            left_edge = a['ground_edge'][1][1]; right_edge = b['ground_edge'][0][1]
            if left_edge <= right_edge:
                continue
            rear_a, sides_a = rear_extent(a,faces); rear_b, sides_b = rear_extent(b,faces)
            # The exit of a thick pair must be observed; do not substitute a length.
            if not sides_a and not sides_b:
                continue
            depth = float((a['depth']+b['depth'])/2)
            pairs.append(dict(depth=depth, left=float((left_edge+right_edge)/2),
                              gap_m=float(left_edge-right_edge), rear_depth=max(rear_a,rear_b),
                              rear_observed=True, faces=[a,b], side_faces=sides_a+sides_b,
                              all_faces=faces, kind='blocks', method='floor_edges',
                              reason='양쪽 앞면의 바닥선으로 틈 중앙, 옆면 끝으로 통과 거리 계산'))
    return min(pairs, key=lambda p:p['depth']) if pairs else None


def wall(image, state):
    faces=solid_faces(image,state)
    fronts=[f for f in front_faces(faces) if f['box'][2]>=25]
    if not fronts:return None
    face=min(fronts,key=lambda f:f['depth'])
    x,y,w,h=face['box'];depth=face['depth'];gray=level_image(image,state)
    base=round(CY+F*state['ekf_height_m']/depth)
    lo=max(0,base-10);hi=min(244,base-2)
    if hi<=lo:return None
    # Visible ground just beyond the obstacle plane. This is an appearance
    # observation, not certified collision-free space or a net detector.
    mean=cv2.blur(gray.astype(float),(7,7))
    floor=(mean>30).astype(np.uint8)
    for other in faces:
        if other is face:continue
        fx,fy,fw,fh=other['box']
        floor[fy:fy+fh,fx:fx+fw]=0
    profile=np.convolve(floor[lo:hi].mean(axis=0),np.ones(7)/7,'same')>.45
    choices=[]
    for side,edge in ((1,x),(-1,x+w)):
        if not 3<edge<321:
            choices.append(dict(side=side,valid=False,reason='벽 끝이 화면 밖',edge_u=int(edge)))
            continue
        columns=range(edge-4,4,-1) if side==1 else range(edge+4,320)
        missed=0;visible=[]
        for u in columns:
            missed=0 if profile[u] else missed+1
            if missed>=6:break
            if profile[u]:visible.append(u)
        if len(visible)<7:
            choices.append(dict(side=side,valid=False,reason='이어지는 바닥 영역 미관측',edge_u=int(edge)))
            continue
        end=visible[-1];gap=float(abs(end-edge)*depth/F)
        choices.append(dict(side=side,valid=True,edge_u=int(edge),floor_end_u=int(end),
            floor_row=int((lo+hi)/2),gap_m=gap,left=float((CX-(edge+end)/2)*depth/F),
            clipped=bool(end<=6 or end>=318),reason='벽 끝 옆에서 연속 관측된 바닥 폭'))
    valid=[c for c in choices if c['valid']]
    if not valid:return None
    selected=max(valid,key=lambda c:c['gap_m']);rear,sides=rear_extent(face,faces)
    return dict(depth=depth,left=selected['left'],edge_u=selected['edge_u'],side=selected['side'],
        gap_m=selected['gap_m'],choices=choices,rear_depth=rear,rear_observed=bool(sides),
        faces=[face],side_faces=sides,all_faces=faces,kind='wall',method='observed_floor_strip',
        reason='양쪽 벽 끝 옆의 관측 바닥 영역 비교 · 넓게 보이는 쪽 선택',
        limitation='바닥 외관에 의존; 그물/투명 장애물의 부재를 보증하지 않음')


def exit_gate(image, state):
    gray = level_image(image, state)
    mask = (gray > 80).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((35, 1), np.uint8))
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    posts = [tuple(map(int, s[:4])) for s in stats[1:]
             if 2 <= s[2] <= 18 and s[3] > 65 and s[3] > 5*s[2] and s[1] < CY]
    posts.sort();candidates = []
    for i, (x, y, w, h) in enumerate(posts):
        for bx, by, bw, bh in posts[i+1:]:
            gap = bx-x-w
            if gap < 35 or abs(h-bh) > 45:
                continue
            feet=[y+h,by+bh]
            if not all(CY+12<foot<240 for foot in feet):continue
            depths=[F*state['ekf_height_m']/(foot-CY) for foot in feet]
            if abs(depths[0]-depths[1])>.25*min(depths):continue
            depth=float(np.mean(depths));center=(x+w+bx)/2
            candidates.append(dict(depth=float(depth),left=float((CX-center)*depth/F),
                rear_depth=float(depth),kind='exit',method='post_floor_contacts',gap_m=float(gap*depth/F),
                faces=[dict(box=[x,y,w,h],floor_edge=[[x,y+h],[x+w,y+h]]),
                       dict(box=[bx,by,bw,bh],floor_edge=[[bx,by+bh],[bx+bw,by+bh]])],
                reason='두 기둥 바닥 접점과 기체 높이로 거리·중앙 계산 · 폭 사전값 없음'))
    return min(candidates,key=lambda p:p['depth']) if candidates else None
