"""사용자 그림의 삼각형을 화각·인식 후보 영역으로 표현하는 평가 도식."""
import math
import cv2
import numpy as np
from optical_visibility import horizontal_fov_deg, display_masks

SCREEN_SIZE = (1100, 840)


def text(image, value, xy, scale=.48, color=0):
    cv2.putText(image, value, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def dashed_ray(image, origin, angle, box):
    end = (origin[0]+int(2000*math.cos(angle)), origin[1]-int(2000*math.sin(angle)))
    ok, a, b = cv2.clipLine(box, origin, end)
    if not ok:
        return
    length = math.dist(a, b)
    if length < 1.:
        return
    direction = (np.array(b)-a)/length
    for d in np.arange(0., length, 12.):
        p, q = np.rint(np.array(a)+direction*d).astype(int), np.rint(np.array(a)+direction*min(length, d+7.)).astype(int)
        cv2.line(image, tuple(p), tuple(q), 70, 1, cv2.LINE_AA)


def panel(gray, row, phase, t, config, profile, initial_angle, frame_observation, surface_sample=None):
    if gray.shape != (320, 320) or gray.dtype != np.uint8:
        raise ValueError("review needs original mono uint8 320x320 frames")
    width, height = SCREEN_SIZE
    image = np.full((height, width), 246, np.uint8)
    image[130:450, 20:340] = gray
    text(image, "NATIVE MONO 320x320", (20, 30), .58)
    text(image, "Recorded camera image", (20, 60))
    text(image, "CAMERA FOV / RECOGNITION CANDIDATES", (380, 30), .56)
    text(image, f"t={t:.2f}s  {phase}", (380, 60), .54)
    text(image, f"Vehicle setting +{config['vehicle']['speed_m_s']:.1f}m/s | travel {float(row['vehicle_travel_m']):.2f}m", (380, 88))
    text(image, f"Initial forward-to-rear angle: {initial_angle:.1f}deg", (380, 116))

    # 카메라 기준 상대 도식. 오른쪽 투영 조건은 제어기로 보내지 않는다.
    left, top, plot_width, plot_height = 370, 140, 720, 350
    origin, scale = (700, 400), 240.
    angle = math.radians(float(row["heading_deg"]))
    marker_z = float(row["pad_z_m"])+config["vehicle"]["marker_offset_from_pad_m"][2]
    fov, candidate = display_masks((plot_height, plot_width), (origin[0]-left, origin[1]-top), scale,
        angle, float(row["camera_z_m"]), marker_z, profile, config["observer"]["min_side_px"])
    # 후보 도식만 확대해 칠한다. 왼쪽 320×320 입력은 확대/변환하지 않는다.
    fov = cv2.resize(fov.astype(np.uint8), (plot_width, plot_height), interpolation=cv2.INTER_NEAREST).astype(bool)
    candidate = cv2.resize(candidate.astype(np.uint8), (plot_width, plot_height), interpolation=cv2.INTER_NEAREST).astype(bool)
    plot = image[top:top+plot_height, left:left+plot_width]
    plot[fov], plot[candidate] = 232, 185
    half_fov = math.radians(horizontal_fov_deg(profile)/2)
    box = (left, top, plot_width, plot_height)
    dashed_ray(image, origin, angle-half_fov, box)
    dashed_ray(image, origin, angle+half_fov, box)
    cv2.rectangle(image, (left, top), (left+plot_width-1, top+plot_height-1), 210, 1)

    cx = origin[0]+int(float(row["relative_x_m"])*scale)
    cy = origin[1]-int(float(row["relative_y_m"])*scale)
    cv2.line(image, (left, cy), (left+plot_width-1, cy), 160, 1)
    text(image, "vehicle path +X", (935, cy-8), .34, 70)
    cv2.arrowedLine(image, (985, cy+12), (1035, cy+12), 70, 1)
    hx, hy = int(config["vehicle"]["pad_length_m"]*scale/2), int(config["vehicle"]["pad_width_m"]*scale/2)
    cv2.rectangle(image, (cx-hx, cy-hy), (cx+hx, cy+hy), 50, 2)
    text(image, "30cm pad", (cx-hx, cy-hy-9), .38)
    mx, my, _ = config["vehicle"]["marker_offset_from_pad_m"]
    mark = (cx+int(mx*scale), cy-int(my*scale))
    mh = int(config["marker"]["length_m"]*scale/2)
    cv2.line(image, (mark[0], mark[1]-mh), (mark[0], mark[1]+mh), 0, 3)
    text(image, "ArUco", (mark[0]-20, mark[1]-mh-8), .37)
    accepted = frame_observation is not None and frame_observation.get("valid", False)
    cv2.circle(image, mark, 7, 0, -1 if accepted else 1)

    # 그림용 드론 기호의 중심은 가상 광학점. 기체 크기/장착값을 뜻하지 않는다.
    for a in (angle+math.pi/4, angle-math.pi/4):
        d = np.array([math.cos(a), -math.sin(a)])*14
        p, q = np.rint(np.array(origin)-d).astype(int), np.rint(np.array(origin)+d).astype(int)
        cv2.line(image, tuple(p), tuple(q), 0, 2, cv2.LINE_AA)
        cv2.circle(image, tuple(p), 4, 0, 1, cv2.LINE_AA)
        cv2.circle(image, tuple(q), 4, 0, 1, cv2.LINE_AA)
    cv2.circle(image, origin, 4, 0, -1)
    direction = (origin[0]+int(65*math.cos(angle)), origin[1]-int(65*math.sin(angle)))
    cv2.arrowedLine(image, origin, direction, 0, 2, tipLength=.12)
    text(image, "camera origin / drone symbol", (origin[0]-86, origin[1]+30), .36)
    if phase == "WAIT":
        text(image, "fixed position + fixed gaze", (origin[0]-86, origin[1]+47), .36)

    verdict = "pending frame" if frame_observation is None else frame_observation.get("reason", "unknown")
    text(image, f"Frame verdict: {verdict}", (20, 477), .43)
    if frame_observation is not None and "min_side_px" in frame_observation:
        text(image, f"Marker min side: {frame_observation['min_side_px']:.1f}px", (20, 505), .43)
    text(image, "Post-run verdict from image detector", (20, 532), .39)
    error = math.hypot(float(row["relative_x_m"]), float(row["relative_y_m"]))
    text(image, f"Optical XY error {100*error:.2f}cm | height {100*float(row['height_above_pad_m']):.2f}cm", (380, 520), .47)
    cv2.rectangle(image, (380, 539), (400, 552), 232, -1)
    text(image, f"Camera horizontal FOV {horizontal_fov_deg(profile):.1f}deg (rays extend beyond plot)", (410, 551), .41)
    cv2.rectangle(image, (380, 563), (400, 576), 185, -1)
    text(image, f"Geometric candidates: front face, whole marker in frame, >= {config['observer']['min_side_px']:.0f}px", (410, 575), .40)
    text(image, "SIDE VIEW / HEIGHT (virtual optical point)", (20, 608), .47)
    cv2.rectangle(image, (20, 620), (550, 801), 210, 1)
    gx, gy, zscale = 290, 785, 260.
    pad_x = gx+int(float(row["relative_x_m"])*180.)
    pad_y = gy-int(float(row["pad_z_m"])*zscale)
    camera_y = gy-int(float(row["camera_z_m"])*zscale)
    cv2.line(image, (30, gy), (540, gy), 90, 1)
    text(image, "ground Z=0", (35, gy-5), .33, 70)
    cv2.rectangle(image, (pad_x-27, pad_y), (pad_x+27, pad_y+8), 70, 2)
    text(image, "moving pad", (pad_x-33, pad_y+23), .32)
    cv2.circle(image, (gx, camera_y), 5, 0, -1)
    text(image, "camera", (gx-24, camera_y-12), .34)
    world_z, above_pad = float(row["camera_z_m"]), float(row["height_above_pad_m"])
    state = ("DESCENDING" if phase == "DESCEND" else "NEAR PAD (NO CONTACT)" if phase in ("SETTLE", "COMPLETE")
             else "STABLE TRACK HOLD" if phase == "TRACK" else "HEIGHT HOLD")
    text(image, state, (35, 640), .40)
    text(image, f"World Z {100*world_z:.1f}cm / above pad {100*above_pad:.1f}cm", (35, 666), .38)
    if surface_sample is not None:
        hit_y = gy-int(float(surface_sample["surface_z_m"])*zscale)
        cv2.arrowedLine(image, (gx, camera_y+5), (gx, hit_y), 70, 1)
        text(image, f"ray {100*float(surface_sample['virtual_range_m']):.1f}cm", (gx+10, (camera_y+hit_y)//2), .33)
    text(image, "VIRTUAL FLOW / ToF REPLAY - OUTSIDE CONTROL LOOP", (585, 610), .41)
    if surface_sample is not None:
        lines = [f"Ray surface: {surface_sample['surface']}",
            f"World VX: {float(surface_sample['camera_vx_m_s']):+.3f} m/s",
            f"Surface VX: {float(surface_sample['surface_vx_m_s']):+.3f} m/s",
            f"Relative VX: {float(surface_sample['relative_vx_world_m_s']):+.3f} m/s",
            f"Static-ground VX bias: {float(surface_sample['naive_static_ground_vx_error_m_s']):+.3f} m/s"]
    else:
        lines = ["Surface replay not available"]
    for i, value in enumerate(lines):
        text(image, value, (585, 642+i*27), .43)
    text(image, "Probe location and flow are ideal diagnostic assumptions", (585, 797), .35)
    text(image, "OPTICAL KINEMATICS | EVALUATION OVERLAY | FLOW NOT FUSED | PHYSICAL TOUCHDOWN UNTESTED", (20, 829), .44)
    return image
