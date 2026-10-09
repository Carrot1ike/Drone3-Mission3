"""사용자 경기장·움직이는 차량·원본 mono 카메라의 기하 시험 장면.

이 모듈에는 기체 질량·관성·추력·카메라 body 변환이 없다. 실제 동역학
프로파일 확보 전에는 광학점 기하 시험으로만 사용한다.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import mujoco
import numpy as np
from .track import make_track, rotation
from .vehicle_motion import VehicleMotion

R_VEHICLE_MARKER = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
R_HEADING_CAMERA = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def level_surface_patch(position,velocity,pad,vehicle_heading,vehicle_velocity,vehicle_yaw_rate,
                        pad_half,half_angle_deg,grid_side=5):
    """Finite ray patch sensitivity model; deck optics/correlation are unknown.

    Internal surface labels never leave the sensor adapter. The sensor returns
    mean angular displacement and candidate optical ranges. Equal ray weights
    and nearest ToF selection are experiment policies, not manufacturer facts.
    """
    slopes=np.linspace(-math.tan(math.radians(half_angle_deg)),math.tan(math.radians(half_angle_deg)),grid_side)
    x,y=np.meshgrid(slopes,slopes)
    directions=np.column_stack([x.ravel(),y.ravel(),-np.ones(x.size)])
    z=position[2]
    t_pad=z-pad[2]
    hits=position+t_pad*directions
    local=(hits-pad)@rotation(vehicle_heading)
    on_pad=(t_pad>0)&(np.abs(local[:,0])<pad_half[0])&(np.abs(local[:,1])<pad_half[1])
    height=np.where(on_pad,t_pad,z)
    points=position+height[:,None]*directions
    moving_v=vehicle_velocity+np.cross(np.tile([0.,0.,vehicle_yaw_rate],(len(points),1)),points-pad)
    surface_v=np.where(on_pad[:,None],moving_v,0.)
    angular=(velocity-surface_v)[:,:2]/np.maximum(height[:,None],.005)
    distances=height*np.linalg.norm(directions,axis=1)
    return angular.mean(axis=0),distances


def build(cfg, path):
    root = ET.Element("mujoco", model="mission3_optical_geometry")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", timestep="0.02", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(visual, "map", znear="0.001", zfar="100")
    ET.SubElement(visual, "quality", shadowsize="2048")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="tile", type="2d", builtin="checker", width="512", height="512",
                  rgb1=".50 .50 .50", rgb2=".62 .62 .62")
    arena = cfg["arena"]
    length, width = arena["length_m"], arena["width_m"]
    ET.SubElement(asset, "material", name="floor_tile", texture="tile", texrepeat=f"{length} {width}", texuniform="true", reflectance="0")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos=f"{length/2} {width/2} 6", directional="true", dir="0 0 -1", diffuse=".8 .8 .8", ambient=".4 .4 .4", castshadow="false")
    ET.SubElement(world, "geom", name="floor", type="plane", pos=f"{length/2} {width/2} 0",
                  size=f"{length/2} {width/2} .1", material="floor_tile", group="0")
    # Net rods are visible objects; no imaginary opaque wall texture or track lane.
    net_h, spacing, radius = arena["net_height_m"], arena["net_mesh_m"], arena["net_thread_radius_m"]
    for edge_id,(a,b) in enumerate((((0,0),(length,0)),((0,width),(length,width)),
                                  ((0,0),(0,width)),((length,0),(length,width)))):
        edge = np.array(b) - a
        n = int(np.linalg.norm(edge) / spacing)
        for i in range(n + 1):
            x, y = np.array(a) + edge * i / n
            ET.SubElement(world, "geom", name=f"net_{edge_id}_vertical_{i}",type="capsule", fromto=f"{x} {y} 0 {x} {y} {net_h}",
                          size=str(radius), rgba=".25 .25 .25 1", group="2", contype="0", conaffinity="0")
        for zi,z in enumerate(np.arange(spacing, net_h + 1e-6, spacing)):
            ET.SubElement(world, "geom",name=f"net_{edge_id}_horizontal_{zi}",type="capsule", fromto=f"{a[0]} {a[1]} {z} {b[0]} {b[1]} {z}",
                          size=str(radius), rgba=".25 .25 .25 1", group="2", contype="0", conaffinity="0")
    vehicle = cfg["vehicle_plant_only"]
    pad_h = vehicle["pad_height_m"]
    cx,cy = cfg["track"]["center_world_xy_m"]
    if cfg["track"].get("kind") == "straight":cx,cy=cfg["track"]["start_world_xy_m"]
    else:cy-=cfg["track"]["turn_diameter_m"]/2
    body = ET.SubElement(world, "body", name="vehicle", pos=f"{cx} {cy} {pad_h}")
    ET.SubElement(body, "freejoint", name="vehicle_free")
    ET.SubElement(body, "inertial", mass="10", pos="0 0 -.1", diaginertia="1 1 1")
    # Prescribed vehicle prop values have no claim of actual vehicle mass/contact.
    thick = vehicle["pad_thickness_m"]
    ET.SubElement(body, "geom", name="landing_pad", type="box", pos=f"0 0 {-thick/2}",
                  size=f"{vehicle['pad_length_m']/2} {vehicle['pad_width_m']/2} {thick/2}",
                  rgba=".85 .85 .85 1", group="0")
    ET.SubElement(body, "geom", name="vehicle_artwork", type="box", pos=".10 0 -.10",
                  size=".23 .12 .06", rgba=".30 .30 .30 1", group="2", contype="0", conaffinity="0")
    mx, my, mz = vehicle["marker_center_from_pad_vehicle_m"]
    board_side = cfg["marker"]["board_side_m"]
    ET.SubElement(body, "geom", name="marker_board", type="box", pos=f"{mx+.002} {my} {mz}",
                  size=f".002 {board_side/2} {board_side/2}", rgba="1 1 1 1", group="2", contype="0", conaffinity="0")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["marker"]["dictionary"]))
    modules = dictionary.markerSize + 2
    bits = cv2.aruco.generateImageMarker(dictionary, cfg["marker"]["id"], modules)
    code_side = cfg["marker"]["code_side_m"]
    cell = code_side / modules
    for row, col in np.argwhere(bits == 0):
        ET.SubElement(body, "geom", type="box",
                      pos=f"{mx-.00025} {my+code_side/2-(col+.5)*cell} {mz+code_side/2-(row+.5)*cell}",
                      size=f".00025 {cell/2} {cell/2}", rgba="0 0 0 1", group="2", contype="0", conaffinity="0")
    ET.SubElement(world, "camera", name="optical", pos="1 2 1", fovy=str(cfg["camera"]["vertical_fov_deg"]))
    ET.SubElement(world, "camera", name="overview", pos=f"{length/2} {-width/2} {max(length,width)*.56}", xyaxes="1 0 0 0 .82 .57", fovy="75")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding="unicode")
    return mujoco.MjModel.from_xml_path(str(path))


class GeometryPlant:
    """명령으로만 움직이는 광학점. 센서는 별도 명시적 수학적 probe."""
    def __init__(self, cfg, out, entry_id=1, seed=1, initial_arc_m=None):
        self.cfg = cfg
        self.track = make_track(cfg["track"])
        self.rng = np.random.default_rng(seed)
        if initial_arc_m is None:initial_arc_m=cfg["vehicle_plant_only"].get("initial_arc_m")
        self.initial_arc = float(self.rng.uniform(0, self.track.length) if initial_arc_m is None else initial_arc_m)
        self.vehicle_motion=VehicleMotion(cfg["vehicle_plant_only"],self.track,self.initial_arc)
        entry = next(v for v in cfg["entry"]["positions"] if v["id"] == entry_id)
        self.position = np.r_[entry["xy_m"], cfg["entry"]["height_m"]].astype(float)
        self.heading = math.radians(entry["heading_deg"])
        if cfg["entry"].get("inject_initial_uncertainty",False):
            self.position[:2]+=self.rng.normal(0.,cfg["entry"]["initial_position_std_m"],2)
            self.heading+=math.radians(self.rng.normal(0.,cfg["entry"]["initial_yaw_std_deg"]))
        self.velocity = np.zeros(3)
        self.yaw_rate = 0.
        self.last_accel = np.zeros(3)
        self.time = 0.
        self.model = build(cfg, Path(out) / "scene.xml")
        self.data = mujoco.MjData(self.model)
        self.camera = self.model.camera("optical").id
        self.vehicle_id = self.model.body("vehicle").id
        self.qadr = self.model.joint("vehicle_free").qposadr[0]
        self.vadr = self.model.joint("vehicle_free").dofadr[0]
        self.renderer = mujoco.Renderer(self.model, height=320, width=320)
        self.update_scene()

    def update_scene(self):
        arc,speed,self.vehicle_motion_phase=self.vehicle_motion.sample(self.time)
        xy, yaw, turn = self.track.pose(arc)
        self.pad = np.r_[xy, self.cfg["vehicle_plant_only"]["pad_height_m"]]
        self.vehicle_heading = yaw
        self.vehicle_velocity = speed * np.array([math.cos(yaw), math.sin(yaw), 0.])
        self.vehicle_yaw_rate = speed / self.track.radius if turn else 0.
        self.in_turn = turn
        self.data.qpos[self.qadr:self.qadr+3] = self.pad
        self.data.qpos[self.qadr+3:self.qadr+7] = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
        self.data.qvel[self.vadr:self.vadr+3] = self.vehicle_velocity
        self.data.qvel[self.vadr+3:self.vadr+6] = [0., 0., self.vehicle_yaw_rate]
        self.model.cam_pos[self.camera] = self.position
        R = rotation(self.heading) @ R_HEADING_CAMERA @ np.diag([1., -1., -1.])
        mujoco.mju_mat2Quat(self.model.cam_quat[self.camera], R.flatten())
        mujoco.mj_forward(self.model, self.data)

    def start_vehicle_motion(self, now):
        return self.vehicle_motion.start(now)

    def step(self, velocity, yaw_rate, dt):
        new_v = np.asarray(velocity, dtype=float)
        self.last_accel = (new_v - self.velocity) / dt
        self.velocity = new_v
        self.yaw_rate = float(yaw_rate)
        self.position += new_v * dt
        self.heading = (self.heading + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        self.time += dt
        self.update_scene()

    def image(self, strict_focus=False):
        self.renderer.update_scene(self.data, camera="optical")
        gray = (self.renderer.render() @ np.array([.2989, .5870, .1140])).astype(np.uint8)
        # Manufacturer gives DOF, not a lens blur PSF. Strict experiment excludes
        # nearer observations, without inventing exact optics outside public DOF.
        marker = self.pad + rotation(self.vehicle_heading) @ np.array(self.cfg["marker"]["known_marker_to_pad_vehicle_m"])
        optical_depth = float((rotation(self.heading).T @ (marker-self.position))[0])
        if strict_focus and 0. < optical_depth < self.cfg["camera"]["published_depth_of_field_near_m"]:
            return None, "outside_published_focus_envelope"
        return gray, "nominal_pinhole"

    def sensor_packet(self, dt):
        """plant 표면 ID는 반환 금지. level probe의 수학 센서만 전달."""
        cfg = self.cfg["sensor_experiment"]
        vcfg = self.cfg["vehicle_plant_only"]
        half=[vcfg["pad_length_m"]/2,vcfg["pad_width_m"]/2]
        if cfg.get("finite_patch_enabled",False):
            angular,_=level_surface_patch(self.position,self.velocity,self.pad,self.vehicle_heading,
                 self.vehicle_velocity,self.vehicle_yaw_rate,half,cfg["flow_half_angle_sensitivity_deg"],cfg["patch_grid_side"])
            _,ranges=level_surface_patch(self.position,self.velocity,self.pad,self.vehicle_heading,
                 self.vehicle_velocity,self.vehicle_yaw_rate,half,cfg["tof_half_angle_sensitivity_deg"],cfg["patch_grid_side"])
            height=float(np.min(ranges))
            angular_body=(rotation(self.heading).T@np.r_[angular,0.])[:2]
        else:
            local = rotation(self.vehicle_heading).T @ (self.position - self.pad)
            pad_hit=(abs(local[0])<=half[0] and abs(local[1])<=half[1] and local[2]>0.)
            height=self.position[2]-(self.pad[2] if pad_hit else 0.)
            surface_v=self.vehicle_velocity+np.cross([0.,0.,self.vehicle_yaw_rate],self.position-self.pad) if pad_hit else np.zeros(3)
            angular_body=(rotation(self.heading).T@(self.velocity-surface_v))[:2]/max(height,.03)
        gyro = self.yaw_rate + math.radians(cfg["gyro_bias_deg_s"] + self.rng.normal(0, cfg["gyro_noise_std_deg_s"]))
        angular_body+=self.rng.normal(0,cfg["flow_velocity_noise_std_m_s"]/max(height,.03),2)
        # Angular flow rather than absolute velocity: depends on surface distance.
        return {"stamp_s": self.time, "gyro_z_rad_s": gyro,
                "accel_heading_m_s2": (rotation(self.heading).T @ self.last_accel + self.rng.normal(0, cfg["imu_accel_noise_std_m_s2"], 3)).tolist(),
                "flow_angular_rad_s": angular_body.tolist(),"flow_valid":True,
                "range_m": float(height + self.rng.normal(0, cfg["tof_noise_std_m"])),
                "range_valid":bool(height>0.),"finite_patch_sensitivity_model":bool(cfg.get("finite_patch_enabled",False)),
                "virtual_level_probe": True, "noise_is_sensitivity_assumption": True}

    def truth(self):
        return {"time_s": self.time, "optical_position_world_m": self.position.tolist(),
                "optical_heading_rad": self.heading, "optical_velocity_world_m_s": self.velocity.tolist(),
                "pad_world_m": self.pad.tolist(), "vehicle_heading_rad": self.vehicle_heading,
                "vehicle_velocity_world_m_s": self.vehicle_velocity.tolist(), "vehicle_in_turn": self.in_turn,
                "physical_contact": False, "motors_exist": False, "scope": "optical kinematics only"}

    def close(self):
        self.renderer.close()
