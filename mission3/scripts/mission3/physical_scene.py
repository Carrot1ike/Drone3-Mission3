"""Conditional free rigid-body adapter. Plant truth is never a mission input.

Published outer dimensions/individual masses and official software motor fit
are reused. Unpublished inertia, mounting, motor response, friction, optics
and aerodynamics remain explicit sensitivity settings. No claim of a matched
assembled aircraft. Aircraft qpos/qvel are written only at initialization.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from yaw_core import rotation_rpy, quaternion_rpy
from .settings import ROOT
from .scene import build, R_HEADING_CAMERA
from .track import make_track, rotation
from .vehicle_motion import VehicleMotion
from .conditional_contact import build_model, FOOT_NAMES
from .optics import SyntheticLens
from .flow_texture import FlowTextureProbe


class PhysicalPlant:
    def __init__(self,cfg,physical,out,entry_id=1,seed=1,initial_arc_m=None,initial_position=None,initial_velocity=None):
        self.cfg,self.physical=cfg,physical
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.rng=np.random.default_rng(seed)
        self.track=make_track(cfg["track"])
        if initial_arc_m is None:initial_arc_m=cfg["vehicle_plant_only"].get("initial_arc_m")
        self.initial_arc=float(self.rng.uniform(0,self.track.length) if initial_arc_m is None else initial_arc_m)
        self.vehicle_motion=VehicleMotion(cfg["vehicle_plant_only"],self.track,self.initial_arc)
        entry=next(e for e in cfg["entry"]["positions"] if e["id"]==entry_id)
        p=np.r_[entry["xy_m"],cfg["entry"]["height_m"]].astype(float) if initial_position is None else np.asarray(initial_position,float).copy()
        yaw=math.radians(entry["heading_deg"])
        if cfg["entry"].get("inject_initial_uncertainty",False):
            p[:2]+=self.rng.normal(0,cfg["entry"]["initial_position_std_m"],2)
            yaw+=math.radians(self.rng.normal(0,cfg["entry"]["initial_yaw_std_deg"]))
        base=self.out/"render-scene.xml"
        build(cfg,base)
        tree=ET.parse(base);root=tree.getroot();world=root.find("worldbody")
        option=root.find("option");option.set("timestep",str(physical["simulation"]["timestep_s"]));option.set("integrator","implicitfast")
        option.set("cone","pyramidal")
        # Reuse audited conditional collision body without changing fixture.
        contact=build_model(physical["aircraft_contact_case"])
        self.contact_description=contact
        air=ET.fromstring(contact["xml"]).find("worldbody/body[@name='aircraft']")
        air.set("pos"," ".join(map(str,p)));air.set("quat",f"{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}")
        world.append(air)
        world.find("geom[@name='floor']").set("contype","1")
        world.find("geom[@name='floor']").set("conaffinity","2")
        for geom in world.findall("geom"):
            if geom.get("name","").startswith("net_"):
                geom.set("contype","4");geom.set("conaffinity","2")
        vehicle=world.find("body[@name='vehicle']")
        for geom in vehicle.findall("geom"):
            if geom.get("name") in ("landing_pad","marker_board","vehicle_artwork"):
                for k,v in {"contype":"4","conaffinity":"2","friction":f"{physical['aircraft_contact_case']['friction_sliding']} .005 .0001","condim":"3","solref":".01 1"}.items():geom.set(k,v)
        camera=world.find("camera[@name='optical']");world.remove(camera);air.append(camera)
        mounts=physical["mounts"]
        camera.set("pos"," ".join(map(str,mounts["camera_position_body_m"])))
        self.camera_rotation=rotation_rpy(*np.deg2rad(mounts["camera_rpy_offset_deg"]))@R_HEADING_CAMERA
        q=np.zeros(4);mujoco.mju_mat2Quat(q,(self.camera_rotation@np.diag([1.,-1.,-1.])).flatten())
        camera.set("quat"," ".join(map(str,q)))
        lens_settings=physical.get("synthetic_lens")
        self.lens=SyntheticLens(lens_settings) if lens_settings else None
        if self.lens:camera.set("fovy",str(self.lens.source_fov_deg))
        texture_settings=physical["sensor"].get("texture_quality_sensitivity")
        self.flow_texture_probe=FlowTextureProbe(texture_settings) if texture_settings else None
        if texture_settings:
            ET.SubElement(air,"camera",name="virtual_flow_texture_probe",
                          pos=" ".join(map(str,mounts["flow_position_body_m"])),
                          xyaxes="1 0 0 0 1 0",fovy=str(texture_settings["virtual_fov_deg"]))
        ET.SubElement(air,"site",name="imu",pos=" ".join(map(str,mounts["imu_position_body_m"])),size=".001",rgba="1 0 0 0")
        sensors=ET.SubElement(root,"sensor")
        ET.SubElement(sensors,"gyro",name="imu_gyro",site="imu")
        ET.SubElement(sensors,"accelerometer",name="imu_accel",site="imu")
        self.xml_path=self.out/"physical-scene.xml";tree.write(self.xml_path,encoding="unicode")
        self.model=mujoco.MjModel.from_xml_path(str(self.xml_path));self.data=mujoco.MjData(self.model)
        # MuJoCo znear is a fraction of stat.extent, not metres. The 50x20m
        # arena otherwise clips surfaces closer than ~7cm from the camera.
        # Save the exact runtime renderer setting in the scene snapshot.
        self.model.vis.map.znear=.001/self.model.stat.extent
        root.find("visual/map").set("znear",str(float(self.model.vis.map.znear)))
        tree.write(self.xml_path,encoding="unicode")
        self.air=self.model.body("aircraft").id;self.vehicle=self.model.body("vehicle").id
        self.aq=int(self.model.joint("aircraft_free").qposadr[0]);self.av=int(self.model.joint("aircraft_free").dofadr[0])
        self.vq=int(self.model.joint("vehicle_free").qposadr[0]);self.vv=int(self.model.joint("vehicle_free").dofadr[0])
        if initial_velocity is not None:self.data.qvel[self.av:self.av+3]=initial_velocity
        self.motor_force=np.zeros(4);self.motor_pwm=np.zeros(4,int)
        self.renderer=None;self.flow_renderer=None
        self.update_vehicle();mujoco.mj_forward(self.model,self.data)
        self.geom_class_map={n:f"foot_{i+1}" for i,n in enumerate(FOOT_NAMES)}
        self.sensor_scope={"kind":"conditional finite ray patch sensor", "sensor_settings":physical["sensor"],"actual_deck_model_verified":False}

    @property
    def time(self):return float(self.data.time)

    @property
    def position(self):return self.data.xpos[self.air].copy()

    @property
    def velocity(self):return self.data.qvel[self.av:self.av+3].copy()

    @property
    def rotation(self):return self.data.xmat[self.air].reshape(3,3).copy()

    @property
    def rpy(self):
        q=self.data.qpos[self.aq+3:self.aq+7]
        return np.array(quaternion_rpy(np.r_[q[1:],q[0]]))

    def update_vehicle(self):
        arc,speed,self.vehicle_motion_phase=self.vehicle_motion.sample(self.time)
        xy,yaw,turn=self.track.pose(arc)
        self.pad=np.r_[xy,self.cfg["vehicle_plant_only"]["pad_height_m"]]
        self.vehicle_heading=yaw;self.vehicle_velocity=speed*np.array([math.cos(yaw),math.sin(yaw),0.])
        self.vehicle_yaw_rate=speed/self.track.radius if turn else 0.;self.in_turn=turn
        # Vehicle is prescribed continuously; only the aircraft is dynamic.
        self.data.qpos[self.vq:self.vq+7]=np.r_[self.pad,[math.cos(yaw/2),0,0,math.sin(yaw/2)]]
        self.data.qvel[self.vv:self.vv+6]=np.r_[self.vehicle_velocity,[0,0,self.vehicle_yaw_rate]]

    def start_vehicle_motion(self, now):
        return self.vehicle_motion.start(now)

    def step(self,motor_output,arm_length,thrust_to_torque):
        dt=self.model.opt.timestep
        target=np.asarray(motor_output["motor_static_force_N"],float)
        self.motor_pwm=np.asarray(motor_output["motor_pwm"],int)
        tau=self.physical["motor"]["response_time_constant_s"]
        gain=1. if tau<=0 else 1-math.exp(-dt/tau)
        self.motor_force+=gain*(target*self.physical["motor"]["force_gain"]-self.motor_force)
        arm=arm_length/math.sqrt(2.)
        # Official CF2 M1 front-right, M2 rear-right, M3 rear-left, M4 front-left.
        sites=np.array([[arm,-arm,0],[-arm,-arm,0],[-arm,arm,0],[arm,arm,0]])
        forces=np.column_stack([np.zeros(4),np.zeros(4),self.motor_force])
        torque=np.cross(sites,forces).sum(axis=0)
        torque[2]+=thrust_to_torque*np.dot(self.physical["motor"]["yaw_torque_signs"],self.motor_force)
        R=self.rotation
        aero=self.physical["aerodynamics"]
        self.data.xfrc_applied[:]=0
        self.data.xfrc_applied[self.air,:3]=R@np.array([0,0,self.motor_force.sum()])-aero["linear_drag_N_per_m_s"]*self.velocity
        self.data.xfrc_applied[self.air,3:]=R@torque-aero["rotational_drag_Nm_per_rad_s"]*(R@self.data.qvel[self.av+3:self.av+6])
        self.update_vehicle();mujoco.mj_step(self.model,self.data)
        self.update_vehicle();mujoco.mj_forward(self.model,self.data)

    def imu(self):
        s=self.physical["sensor"]
        gyro=self.data.sensor("imu_gyro").data.copy()+np.deg2rad(s["gyro_bias_deg_s"])+self.rng.normal(0,math.radians(s["gyro_noise_std_deg_s"]),3)
        accel=self.data.sensor("imu_accel").data.copy()+self.rng.normal(0,s["accel_noise_std_m_s2"],3)
        return {"stamp_s":self.time,"gyro_body_rad_s":gyro.tolist(),"specific_force_body_m_s2":accel.tolist(),"conditional_mount":True}

    def patch_sensor(self,half_angle,side):
        """Equal-area discrete ray hypothesis, not PMW3901 correlation firmware.

        Returns mathematical angular surface motion, never surface IDs. Board
        occlusion/texture quality/reflectivity are still unmodeled limitations.
        """
        R=self.rotation;offset=np.asarray(self.physical["mounts"]["flow_position_body_m"])
        origin=self.position+R@offset
        omega=self.data.qvel[self.av+3:self.av+6]
        sensor_v=self.velocity+R@np.cross(omega,offset)
        angular=[];distances=[]
        tan=math.tan(math.radians(half_angle))
        for x in np.linspace(-tan,tan,side):
            for y in np.linspace(-tan,tan,side):
                ray=R@np.array([x,y,-1.]);ray/=np.linalg.norm(ray)
                if ray[2]>=-.1:continue
                t_floor=-origin[2]/ray[2]
                if t_floor<=0:continue
                t_pad=(self.pad[2]-origin[2])/ray[2]
                hit=origin+t_pad*ray
                local=rotation(self.vehicle_heading).T@(hit-self.pad)
                on_pad=(0<t_pad<t_floor and abs(local[0])<self.cfg["landing_geometry"]["pad_length_m"]/2 and abs(local[1])<self.cfg["landing_geometry"]["pad_width_m"]/2)
                d=t_pad if on_pad else t_floor
                point=origin+d*ray
                surface_v=self.vehicle_velocity+np.cross([0,0,self.vehicle_yaw_rate],point-self.pad) if on_pad else np.zeros(3)
                relative=R.T@(sensor_v-surface_v)
                # Firmware convention: projected horizontal velocity / body
                # axial range plus gyro compensation, with off-axis divergence.
                height=max(-(R.T@(point-origin))[2],.005)
                angular.append(relative[:2]/height+np.array([-omega[1],omega[0]]))
                distances.append(d)
        if not angular:return {"valid":False,"angular_rad_s":[0.,0.],"distances_m":[]}
        return {"valid":True,"angular_rad_s":np.mean(angular,axis=0).tolist(),"distances_m":distances,
                "angular_spread_rad_s":np.std(angular,axis=0).tolist()}

    def deck(self,dt=.01):
        return {**self.flow_packet(dt),**self.range_packet()}

    def flow_packet(self,dt=.01):
        s=self.physical["sensor"]
        flow=self.patch_sensor(s["flow_half_angle_deg"],s["patch_grid_side"])
        angular=np.asarray(flow["angular_rad_s"])
        distance=min(flow["distances_m"]) if flow["distances_m"] else .1
        angular+=self.rng.normal(0,s["flow_velocity_noise_std_m_s"]/max(distance,.03),2)
        factor=35./.71674/.1 # official firmware conversion, not measured FoV
        minimum=s.get("flow_component_min_working_range_m",0.)
        qualified=bool(flow["valid"] and distance>=minimum)
        texture=None
        if getattr(self,"flow_texture_probe",None) is not None:
            texture=self.flow_texture_probe.measure(self.flow_probe_image(),self.time)
            qualified=qualified and texture["usable"]
        quality_reason=("within_conditional_component_envelope" if qualified else "outside_component_working_range")
        if texture is not None and not texture["usable"]:
            quality_reason=texture["reason"]
        return {"stamp_s":self.time,"flow_raw_dpixel":(angular*dt*factor).tolist(),"flow_dt_s":dt,
                "flow_valid":qualified,"flow_target_return":flow["valid"],
                "flow_quality_reason":quality_reason,"flow_image_texture":texture,
                "flow_angular_spread_rad_s":flow.get("angular_spread_rad_s"),
                "flow_stamp_s":self.time,"model":self.sensor_scope}

    def flow_probe_image(self):
        """Native virtual downward probe; this is not an exposed chip image."""
        if self.flow_texture_probe is None:raise ValueError("texture sensitivity profile is not enabled")
        size=self.flow_texture_probe.settings.get("virtual_resolution_px",64)
        if self.flow_renderer is None:self.flow_renderer=mujoco.Renderer(self.model,height=size,width=size)
        self.flow_renderer.update_scene(self.data,camera="virtual_flow_texture_probe")
        return (self.flow_renderer.render()@np.array([.2989,.5870,.1140])).astype(np.uint8)

    def range_packet(self):
        s=self.physical["sensor"]
        tof=self.patch_sensor(s["tof_half_angle_deg"],s["patch_grid_side"])
        distance=min(tof["distances_m"]) if tof["distances_m"] else None
        target_valid=bool(distance is not None and 0<distance<=4.)
        accurate=bool(target_valid and distance>=s.get("tof_component_min_accurate_distance_m",0.))
        return {"range_stamp_s":self.time,
                "range_m":None if distance is None else float(distance+self.rng.normal(0,s["tof_noise_std_m"])),
                "range_valid":target_valid,"range_accurate":accurate,
                "range_quality_reason":"within_conditional_component_accuracy_envelope" if accurate else "outside_component_accuracy_envelope"}

    def image(self,strict_focus=False):
        if self.renderer is None:self.renderer=mujoco.Renderer(self.model,height=320,width=320)
        self.renderer.update_scene(self.data,camera="optical")
        gray=(self.renderer.render()@np.array([.2989,.5870,.1140])).astype(np.uint8)
        if self.lens:gray=self.lens.sample(gray)
        marker=self.pad+rotation(self.vehicle_heading)@np.asarray(self.cfg["marker"]["known_marker_to_pad_vehicle_m"])
        camera=self.position+self.rotation@np.asarray(self.physical["mounts"]["camera_position_body_m"])
        depth=float((self.camera_rotation.T@self.rotation.T@(marker-camera))[2])
        if strict_focus and 0<depth<self.cfg["camera"]["published_depth_of_field_near_m"]:return None,"outside_published_focus_envelope"
        return gray,"conditional_body_camera_pinhole"

    def truth(self):
        contacts=[]
        for k in range(self.data.ncon):
            c=self.data.contact[k];a,b=int(c.geom1),int(c.geom2)
            if self.model.geom_bodyid[a]!=self.air and self.model.geom_bodyid[b]!=self.air:continue
            if self.model.geom_bodyid[a]!=self.air:a,b=b,a
            name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,a) or f"aircraft_geom_{a}"
            other=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,b) or "other"
            other="net" if other.startswith("net_") else {"landing_pad":"pad","floor":"floor","marker_board":"marker_board","vehicle_artwork":"vehicle_body"}.get(other,"other")
            force=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,k,force)
            contacts.append({"aircraft_part":self.geom_class_map.get(name,name),"other":other,"geom_ids":[a,b],"distance_m":float(c.dist),
                             "normal_force_N":float(force[0]),"contact_frame_force_torque":force.tolist()})
        return {"t":self.time,"aircraft":{"center_world_m":self.position.tolist(),"velocity_world_m_s":self.velocity.tolist(),"rpy_rad":self.rpy.tolist(),"quaternion_wxyz":self.data.qpos[self.aq+3:self.aq+7].tolist()},
                "pad":{"center_world_m":self.pad.tolist()},"vehicle":{"velocity_world_m_s":self.vehicle_velocity.tolist(),"heading_rad":self.vehicle_heading,"in_turn":self.in_turn},"contacts":contacts,
                "motor_actual_force_N":self.motor_force.tolist(),"scope":"conditional free rigid-body aircraft; aircraft state never prescribed after t=0"}

    def close(self):
        if self.renderer is not None:self.renderer.close()
        if self.flow_renderer is not None:self.flow_renderer.close()
