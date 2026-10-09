"""Official PID/mixer wrapper; supplied estimated state only."""
import ctypes as ct
import numpy as np
from .settings import ROOT

class StockPid:
    def __init__(self):
        self.lib=ct.CDLL(str(ROOT/"build/mission3-flight-controller/libmission3_controller.so"))
        f,u,i=ct.c_float,ct.c_uint,ct.c_int;p=ct.POINTER(f)
        self.lib.flight_init.argtypes=[];self.lib.flight_init.restype=None
        self.lib.flight_update.argtypes=[u]+[p]*6+[f,i,f,f];self.lib.flight_update.restype=None
        self.lib.flight_update_rpyt_wire.argtypes=[u]+[p]*3+[ct.POINTER(ct.c_uint8),u,f,i];self.lib.flight_update_rpyt_wire.restype=None
        self.lib.flight_motor.argtypes=[i];self.lib.flight_motor.restype=u
        self.lib.flight_thrust.argtypes=[];self.lib.flight_thrust.restype=f
        self.lib.flight_requested.argtypes=[i];self.lib.flight_requested.restype=f
        self.lib.flight_voltage.argtypes=[];self.lib.flight_voltage.restype=f
        self.lib.flight_static_force.argtypes=[u,f];self.lib.flight_static_force.restype=f
        for n in ("flight_arm","flight_thrust_to_torque"):
            getattr(self.lib,n).argtypes=[];getattr(self.lib,n).restype=f
        for n in ("flight_roll","flight_pitch","flight_yaw"):
            getattr(self.lib,n).argtypes=[];getattr(self.lib,n).restype=i
        self.lib.flight_init()

    def update_rpyt(self,tick,rpy_deg,gyro_deg_s,accel_g,wire_payload,supply_voltage=4.2,fresh_packet=True):
        from mission3_rpyt_protocol import decode_firmware_rpyt
        packet=decode_firmware_rpyt(wire_payload)
        vectors=[(ct.c_float*3)(*v) for v in (rpy_deg,gyro_deg_s,accel_g)]
        if not np.isfinite([*rpy_deg,*gyro_deg_s,*accel_g,supply_voltage]).all() or supply_voltage<=0:
            raise ValueError("finite sensor attitude/gyro/accel and positive supply required")
        payload=(ct.c_uint8*len(wire_payload)).from_buffer_copy(wire_payload)
        self.lib.flight_update_rpyt_wire(int(tick),*vectors,payload,len(wire_payload),float(supply_voltage),int(fresh_packet))
        pwm=[self.lib.flight_motor(i) for i in range(4)]
        return {"motor_pwm":pwm,
                "requested_normalized_thrust":[self.lib.flight_requested(i) for i in range(4)],
                "motor_static_force_N":[self.lib.flight_static_force(v,float(supply_voltage)) for v in pwm],
                "supply_voltage_V":float(supply_voltage),"filtered_supply_voltage_V":self.lib.flight_voltage(),
                "legacy_collective_thrust":self.lib.flight_thrust(),
                "legacy_rpy":[self.lib.flight_roll(),self.lib.flight_pitch(),self.lib.flight_yaw()],
                "position_loops_onboard_enabled":False,"rpyt_wire_hex":wire_payload.hex(),"rpyt_packet_received":bool(fresh_packet),
                "source":"official CF attitude/rate PID + mixer/battery compensation; stock RPYT-mode contract, no hardware link"}

    def update(self,tick,position,velocity,rpy_deg,gyro_deg_s,accel_g,v_command,yaw_rate_deg_s,stop=False,thrust_multiplier=1.,supply_voltage=4.2):
        arrays=[(ct.c_float*3)(*v) for v in (position,velocity,rpy_deg,gyro_deg_s,accel_g,v_command)]
        self.lib.flight_update(int(tick),*arrays,float(yaw_rate_deg_s),int(stop),float(thrust_multiplier),float(supply_voltage))
        pwm=[self.lib.flight_motor(i) for i in range(4)]
        return {"motor_pwm":pwm,
                "requested_normalized_thrust":[self.lib.flight_requested(i) for i in range(4)],
                "motor_static_force_N":[self.lib.flight_static_force(v,float(supply_voltage)) for v in pwm],
                "supply_voltage_V":float(supply_voltage),"filtered_supply_voltage_V":self.lib.flight_voltage(),
                "legacy_collective_thrust":self.lib.flight_thrust(),
                "legacy_rpy":[self.lib.flight_roll(),self.lib.flight_pitch(),self.lib.flight_yaw()],
                "source":"official CF2 PID+legacy mixer+battery compensation; official software thrust curve is conditional plant model, not measured assembled hardware"}

    @property
    def arm_length(self):return self.lib.flight_arm()

    @property
    def thrust_to_torque(self):return self.lib.flight_thrust_to_torque()
