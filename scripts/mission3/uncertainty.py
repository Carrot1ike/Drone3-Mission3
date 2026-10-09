"""조건부 수평 오차 공분산. 실측 noise로 보정한 확률 보장이 아니다.

평균 상태는 기존 complementary observer가 유지한다. 이 클래스는
같은 Flow 융합 gain의 Joseph 전파를 사용하며 yaw/bias를 추적한다.
지도/영상 anchor의 상관성은 미확정이라 공분산 축소에는 사용하지 않는다.
"""
import math
import numpy as np


class MotionUncertainty:
    # x,y,vx,vy,yaw,gyro_z_bias,accel_body_x_bias,accel_body_y_bias
    def __init__(self, entry, settings):
        self.c=settings
        sigma=[entry["initial_position_std_m"]]*2+[settings["initial_velocity_std_m_s"]]*2+[
            math.radians(entry["initial_yaw_std_deg"]),math.radians(settings["initial_gyro_bias_std_deg_s"]),
            settings["initial_accel_bias_std_m_s2"],settings["initial_accel_bias_std_m_s2"]]
        self.P=np.diag(np.square(sigma));self.floor_updates=0;self.propagation_s=0.
        self.heading_floor_active=False
        self.straight_updates=0

    def predict(self, dt, heading, world_acceleration):
        if dt<=0:return
        self.propagation_s+=dt
        c,s=math.cos(heading),math.sin(heading);R=np.array([[c,-s],[s,c]])
        accel=np.asarray(world_acceleration)[:2]
        F=np.eye(8);F[:2,2:4]=np.eye(2)*dt
        F[2:4,4]=np.array([-accel[1],accel[0]])*dt
        F[:2,4]=np.array([-accel[1],accel[0]])*dt*dt/2
        F[2:4,6:8]=-R*dt;F[:2,6:8]=-R*dt*dt/2;F[4,5]=-dt
        L=np.zeros((8,3));L[:2,:2]=R*dt*dt/2;L[2:4,:2]=R*dt;L[4,2]=dt
        noise=np.diag([self.c["accel_sample_std_m_s2"]**2]*2+[math.radians(self.c["gyro_sample_std_deg_s"])**2])
        Q=L@noise@L.T
        Q[5,5]+=math.radians(self.c["gyro_bias_walk_deg_s_sqrt_s"])**2*dt
        Q[6,6]+=self.c["accel_bias_walk_m_s2_sqrt_s"]**2*dt;Q[7,7]=Q[6,6]
        self.P=F@self.P@F.T+Q;self.P=(self.P+self.P.T)/2
        if self.heading_floor_active:
            self.P[4,4]=max(self.P[4,4],math.radians(self.c.get("straight_heading_systematic_floor_deg",2.))**2)

    def qualified_floor_flow(self, world_velocity, gain,bias_gain=0.,heading=0.):
        velocity=np.asarray(world_velocity)[:2]
        H=np.zeros((2,8));H[:,2:4]=np.eye(2)
        # World velocity projected with an uncertain yaw shares the yaw gauge.
        H[:,4]=-np.array([-velocity[1],velocity[0]])
        K=np.zeros((8,2));K[2:4,:]=np.eye(2)*gain
        c,s=math.cos(heading),math.sin(heading)
        K[6:8,:]=-bias_gain*np.array([[c,s],[-s,c]])
        A=np.eye(8)-K@H
        R=np.eye(2)*self.c["floor_flow_velocity_std_m_s"]**2
        self.P=A@self.P@A.T+K@R@K.T;self.P=(self.P+self.P.T)/2
        self.floor_updates+=1

    def change_heading_gauge(self, yaw_delta):
        c,s=math.cos(yaw_delta),math.sin(yaw_delta);J=np.eye(8)
        J[2:4,2:4]=[[c,-s],[s,c]];self.P=J@self.P@J.T

    def straight_heading_update(self, heading_gain, bias_gain):
        # Same gains as the mean observer. Relative pose plus known straight
        # supplies the datum; repeated correlated frames retain a yaw floor.
        self.heading_floor_active=True
        self.straight_updates+=1
        H=np.zeros((1,8));H[0,4]=1.
        K=np.zeros((8,1));K[4,0]=heading_gain;K[5,0]=bias_gain
        A=np.eye(8)-K@H
        R=math.radians(self.c.get("straight_heading_measurement_std_deg",5.))**2
        self.P=A@self.P@A.T+(K@K.T)*R
        floor=math.radians(self.c.get("straight_heading_systematic_floor_deg",2.))**2
        self.P[4,4]=max(self.P[4,4],floor)
        self.P=(self.P+self.P.T)/2

    def state(self):
        sigma=math.sqrt(max(0.,float(np.linalg.eigvalsh(self.P[:2,:2]).max())))
        return {"position_std_m":sigma,"horizontal_covariance_m2":self.P[:2,:2].tolist(),
            "heading_std_deg":math.degrees(math.sqrt(max(0.,self.P[4,4]))),
            "uncertainty_method":"conditional yaw/bias propagation with fixed-gain qualified-floor-Flow Joseph updates; map correlation not used to shrink covariance",
            "uncertainty_is_calibrated_probability":False,"qualified_floor_covariance_updates":self.floor_updates,
            "qualified_straight_heading_updates":self.straight_updates}
