/* Host execution adapter for unmodified Bitcraze stock PID and power mixer.
 * Sensor/estimated state and velocity setpoint are inputs, never plant truth.
 * Hardware/RTOS/link/motor physics remain outside this C adapter.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "autoconf.h"
#include "controller_pid.h"
#include "crtp_commander.h"
#include "commander.h"
#include "power_distribution.h"
#include "platform_defaults.h"

#ifndef NBR_OF_MOTORS
#define NBR_OF_MOTORS 4
#endif
void assertFail(char *exp,char *file,int line);
#ifndef ASSERT
#define ASSERT(e) do { if(!(e)) assertFail(#e,__FILE__,__LINE__); } while(0)
#endif
#include "official_motor_compensation.inc"

static control_t output;
static uint16_t motor[4];
static float requested[4];
static float supply_filtered=4.2f;
static int rpyt_priority=COMMANDER_PRIORITY_DISABLE;
static setpoint_t rpyt_setpoint;
int commanderGetActivePriority(void) {return rpyt_priority;}

static void flight_mix(int stop,float supply_voltage) {
  motors_thrust_uncapped_t uncapped={0},compensated={0};motors_thrust_pwm_t capped={0};
  powerDistribution(&output,&uncapped);
  supply_filtered+=0.01f*(supply_voltage-supply_filtered);
  for(int i=0;i<4;i++) {
    requested[i]=stop?0:uncapped.list[i];
    compensated.list[i]=motorsCompensateBatteryVoltage(i,requested[i],supply_filtered);
  }
  powerDistributionCap(&compensated,&capped);
  for(int i=0;i<4;i++)motor[i]=stop?0:capped.list[i];
}

void assertFail(char *exp,char *file,int line) {
  fprintf(stderr,"controller assertion: %s %s:%d\n",exp,file,line);abort();
}

void flight_init(void) {
  memset(&output,0,sizeof(output));memset(motor,0,sizeof(motor));
  memset(requested,0,sizeof(requested));supply_filtered=4.2f;
  rpyt_priority=COMMANDER_PRIORITY_DISABLE;
  memset(&rpyt_setpoint,0,sizeof(rpyt_setpoint));
  controllerPidInit();powerDistributionInit();
}

void flight_update(unsigned tick,const float *position,const float *velocity,
                   const float *attitude_degrees,const float *gyro_degrees_s,
                   const float *accel_g,const float *command_velocity,float command_yaw_rate_deg_s,
                   int stop,float thrust_multiplier,float supply_voltage) {
  state_t state={0};sensorData_t sensors={0};setpoint_t sp={0};
  state.position.x=position[0];state.position.y=position[1];state.position.z=position[2];
  state.velocity.x=velocity[0];state.velocity.y=velocity[1];state.velocity.z=velocity[2];
  state.attitude.roll=attitude_degrees[0];state.attitude.pitch=attitude_degrees[1];state.attitude.yaw=attitude_degrees[2];
  sensors.gyro.x=gyro_degrees_s[0];sensors.gyro.y=gyro_degrees_s[1];sensors.gyro.z=gyro_degrees_s[2];
  sensors.acc.x=accel_g[0];sensors.acc.y=accel_g[1];sensors.acc.z=accel_g[2];
  sp.mode.x=modeVelocity;sp.mode.y=modeVelocity;sp.mode.z=modeVelocity;
  sp.mode.yaw=modeVelocity;sp.velocity_body=false;
  sp.velocity.x=command_velocity[0];sp.velocity.y=command_velocity[1];sp.velocity.z=command_velocity[2];
  sp.attitudeRate.yaw=command_yaw_rate_deg_s;
  if(stop) {sp.mode.z=modeDisable;sp.thrust=0.;}
  controllerPid(&output,&sp,&sensors,&state,tick);
  output.thrust*=thrust_multiplier;
  // Matches stabilizer batteryCompensation order and initial filter value.
  flight_mix(stop,supply_voltage);
}

void flight_update_rpyt_wire(unsigned tick,const float *attitude_degrees,const float *gyro_degrees_s,
                            const float *accel_g,const uint8_t *payload,unsigned length,float supply_voltage,int fresh_packet) {
  state_t state={0};sensorData_t sensors={0};
  CRTPPacket packet={0};
  if(length!=14) {fprintf(stderr,"RPYT payload length must be14\n");abort();}
  packet.size=length;packet.header=CRTP_HEADER(CRTP_PORT_SETPOINT,0);
  if(fresh_packet) {
    memset(&rpyt_setpoint,0,sizeof(rpyt_setpoint));
    memcpy(packet.data,payload,length);
    // Unmodified decoder applies default ANGLE/RATE/XMODE and thrust lock.
    crtpCommanderRpytDecodeSetpoint(&rpyt_setpoint,&packet);
    rpyt_priority=COMMANDER_PRIORITY_CRTP;
  }
  state.attitude.roll=attitude_degrees[0];state.attitude.pitch=attitude_degrees[1];state.attitude.yaw=attitude_degrees[2];
  sensors.gyro.x=gyro_degrees_s[0];sensors.gyro.y=gyro_degrees_s[1];sensors.gyro.z=gyro_degrees_s[2];
  sensors.acc.x=accel_g[0];sensors.acc.y=accel_g[1];sensors.acc.z=accel_g[2];
  controllerPid(&output,&rpyt_setpoint,&sensors,&state,tick);
  flight_mix(rpyt_setpoint.thrust==0,supply_voltage);
}

unsigned flight_motor(int i) { return motor[i]; }
float flight_requested(int i) { return requested[i]; }
float flight_voltage(void) { return supply_filtered; }
float flight_static_force(unsigned pwm,float actual_supply) {
  if(!pwm)return 0.0f;
  float v=actual_supply*pwm/65535.0f;
  float f=VMOTOR2THRUST0+v*(VMOTOR2THRUST1+v*(VMOTOR2THRUST2+v*VMOTOR2THRUST3));
  return f>0?f:0;
}
float flight_arm(void) {return ARM_LENGTH;}
float flight_thrust_to_torque(void) {return THRUST2TORQUE;}
float flight_thrust(void) { return output.thrust; }
int flight_roll(void) { return output.roll; }
int flight_pitch(void) { return output.pitch; }
int flight_yaw(void) { return output.yaw; }
