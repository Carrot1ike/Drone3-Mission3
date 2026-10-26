/* PC-side execution of the unmodified official velocity/position PID.
 * This produces attitude angles and legacy collective for a stock RPYT
 * commander. It never receives plant state or motor/contact truth.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "position_controller.h"
#include "stabilizer_types.h"

static attitude_t attitude;
static float thrust;

void assertFail(char *exp,char *file,int line) {
  fprintf(stderr,"position bridge assertion: %s %s:%d\n",exp,file,line);abort();
}

void bridge_init(float x,float y,float z) {
  memset(&attitude,0,sizeof(attitude));thrust=0;
  positionControllerInit();
  positionControllerResetAllPID(x,y,z);positionControllerResetAllfilters();
}

void bridge_update(const float *position,const float *velocity,float heading_deg,
                   const float *command_velocity) {
  state_t state={0};setpoint_t sp={0};
  state.position.x=position[0];state.position.y=position[1];state.position.z=position[2];
  state.velocity.x=velocity[0];state.velocity.y=velocity[1];state.velocity.z=velocity[2];
  state.attitude.yaw=heading_deg;
  sp.mode.x=modeVelocity;sp.mode.y=modeVelocity;sp.mode.z=modeVelocity;
  sp.velocity_body=false;
  sp.velocity.x=command_velocity[0];sp.velocity.y=command_velocity[1];sp.velocity.z=command_velocity[2];
  positionController(&thrust,&attitude,&sp,&state);
}

float bridge_roll_deg(void) {return attitude.roll;}
float bridge_pitch_deg(void) {return attitude.pitch;}
float bridge_collective(void) {return thrust;}
