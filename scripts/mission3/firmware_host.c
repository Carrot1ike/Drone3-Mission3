/* Project host adapter for unmodified Bitcraze kalman core/Flow/ToF code.
 * This is not FreeRTOS, a sensor driver, the flight controller or an aircraft.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "kalman_core.h"
#include "mm_flow.h"
#include "mm_tof.h"

static kalmanCoreData_t core;
static kalmanCoreParams_t params;
/* A second sensor-only PC attitude context. It shares unmodified algorithms,
 * not state/covariance, with the onboard context. No deck update reaches it. */
static kalmanCoreData_t pc_core;
static kalmanCoreParams_t pc_params;

void assertFail(char *exp, char *file, int line) {
  fprintf(stderr,"firmware assertion: %s %s:%d\n", exp, file, line);
  abort();
}

void bench_init(float x,float y,float z,float yaw) {
  memset(&core,0,sizeof(core));
  kalmanCoreDefaultParams(&params);
  params.initialX=x;params.initialY=y;params.initialZ=z;params.initialYaw=yaw;
  kalmanCoreInit(&core,&params,0);
}

void bench_predict(unsigned now,float ax,float ay,float az,float gx,float gy,float gz,int flying) {
  Axis3f acc={.axis={ax,ay,az}},gyro={.axis={gx,gy,gz}};
  kalmanCorePredict(&core,&params,&acc,&gyro,now,flying!=0);
}

void bench_process_noise(unsigned now) { kalmanCoreAddProcessNoise(&core,&params,now); }

void bench_tof(unsigned stamp,float distance,float std) {
  tofMeasurement_t t={.timestamp=stamp,.distance=distance,.stdDev=std};
  kalmanCoreUpdateWithTof(&core,&t);
}

void bench_flow(unsigned stamp,float x,float y,float dt,float stdx,float stdy,float gx,float gy,float gz) {
  flowMeasurement_t f={.timestamp=stamp,.dpixelx=x,.dpixely=y,.dt=dt,.stdDevX=stdx,.stdDevY=stdy};
  Axis3f gyro={.axis={gx,gy,gz}};
  kalmanCoreUpdateWithFlow(&core,&f,&gyro);
}

void bench_finalize(void) { kalmanCoreFinalize(&core); }
float bench_state(int i) { return core.S[i]; }
float bench_covariance(int i,int j) { return core.P[i][j]; }
float bench_rotation(int i,int j) { return core.R[i][j]; }
float bench_quaternion(int i) { return core.q[i]; }

void pc_bench_init(float x,float y,float z,float yaw) {
  memset(&pc_core,0,sizeof(pc_core));
  kalmanCoreDefaultParams(&pc_params);
  pc_params.initialX=x;pc_params.initialY=y;pc_params.initialZ=z;pc_params.initialYaw=yaw;
  kalmanCoreInit(&pc_core,&pc_params,0);
}
void pc_bench_predict(unsigned now,float ax,float ay,float az,float gx,float gy,float gz,int flying) {
  Axis3f acc={.axis={ax,ay,az}},gyro={.axis={gx,gy,gz}};
  kalmanCorePredict(&pc_core,&pc_params,&acc,&gyro,now,flying!=0);
}
void pc_bench_process_noise(unsigned now) { kalmanCoreAddProcessNoise(&pc_core,&pc_params,now); }
void pc_bench_finalize(void) { kalmanCoreFinalize(&pc_core); }
float pc_bench_state(int i) { return pc_core.S[i]; }
float pc_bench_covariance(int i,int j) { return pc_core.P[i][j]; }
float pc_bench_rotation(int i,int j) { return pc_core.R[i][j]; }
float pc_bench_quaternion(int i) { return pc_core.q[i]; }
