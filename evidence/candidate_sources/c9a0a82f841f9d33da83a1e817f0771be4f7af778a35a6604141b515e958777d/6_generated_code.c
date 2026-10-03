#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define D2R ((float)M_PI / 180.0f)
#define R2D (180.0f / (float)M_PI)
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static uint16_t pwmFromInt(int32_t v) { return (uint16_t)clampi(v, 0, 65535); }
static uint16_t roundPwm(float v) { if (v <= 0.0f) return 0; if (v >= 65535.0f) return 65535; return (uint16_t)(v + 0.5f); }

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 1.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
float twoKp = 0.8f, twoKi = 0.002f, beta = 0.01f, baseZacc = 0.0f;
bool sensfusion6IsInit = false, sensfusion6IsCalibrated = false;
PidObject pidRoll = {0}, pidPitch = {0}, pidYaw = {0};
PidObject pidRollRate = {0}, pidPitchRate = {0}, pidYawRate = {0};
bool thrustLocked = true, commanderModeSet = false;
SupervisorState supervisorState = supervisorStateLocked;
uint32_t supervisorConditionBits = 0;
TestState healthTestState = testDone;
uint8_t motorPass = 0, batteryPass = 0;
float batterySag = 0.0f;
StateEstimateLog stateEstimate = {0};
Axis3Log gyro = {0}, acc = {0};
BaroLog baro = {0};
MotorLog motor = {0};
Sensfusion6Log sensfusion6Log = {0};
SupervisorLog supervisorLog = {0};
HealthLog healthLog = {0};

static uint32_t hostTick;
static void syncFusionLog(void) { sensfusion6Log = (Sensfusion6Log){qw,qx,qy,qz,gravityX,gravityY,gravityZ,baseZacc,sensfusion6IsInit,sensfusion6IsCalibrated}; }
int16_t saturateSignedInt16(int32_t value) { return (int16_t)clampi(value, -32767, 32767); }
float capAngle(float angle_deg) { while (angle_deg > 180.0f) angle_deg -= 360.0f; while (angle_deg < -180.0f) angle_deg += 360.0f; return angle_deg; }
float invSqrt(float x) { union { float f; uint32_t u; } v; if (x <= 0.0f) return 0.0f; v.f = x; v.u = 0x5f3759dfU - (v.u >> 1); v.f = v.f * (1.5f - 0.5f * x * v.f * v.f); return v.f; }
void estimatedGravityDirection(float w, float x, float y, float z, float *gx, float *gy, float *gz) { if (gx) *gx = 2.0f * (x*z - w*y); if (gy) *gy = 2.0f * (w*x + y*z); if (gz) *gz = w*w - x*x - y*y + z*z; }
static void normalizeQuaternion(void) { float n = invSqrt(qw*qw + qx*qx + qy*qy + qz*qz); if (n > 0.0f) { qw*=n; qx*=n; qy*=n; qz*=n; } }
void sensfusion6Init(void) { if (sensfusion6IsInit) return; qw=1.0f; qx=qy=qz=0.0f; integralFBx=integralFBy=integralFBz=0.0f; gravityX=gravityY=0.0f; gravityZ=1.0f; baseZacc=0.0f; sensfusion6IsCalibrated=false; sensfusion6IsInit=true; syncFusionLog(); }
bool sensfusion6Test(void) { return sensfusion6IsInit; }
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  bool valid; float rawx=ax, rawy=ay, rawz=az;
  if (!sensfusion6IsInit) sensfusion6Init();
  if (dt <= 0.0f) return;
  gx*=D2R; gy*=D2R; gz*=D2R; valid = !(ax==0.0f && ay==0.0f && az==0.0f);
#if defined(CONFIG_IMU_MADGWICK_QUATERNION)
  if (valid) {
    float n=invSqrt(ax*ax+ay*ay+az*az), s1,s2,s3,s4;
    ax*=n; ay*=n; az*=n;
    s1=-2*qy*(2*(qx*qz-qw*qy)-ax)+2*qx*(2*(qw*qx+qy*qz)-ay);
    s2=2*qz*(2*(qx*qz-qw*qy)-ax)+2*qw*(2*(qw*qx+qy*qz)-ay)-4*qx*(1-2*(qx*qx+qy*qy)-az);
    s3=-2*qw*(2*(qx*qz-qw*qy)-ax)+2*qz*(2*(qw*qx+qy*qz)-ay)-4*qy*(1-2*(qx*qx+qy*qy)-az);
    s4=2*qx*(2*(qx*qz-qw*qy)-ax)+2*qy*(2*(qw*qx+qy*qz)-ay);
    n=invSqrt(s1*s1+s2*s2+s3*s3+s4*s4); if(n>0.0f){gx-=beta*s1*n;gy-=beta*s2*n;gz-=beta*s3*n;}
  }
  integralFBx=integralFBy=integralFBz=0.0f;
#else
  if (valid) {
    float n=invSqrt(ax*ax+ay*ay+az*az),hx,hy,hz,ex,ey,ez;
    ax*=n; ay*=n; az*=n; hx=qx*qz-qw*qy; hy=qw*qx+qy*qz; hz=qw*qw-0.5f+qz*qz;
    ex=ay*hz-az*hy; ey=az*hx-ax*hz; ez=ax*hy-ay*hx;
    if(twoKi>0.0f){integralFBx+=twoKi*ex*dt;integralFBy+=twoKi*ey*dt;integralFBz+=twoKi*ez*dt;gx+=integralFBx;gy+=integralFBy;gz+=integralFBz;} else integralFBx=integralFBy=integralFBz=0.0f;
    gx+=twoKp*ex; gy+=twoKp*ey; gz+=twoKp*ez;
  } else if(twoKi<=0.0f) integralFBx=integralFBy=integralFBz=0.0f;
#endif
  { float w=qw,x=qx,y=qy; qw+=(-x*gx-y*gy-qz*gz)*0.5f*dt; qx+=(w*gx+y*gz-qz*gy)*0.5f*dt; qy+=(w*gy-x*gz+qz*gx)*0.5f*dt; qz+=(w*gz+x*gy-y*gx)*0.5f*dt; }
  normalizeQuaternion(); estimatedGravityDirection(qw,qx,qy,qz,&gravityX,&gravityY,&gravityZ);
  if(!sensfusion6IsCalibrated && valid){baseZacc=rawx*gravityX+rawy*gravityY+rawz*gravityZ;sensfusion6IsCalibrated=true;}
  syncFusionLog();
}
void sensfusion6GetEulerRPY(float *r, float *p, float *y) { float sx,rr,pp,yy; estimatedGravityDirection(qw,qx,qy,qz,&gravityX,&gravityY,&gravityZ); rr=atan2f(2.0f*(qw*qx+qy*qz),1.0f-2.0f*(qx*qx+qy*qy))*R2D; sx=clampf(2.0f*(qw*qy-qz*qx),-1.0f,1.0f); pp=asinf(sx)*R2D; yy=atan2f(2.0f*(qw*qz+qx*qy),1.0f-2.0f*(qy*qy+qz*qz))*R2D; if(r)*r=rr;if(p)*p=pp;if(y)*y=yy;stateEstimate=(StateEstimateLog){rr,pp,yy,qx,qy,qz,qw};syncFusionLog(); }
void sensfusion6GetQuaternion(float *w,float *x,float *y,float *z){if(w)*w=qw;if(x)*x=qx;if(y)*y=qy;if(z)*z=qz;}
float sensfusion6GetAccZ(float ax,float ay,float az){estimatedGravityDirection(qw,qx,qy,qz,&gravityX,&gravityY,&gravityZ);syncFusionLog();return ax*gravityX+ay*gravityY+az*gravityZ;}
float sensfusion6GetAccZWithoutGravity(float ax,float ay,float az){return sensfusion6GetAccZ(ax,ay,az)-baseZacc;}

void powerDistributionLegacy(uint16_t t,int16_t r0,int16_t p0,int16_t y0,MotorPower *o){int32_t r,p,y;if(!o)return;r=r0/2;p=p0/2;y=y0;o->m1=(int32_t)t-r+p+y;o->m2=(int32_t)t-r-p-y;o->m3=(int32_t)t+r-p+y;o->m4=(int32_t)t+r+p-y;}
void powerDistributionForceTorque(float t,float tx,float ty,float tz,float len,float ratio,float f[4]){float tp,arm,r,p,y;if(!f)return;tp=.25f*t;arm=.707106781f*len;r=arm!=0.0f?.25f*tx/arm:0.0f;p=arm!=0.0f?.25f*ty/arm:0.0f;y=ratio!=0.0f?.25f*tz/ratio:0.0f;f[0]=tp-r+p+y;f[1]=tp-r-p-y;f[2]=tp+r-p+y;f[3]=tp+r+p-y;for(int i=0;i<4;i++)if(f[i]<0.0f)f[i]=0.0f;}
void powerDistributionForce(const float f[4],uint16_t p[4]){if(!f||!p)return;for(int i=0;i<4;i++)p[i]=roundPwm(clampf(f[i],0.0f,1.0f)*65535.0f);}
void powerDistribution(const ControlData *c,MotorPower *m){if(!c||!m)return;if(c->controlMode==controlModeLegacy)powerDistributionLegacy(c->thrust,c->roll,c->pitch,c->yaw,m);else if(c->controlMode==controlModeForceTorque){float f[4];uint16_t p[4];powerDistributionForceTorque(c->thrustSi,c->torque.x,c->torque.y,c->torque.z,CRAZYFLIE_ARM_LENGTH_M,CRAZYFLIE_THRUST_TO_TORQUE,f);for(int i=0;i<4;i++)f[i]=clampf(f[i]/CRAZYFLIE_MAX_MOTOR_FORCE_N,0.0f,1.0f);powerDistributionForce(f,p);m->m1=p[0];m->m2=p[1];m->m3=p[2];m->m4=p[3];}else if(c->controlMode==controlModeForce){uint16_t p[4];powerDistributionForce(c->normalizedForces,p);m->m1=p[0];m->m2=p[1];m->m3=p[2];m->m4=p[3];}}
int32_t capMinThrust(int32_t v,int32_t idle){return v<idle?idle:v;}
PowerCapResult powerDistributionCap(int32_t m[4],int32_t maxAllowed,int32_t idle){PowerCapResult r={false,0};int32_t mx;if(!m)return r;mx=m[0];for(int i=1;i<4;i++)if(m[i]>mx)mx=m[i];if(mx>maxAllowed){r.isCapped=true;r.reduction=mx-maxAllowed;for(int i=0;i<4;i++)m[i]=capMinThrust(m[i]-r.reduction,idle);}return r;}
float batteryCompensation(float s,float old,float a){return old+a*(s-old);}
uint16_t motorsCompensateBatteryVoltage(uint16_t t,float n,float a){return a<=0.0f?t:roundPwm(clampf((float)t*n/a,0.0f,65535.0f));}

static void pidInit(PidObject *p,float kp){if(!p->initialized){memset(p,0,sizeof(*p));p->kp=kp;p->initialized=true;}}
static float pidUpdate(PidObject *p,float actual,float desired,bool reset){float e=desired-actual;if(reset){p->integral=0.0f;p->prevError=e;}p->integral+=e;p->output=p->kp*e+p->ki*p->integral+p->kd*(e-p->prevError)+p->kff*desired;p->prevError=e;return p->output;}
void attitudeControllerInit(float dt){(void)dt;pidInit(&pidRoll,6.0f);pidInit(&pidPitch,6.0f);pidInit(&pidYaw,6.0f);pidInit(&pidRollRate,250.0f);pidInit(&pidPitchRate,250.0f);pidInit(&pidYawRate,120.0f);}
void attitudeControllerCorrectRatePID(float ra,float rd,float pa,float pd,float ya,float yd){attitudeControllerInit(0);pidUpdate(&pidRollRate,ra,rd,false);pidUpdate(&pidPitchRate,pa,pd,false);pidUpdate(&pidYawRate,ya,yd,false);pidRollRate.output=saturateSignedInt16((int32_t)pidRollRate.output);pidPitchRate.output=saturateSignedInt16((int32_t)pidPitchRate.output);pidYawRate.output=saturateSignedInt16((int32_t)pidYawRate.output);}
void attitudeControllerCorrectAttitudePID(float ra,float rd,float pa,float pd,float ya,float yd){attitudeControllerInit(0);pidUpdate(&pidRoll,ra,rd,false);pidUpdate(&pidPitch,pa,pd,false);pidUpdate(&pidYaw,ya,yd,true);}
void attitudeControllerResetAllPID(float r,float p,float y){PidObject *v[6]={&pidRoll,&pidPitch,&pidYaw,&pidRollRate,&pidPitchRate,&pidYawRate};for(int i=0;i<6;i++){v[i]->integral=0;v[i]->prevError=0;v[i]->output=0;}pidRoll.prevError=r;pidPitch.prevError=p;pidYaw.prevError=y;}
void attitudeControllerResetRollAttitudePID(float r){pidRoll.integral=0;pidRoll.prevError=r;pidRoll.output=0;}
void attitudeControllerResetPitchAttitudePID(float p){pidPitch.integral=0;pidPitch.prevError=p;pidPitch.output=0;}
void attitudeControllerGetActuatorOutput(int16_t *r,int16_t *p,int16_t *y){if(r)*r=saturateSignedInt16((int32_t)pidRollRate.output);if(p)*p=saturateSignedInt16((int32_t)pidPitchRate.output);if(y)*y=saturateSignedInt16((int32_t)pidYawRate.output);}
#if defined(__GNUC__)
__attribute__((weak))
#endif
uint16_t positionControllerUpdate(const Setpoint *sp,const State *st){if(!sp)return 0;if(!st)return sp->thrust;float e=sp->position.z-st->position.z;float v=sp->velocity.z-st->velocity.z;float out=(float)sp->thrust+1000.0f*e+100.0f*v;return (uint16_t)clampi((int32_t)out,0,65535);}
void controllerPid(const SensorData *s,const Setpoint *sp,const State *st,ControlData *c,float maxDelta,float dt){static float desiredYaw;static bool yawValid;float dr,dp,dy;uint16_t t;if(!s||!sp||!st||!c)return;attitudeControllerInit(dt);if(!yawValid){desiredYaw=st->attitude.yaw;yawValid=true;}dr=sp->attitude.roll;dp=sp->attitude.pitch;dy=sp->attitude.yaw;if(sp->mode.yaw==modeVelocity)dy=desiredYaw+sp->attitudeRate.yaw*dt;else if(sp->mode.quat==modeAbs){Quaternion q=sp->attitudeQuaternion;dy=atan2f(2.0f*(q.w*q.z+q.x*q.y),1.0f-2.0f*(q.y*q.y+q.z*q.z))*R2D;}if(maxDelta!=0.0f)dy=st->attitude.yaw+clampf(capAngle(dy-st->attitude.yaw),-maxDelta,maxDelta);desiredYaw=dy;t=sp->mode.z==modeDisable?sp->thrust:positionControllerUpdate(sp,st);if(!t){memset(c,0,sizeof(*c));attitudeControllerResetAllPID(st->attitude.roll,st->attitude.pitch,st->attitude.yaw);desiredYaw=st->attitude.yaw;return;}if(sp->mode.roll==modeVelocity){attitudeControllerResetRollAttitudePID(st->attitude.roll);pidRoll.output=sp->attitudeRate.roll;}else pidRoll.output=pidUpdate(&pidRoll,st->attitude.roll,dr,false);if(sp->mode.pitch==modeVelocity){attitudeControllerResetPitchAttitudePID(st->attitude.pitch);pidPitch.output=sp->attitudeRate.pitch;}else pidPitch.output=pidUpdate(&pidPitch,st->attitude.pitch,dp,false);pidYaw.output=pidUpdate(&pidYaw,st->attitude.yaw,dy,true);attitudeControllerCorrectRatePID(s->gyro.x,pidRoll.output,-s->gyro.y,pidPitch.output,s->gyro.z,pidYaw.output);memset(c,0,sizeof(*c));c->controlMode=controlModeLegacy;attitudeControllerGetActuatorOutput(&c->roll,&c->pitch,&c->yaw);c->yaw=(int16_t)-c->yaw;c->thrust=t;}

void rotateYaw(float r,float p,float d,float *rp,float *pp){float a=d*D2R,c=cosf(a),s=sinf(a);if(rp)*rp=r*c-p*s;if(pp)*pp=r*s+p*c;}
void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *v,Setpoint *sp,bool alt,bool posh,bool poss,StabilizationType rm,StabilizationType pm,StabilizationType ym,YawMode mode){uint16_t raw;float r,p;if(!v||!sp)return;raw=v->thrust;if(raw==0&&thrustLocked)thrustLocked=false;memset(sp,0,sizeof(*sp));r=v->roll;p=v->pitch;if(mode==PLUSMODE)rotateYaw(r,p,45.0f,&r,&p);if(mode==CAREFREE)return;if(alt){if(!commanderModeSet){commanderModeSet=true;}sp->mode.z=modeVelocity;sp->velocity.z=((float)raw-32767.0f)/32767.0f;return;}commanderModeSet=false;if(posh){sp->mode.x=sp->mode.y=modeVelocity;sp->mode.roll=sp->mode.pitch=modeDisable;sp->velocity.x=p/30.0f;sp->velocity.y=r/30.0f;sp->attitude.roll=sp->attitude.pitch=0.0f;}else if(poss&&raw!=0){sp->mode.x=sp->mode.y=sp->mode.z=modeAbs;sp->mode.roll=sp->mode.pitch=modeDisable;sp->mode.yaw=modeAbs;sp->position.x=-p;sp->position.y=r;sp->position.z=raw/1000.0f;sp->attitude.yaw=v->yaw;return;}else{sp->mode.roll=rm==RATE?modeVelocity:modeAbs;sp->mode.pitch=pm==RATE?modeVelocity:modeAbs;if(rm==RATE)sp->attitudeRate.roll=r;else sp->attitude.roll=r;if(pm==RATE)sp->attitudeRate.pitch=p;else sp->attitude.pitch=p;}sp->mode.yaw=ym==RATE?modeVelocity:modeAbs;if(ym==RATE)sp->attitudeRate.yaw=-v->yaw;else sp->attitude.yaw=v->yaw;sp->thrust=(thrustLocked||raw<MIN_THRUST)?0:(raw>MAX_THRUST?MAX_THRUST:raw);}

static SensorData supervisorSensors;static uint32_t supervisorRatios[4],supervisorIdle;static int32_t supervisorRPM[4];static float crashGs,freeGs,tiltAz,upsideAz;static uint32_t tiltMs,upsideMs,spinMs,spinStart,tiltStart,upsideStart,rpmStart;static bool tumbleEnabled,autoArm,armed,crashed,flying,tumbled,freeFalling;
void supervisorInit(void){static bool initialized;if(initialized)return;initialized=true;supervisorState=supervisorStateLocked;supervisorConditionBits=0;armed=crashed=flying=tumbled=freeFalling=false;spinStart=tiltStart=upsideStart=rpmStart=0;}
bool supervisorCanFly(void){return supervisorState==supervisorStateReadyToFly||supervisorState==supervisorStateFlying||supervisorState==supervisorStateWarningLevelOut||supervisorState==supervisorStateLanded;}
bool supervisorCanArm(void){return supervisorState==supervisorStatePreFlChecksPassed;}
bool supervisorIsArmed(void){return armed;} bool supervisorIsCrashed(void){return crashed;}
bool supervisorRequestArming(bool arm){if(!arm){armed=false;supervisorConditionBits&=~SUPERVISOR_CB_ARMED;return true;}if(supervisorCanArm()||supervisorState==supervisorStateArming||armed){armed=true;supervisorState=supervisorStateArming;supervisorConditionBits|=SUPERVISOR_CB_ARMED;return true;}return false;}
bool supervisorRequestCrashRecovery(bool recover){if(recover){if(tumbled)return false;crashed=false;supervisorConditionBits&=~SUPERVISOR_CB_CRASHED;return true;}crashed=true;supervisorConditionBits|=SUPERVISOR_CB_CRASHED;return true;}
bool supervisorAreMotorsAllowedToRun(void){return supervisorState==supervisorStateArming||supervisorCanFly();}
bool isFlyingCheck(const uint32_t m[4],uint32_t idle,uint32_t now){static bool seen;static uint32_t last;if(!m)return false;for(int i=0;i<4;i++)if(m[i]>idle){seen=true;last=now;return true;}return seen&&now-last<IS_FLYING_HYSTERESIS_THRESHOLD;}
bool isTumbledCheck(float ax,float ay,float az,float crash,float free,float tilt,float upside,uint32_t tm,uint32_t um,bool en,uint32_t now,bool *ffo){float n=sqrtf(ax*ax+ay*ay+az*az);bool ff=fabsf(ax)<free&&fabsf(ay)<free&&fabsf(az)<free;if(crash>0.0f&&fabsf(n-1.0f)>crash){crashed=true;supervisorConditionBits|=SUPERVISOR_CB_CRASHED;}if(ffo)*ffo=ff;if(ff){freeFalling=true;supervisorConditionBits|=SUPERVISOR_CB_FREE_FALL;return false;}freeFalling=false;supervisorConditionBits&=~SUPERVISOR_CB_FREE_FALL;if(!en){tiltStart=upsideStart=0;return false;}if(az<upside){if(!upsideStart)upsideStart=now;if(now-upsideStart>=um)tumbled=true;}else upsideStart=0;if(az<tilt){if(!tiltStart)tiltStart=now;if(now-tiltStart>=tm)tumbled=true;}else tiltStart=0;return tumbled;}
bool checkEmergencyStopWatchdog(uint32_t now,uint32_t last){return last==0||now-last<=DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;}
bool supervisorIsPreflightTimeout(SupervisorState st,uint32_t start,uint32_t now,uint32_t dur){return st==supervisorStateReadyToFly&&start!=0&&now-start>=dur;}
bool supervisorIsLandingTimeout(uint32_t start,uint32_t now,uint32_t dur){return start!=0&&now-start>=dur;}
uint32_t updateAndPopulateConditions(bool c,bool p,bool w){if(c||p||w)supervisorConditionBits|=SUPERVISOR_CB_EMERGENCY_STOP;else supervisorConditionBits&=~SUPERVISOR_CB_EMERGENCY_STOP;if(armed)supervisorConditionBits|=SUPERVISOR_CB_ARMED;if(flying)supervisorConditionBits|=SUPERVISOR_CB_IS_FLYING;else supervisorConditionBits&=~SUPERVISOR_CB_IS_FLYING;if(tumbled)supervisorConditionBits|=SUPERVISOR_CB_IS_TUMBLED;if(crashed)supervisorConditionBits|=SUPERVISOR_CB_CRASHED;return supervisorConditionBits;}
void supervisorOverrideSetpoint(Setpoint *sp,uint32_t bits,SupervisorState st){if(!sp)return;if(bits&(SUPERVISOR_CB_EMERGENCY_STOP|SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT|SUPERVISOR_CB_IS_TUMBLED|SUPERVISOR_CB_FREE_FALL|SUPERVISOR_CB_MOTORS_NOT_RESPONDING|SUPERVISOR_CB_CRASHED)){memset(sp,0,sizeof(*sp));return;}if(st==supervisorStateWarningLevelOut){sp->mode.x=sp->mode.y=modeDisable;sp->mode.roll=sp->mode.pitch=modeAbs;sp->mode.yaw=modeVelocity;sp->attitude.roll=sp->attitude.pitch=0;sp->attitudeRate.yaw=0;return;}if(st!=supervisorStateArming&&st!=supervisorStateReadyToFly&&st!=supervisorStateFlying&&st!=supervisorStateLanded)memset(sp,0,sizeof(*sp));}
bool isRPMatArmingValid(const int32_t r[4],int32_t mn,int32_t mx){if(!r)return false;for(int i=0;i<4;i++)if(r[i]<mn||r[i]>mx)return false;return true;}
bool isMotorsNotResponding(const int32_t r[4],int32_t th,uint32_t dur,bool can,uint32_t now){bool low=false;if(!can){rpmStart=0;return false;}if(!r)return true;for(int i=0;i<4;i++)if(r[i]<th)low=true;if(!low){rpmStart=0;return false;}if(!rpmStart)rpmStart=now;return now-rpmStart>=dur;}
void supervisorSetSensorData(const SensorData *s){if(s)supervisorSensors=*s;}void supervisorSetMotorRatios(const uint32_t r[4],uint32_t idle){if(r)memcpy(supervisorRatios,r,sizeof(supervisorRatios));supervisorIdle=idle;}void supervisorSetMotorRPMs(const int32_t r[4]){if(r)memcpy(supervisorRPM,r,sizeof(supervisorRPM));}
void supervisorConfigureSafety(float c,float f,float t,float u,uint32_t tm,uint32_t um,bool e){crashGs=c;freeGs=f;tiltAz=t;upsideAz=u;tiltMs=tm;upsideMs=um;tumbleEnabled=e;}void supervisorConfigureArming(bool a,uint32_t t){autoArm=a;spinMs=t;}
uint16_t supervisorGetInfoBitfield(void){uint16_t b=0;if(supervisorCanArm())b|=1U<<0;if(armed)b|=1U<<1;if(autoArm)b|=1U<<2;if(supervisorCanFly())b|=1U<<3;if(flying)b|=1U<<4;if(tumbled)b|=1U<<5;if(supervisorState==supervisorStateLocked)b|=1U<<6;if(crashed)b|=1U<<7;return b;}
void supervisorUpdate(uint32_t step){if(!RATE_DO_EXECUTE(RATE_SUPERVISOR,step))return;hostTick=step;flying=isFlyingCheck(supervisorRatios,supervisorIdle,hostTick);tumbled=isTumbledCheck(supervisorSensors.acc.x,supervisorSensors.acc.y,supervisorSensors.acc.z,crashGs,freeGs,tiltAz,upsideAz,tiltMs,upsideMs,tumbleEnabled,hostTick,&freeFalling);if(freeFalling)supervisorState=supervisorStateExceptFreeFall;if(autoArm&&supervisorState==supervisorStatePreFlChecksPassed)supervisorRequestArming(true);if(supervisorState==supervisorStateArming){if(!spinStart)spinStart=hostTick;if(spinMs&&hostTick-spinStart>=spinMs)supervisorConditionBits|=SUPERVISOR_CB_SPINUP_TIMEOUT;}else{spinStart=0;supervisorConditionBits&=~SUPERVISOR_CB_SPINUP_TIMEOUT;armed=false;supervisorConditionBits&=~SUPERVISOR_CB_ARMED;}if(tumbled)supervisorState=supervisorStateCrashed;supervisorLog.info=supervisorGetInfoBitfield();supervisorLog.accNorm=sqrtf(supervisorSensors.acc.x*supervisorSensors.acc.x+supervisorSensors.acc.y*supervisorSensors.acc.y+supervisorSensors.acc.z*supervisorSensors.acc.z);updateAndPopulateConditions(false,false,false);}

#define ESTIMATOR_FIFO 16
static EstimatorMeasurement estimatorQueue[ESTIMATOR_FIFO];static unsigned estimatorHead,estimatorTail,estimatorCount;static SensorData estimatorSensors;static State estimatorState;
bool estimatorEnqueue(const EstimatorMeasurement *m){if(!m||estimatorCount>=ESTIMATOR_FIFO)return false;estimatorQueue[estimatorTail]=*m;estimatorTail=(estimatorTail+1U)%ESTIMATOR_FIFO;estimatorCount++;return true;}
bool estimatorDequeue(EstimatorMeasurement *m){if(!m||!estimatorCount)return false;*m=estimatorQueue[estimatorHead];estimatorHead=(estimatorHead+1U)%ESTIMATOR_FIFO;estimatorCount--;return true;}
void estimatorComplementary(uint32_t step){EstimatorMeasurement m;while(estimatorDequeue(&m)){if(m.type==MeasurementTypeGyroscope)estimatorSensors.gyro=(Axis3f){m.data[0],m.data[1],m.data[2]};else if(m.type==MeasurementTypeAcceleration)estimatorSensors.acc=(Axis3f){m.data[0],m.data[1],m.data[2]};else if(m.type==MeasurementTypeBarometer){estimatorSensors.baroPressure=m.data[0];estimatorSensors.baroTemperature=m.data[1];estimatorSensors.baroAsl=m.data[2];}else estimatorSensors.tofRange=m.data[0];}if(RATE_DO_EXECUTE(RATE_250_HZ,step)){sensfusion6UpdateQ(estimatorSensors.gyro.x,estimatorSensors.gyro.y,estimatorSensors.gyro.z,estimatorSensors.acc.x,estimatorSensors.acc.y,estimatorSensors.acc.z,1.0f/250.0f);sensfusion6GetEulerRPY(&estimatorState.attitude.roll,&estimatorState.attitude.pitch,&estimatorState.attitude.yaw);estimatorState.attitudeQuaternion=(Quaternion){qw,qx,qy,qz};estimatorState.acc.z=sensfusion6GetAccZWithoutGravity(estimatorSensors.acc.x,estimatorSensors.acc.y,estimatorSensors.acc.z);estimatorState.velocity.z+=estimatorState.acc.z*9.81f/250.0f;}if(RATE_DO_EXECUTE(RATE_100_HZ,step))estimatorState.position.z+=estimatorState.velocity.z/100.0f;}
static Setpoint activeSetpoint,highLevelSetpoint;static int activePriority=COMMANDER_PRIORITY_LOWEST;static uint32_t lastSetpointTick;static bool highPending;
bool commanderSetSetpoint(const Setpoint *s,int priority){if(!s)return false;if(priority==COMMANDER_PRIORITY_DISABLE||priority>=activePriority){activeSetpoint=*s;activePriority=priority;lastSetpointTick=hostTick;return true;}return false;}void commanderRelaxPriority(void){activePriority=COMMANDER_PRIORITY_LOWEST;}uint32_t commanderGetInactivityTime(void){return hostTick-lastSetpointTick;}int commanderGetActivePriority(void){return activePriority;}
static uint32_t compressQuat(Quaternion q){uint32_t a=(uint32_t)clampf((q.x+1.0f)*511.0f,0,1023),b=(uint32_t)clampf((q.y+1.0f)*511.0f,0,1023),c=(uint32_t)clampf((q.z+1.0f)*511.0f,0,1023);return (a<<20)|(b<<10)|c;}
void stabilizerInit(void){static bool done;if(done)return;done=true;sensfusion6Init();attitudeControllerInit(1.0f/500.0f);crtpInit();supervisorInit();}
void stabilizerTask(void){ControlData c={0};MotorPower mp={0};int32_t mix[4];Setpoint sp;if(highPending){commanderSetSetpoint(&highLevelSetpoint,COMMANDER_PRIORITY_HIGHLEVEL);highPending=false;}hostTick++;estimatorComplementary(hostTick);supervisorUpdate(hostTick);sp=activeSetpoint;supervisorOverrideSetpoint(&sp,supervisorConditionBits,supervisorState);if(healthShallWeRunTest())healthRunTests(&estimatorSensors);else if(supervisorCanFly()&&supervisorAreMotorsAllowedToRun()){controllerPid(&estimatorSensors,&sp,&estimatorState,&c,0,1.0f/500.0f);powerDistribution(&c,&mp);mix[0]=mp.m1;mix[1]=mp.m2;mix[2]=mp.m3;mix[3]=mp.m4;powerDistributionCap(mix,65535,0);mp.m1=pwmFromInt(mix[0]);mp.m2=pwmFromInt(mix[1]);mp.m3=pwmFromInt(mix[2]);mp.m4=pwmFromInt(mix[3]);}motor=(MotorLog){pwmFromInt(mp.m1),pwmFromInt(mp.m2),pwmFromInt(mp.m3),pwmFromInt(mp.m4)};gyro=(Axis3Log){estimatorSensors.gyro.x,estimatorSensors.gyro.y,estimatorSensors.gyro.z};acc=(Axis3Log){estimatorSensors.acc.x,estimatorSensors.acc.y,estimatorSensors.acc.z};baro=(BaroLog){estimatorSensors.baroAsl,estimatorSensors.baroTemperature,estimatorSensors.baroPressure};}
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *s){if(!s)return false;highLevelSetpoint=*s;highPending=true;return true;}
void compressState(const State *s,const SensorData *n,CompressedState *o){if(!s||!n||!o)return;const float *p=&s->position.x,*v=&s->velocity.x;for(int i=0;i<3;i++){o->position_mm[i]=(int32_t)(p[i]*1000.0f);o->velocity_mms[i]=(int32_t)(v[i]*1000.0f);}o->acceleration_mms2[0]=(int32_t)(n->acc.x*9810.0f);o->acceleration_mms2[1]=(int32_t)(n->acc.y*9810.0f);o->acceleration_mms2[2]=(int32_t)((n->acc.z+1.0f)*9810.0f);o->gyro_millirad_s[0]=n->gyro.x*D2R*1000.0f;o->gyro_millirad_s[1]=-n->gyro.y*D2R*1000.0f;o->gyro_millirad_s[2]=n->gyro.z*D2R*1000.0f;o->quatCompressed=compressQuat(s->attitudeQuaternion);}
bool rateSupervisorValidate(uint32_t r){return r>=997U&&r<=1003U;}void rateSupervisorTask(void){if(!rateSupervisorValidate(1000U)&&fabsf(supervisorSensors.acc.x)+fabsf(supervisorSensors.acc.y)+fabsf(supervisorSensors.acc.z)>0.0f)supervisorConditionBits|=SUPERVISOR_CB_DECK_FAULT;}

static bool propRequest,batteryRequest;static float healthSamples[PROPTEST_NBR_OF_VARIANCE_VALUES],idleVoltage,minLoadedVoltage;static int healthSampleCount;static uint8_t healthMotor;static uint32_t healthStart;
void healthRequestPropTest(void){propRequest=true;}void healthRequestBatteryTest(void){batteryRequest=true;}
bool healthShallWeRunTest(void){if(propRequest){propRequest=false;healthTestState=configureAcc;healthSampleCount=0;healthMotor=0;motorPass=0;healthLog.motorTestCount=0;return true;}if(batteryRequest){batteryRequest=false;healthTestState=testBattery;healthStart=hostTick;minLoadedVoltage=idleVoltage;return true;}return healthTestState!=testDone;}
float variance(const float *b,int n){float sum=0,sq=0;if(!b||n<=0)return 0;for(int i=0;i<n;i++){sum+=b[i];sq+=b[i]*b[i];}return sq-sum*sum/(float)n;}
bool evaluatePropTest(float lo,float hi,float value,uint8_t m){bool ok;if(m>3)return false;if(hi==0.0f)ok=true;else ok=value>=lo&&value<=hi;if(ok)motorPass|=(uint8_t)(1U<<m);else healthLog.motorTestCount++;healthLog.motorPass=motorPass;return ok;}
void healthRunTests(const SensorData *s){float sample=s?s->acc.x+s->acc.y+s->acc.z:0;uint32_t elapsed=hostTick-healthStart;switch(healthTestState){case configureAcc:healthStart=hostTick;healthSampleCount=0;healthTestState=measureNoiseFloor;break;case measureNoiseFloor:if(healthSampleCount<(int)PROPTEST_NBR_OF_VARIANCE_VALUES)healthSamples[healthSampleCount++]=sample;if(healthSampleCount>=(int)PROPTEST_NBR_OF_VARIANCE_VALUES)healthTestState=measureProp;break;case measureProp:evaluatePropTest(0,0,fabsf(sample),healthMotor);healthLog.motorTestCount++;if(++healthMotor>=4)healthTestState=evaluatePropResult;break;case evaluatePropResult:healthTestState=testDone;healthLog.motorPass=motorPass;break;case testBattery:if(s&&s->baroPressure<minLoadedVoltage)minLoadedVoltage=s->baroPressure;if(elapsed>=50)healthTestState=evaluateBatResult;break;case evaluateBatResult:batterySag=idleVoltage-minLoadedVoltage;batteryPass=(batterySag<=0.5f);healthLog.batteryPass=batteryPass;healthLog.batterySag=batterySag;healthTestState=testDone;break;case restartBatTest:if(elapsed>=2000)healthTestState=testBattery;break;default:break;}}

typedef struct {CrtpPacket q[CRTP_RX_QUEUE_SIZE];unsigned head,tail,count;bool created;} RxQueue;static RxQueue rxq[CRTP_NBR_OF_PORTS];static CrtpPacket txq[CRTP_TX_QUEUE_SIZE];static unsigned txHead,txTail,txCount;static CrtpPortCallback callbacks[CRTP_NBR_OF_PORTS];static CrtpLink nopLink,*link;static uint32_t rxPackets,txPackets,statsTick,retryTick;static bool crtpError;
static bool queuePush(CrtpPacket *q,unsigned cap,unsigned *tail,unsigned *count,const CrtpPacket *p){if(*count>=cap)return false;q[*tail]=*p;*tail=(*tail+1U)%cap;(*count)++;return true;}
static bool queuePop(CrtpPacket *q,unsigned cap,unsigned *head,unsigned *count,CrtpPacket *p){if(!*count||!p)return false;*p=q[*head];*head=(*head+1U)%cap;(*count)--;return true;}
void crtpInit(void){static bool done;if(done)return;done=true;memset(rxq,0,sizeof(rxq));memset(callbacks,0,sizeof(callbacks));txHead=txTail=txCount=0;link=&nopLink;}
void crtpInitTaskQueue(uint8_t port){if(port>=CRTP_NBR_OF_PORTS){crtpError=true;return;}if(rxq[port].created){crtpError=true;return;}rxq[port].created=true;}
bool crtpSendPacket(const CrtpPacket *p){if(!p)return false;if(!queuePush(txq,CRTP_TX_QUEUE_SIZE,&txTail,&txCount,p))return false;return true;}bool crtpSendPacketBlock(const CrtpPacket *p){return crtpSendPacket(p);}
bool crtpReceivePacket(uint8_t port,CrtpPacket *p){if(port>=CRTP_NBR_OF_PORTS||!p||!rxq[port].created)return false;return queuePop(rxq[port].q,CRTP_RX_QUEUE_SIZE,&rxq[port].head,&rxq[port].count,p);}bool crtpReceivePacketBlock(uint8_t port,CrtpPacket *p){if(crtpReceivePacket(port,p))return true;crtpRxTask();return crtpReceivePacket(port,p);}bool crtpReceivePacketWait(uint8_t port,CrtpPacket *p,uint32_t wait){for(uint32_t i=0;i<=wait;i++){if(crtpReceivePacket(port,p))return true;crtpRxTask();}return false;}
void crtpRxTask(void){CrtpPacket p;if(link&&link->receivePacket&&link->receivePacket(&p)){rxPackets++;if(p.port<CRTP_NBR_OF_PORTS){if(rxq[p.port].created)queuePush(rxq[p.port].q,CRTP_RX_QUEUE_SIZE,&rxq[p.port].tail,&rxq[p.port].count,&p);if(callbacks[p.port])callbacks[p.port](&p);}}}
void crtpTxTask(void){CrtpPacket p;if(!link||!link->sendPacket||!txCount||hostTick<retryTick)return;p=txq[txHead];if(link->sendPacket(&p)){txHead=(txHead+1U)%CRTP_TX_QUEUE_SIZE;txCount--;txPackets++;retryTick=0;}else retryTick=hostTick+10U;}
void crtpSetLink(CrtpLink *l){if(link&&link->setEnable)link->setEnable(false);link=l?l:&nopLink;if(link->setEnable)link->setEnable(true);}void crtpReset(void){txHead=txTail=txCount=0;retryTick=0;if(link&&link->reset)link->reset();}bool crtpIsConnected(void){return link&&link->isConnected?link->isConnected():true;}uint32_t crtpGetFreeTxQueuePackets(void){return CRTP_TX_QUEUE_SIZE-txCount;}void crtpRegisterPortCB(uint8_t port,CrtpPortCallback f){if(port<CRTP_NBR_OF_PORTS)callbacks[port]=f;}void updateStats(void){if(hostTick-statsTick>=500U){rxPackets=txPackets=0;statsTick=hostTick;}}
uint8_t deckDiscovery(DeckInfo *d,uint8_t cap){uint8_t n=0;if(!d||!cap)return 0;for(uint8_t i=0;i<cap;i++){bool duplicate=false;for(uint8_t j=0;j<n;j++){if((d[i].foundByI2C&&d[j].foundByI2C&&d[i].i2cAddress==d[j].i2cAddress)||(d[i].foundByOneWire&&d[j].foundByOneWire&&d[i].oneWireRomId==d[j].oneWireRomId)){duplicate=true;break;}}if(!duplicate)n++;}return n;}