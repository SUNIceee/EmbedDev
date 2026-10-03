#include "6_generated_code.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)
static int32_t clampi(int32_t v,int32_t lo,int32_t hi){return v<lo?lo:v>hi?hi:v;}
static float clampf(float v,float lo,float hi){return v<lo?lo:v>hi?hi:v;}
static uint16_t pwm(int32_t v){return (uint16_t)clampi(v,0,65535);}
float qw=1.0f,qx=0.0f,qy=0.0f,qz=0.0f,gravityX=0.0f,gravityY=0.0f,gravityZ=1.0f;
float integralFBx=0.0f,integralFBy=0.0f,integralFBz=0.0f,twoKp=0.8f,twoKi=0.002f,beta=0.01f,baseZacc=0.0f;
bool sensfusion6IsInit=false,sensfusion6IsCalibrated=false;
PidObject pidRoll,pidPitch,pidYaw,pidRollRate,pidPitchRate,pidYawRate;
bool thrustLocked=false,commanderModeSet=false;
SupervisorState supervisorState=supervisorStateLocked;
uint32_t supervisorConditionBits=0;
TestState healthTestState=testDone;
uint8_t motorPass=0,batteryPass=0;
float batterySag=0.0f;
StateEstimateLog stateEstimate;
Axis3Log gyro,acc;
BaroLog baro;
MotorLog motor;
Sensfusion6Log sensfusion6Log;
SupervisorLog supervisorLog;
HealthLog healthLog;
static uint32_t hostTick=0,commanderLastUpdate=0,lastFlightTick=0,armingStartTick=0,tumbleStartTick=0,motorBadStartTick=0,rateTick=0;
static bool fusionReady=false,supervisorReady=false,armed=false,crashed=false,autoArm=false,flightSeen=false,tiltStarted=false,motorBadStarted=false;
static bool armingStarted=false,tumbleEnabled=true;
static uint32_t spinupDuration=0,motorBadDuration=0;
static float crashGs=0,freeFallLimit=0,tiltLimit=0,upsideLimit=0;
static uint32_t tiltDuration=0,upsideDuration=0;
static float pidDt=0.002f,desiredYaw=0;
static int commanderPriority=COMMANDER_PRIORITY_LOWEST;
static Setpoint activeSetpoint;
static SensorData supervisorSensors,estSensors;
static uint32_t motorRatios[4],idleRatio=0;
static int32_t motorRPMs[4];
static State estimatorState;
static EstimatorMeasurement estimatorQueue[16];
static uint8_t estimatorHead=0,estimatorTail=0,estimatorCount=0;
static bool highPending=false;
static Setpoint highSetpoint;
static bool propRequest=false,batRequest=false;
static float healthSamples[PROPTEST_NBR_OF_VARIANCE_VALUES];
static int healthSampleCount=0;
static uint8_t healthMotor=0;
static uint32_t healthTick=0;
static float idleVoltage=4.2f,minLoadedVoltage=4.2f;
static uint32_t motorTestCount=0;
static void syncFusionLog(void){sensfusion6Log.qw=qw;sensfusion6Log.qx=qx;sensfusion6Log.qy=qy;sensfusion6Log.qz=qz;sensfusion6Log.gravityX=gravityX;sensfusion6Log.gravityY=gravityY;sensfusion6Log.gravityZ=gravityZ;sensfusion6Log.accZbase=baseZacc;sensfusion6Log.isInit=sensfusion6IsInit;sensfusion6Log.isCalibrated=sensfusion6IsCalibrated;}
int16_t saturateSignedInt16(int32_t v){return (int16_t)clampi(v,-32767,32767);}
float capAngle(float a){while(a>180.0f)a-=360.0f;while(a<-180.0f)a+=360.0f;return a;}
float invSqrt(float x){if(x<=0.0f)return 0.0f;union{float f;uint32_t i;}u;u.f=x;u.i=0x5f3759dfU-(u.i>>1);u.f=u.f*(1.5f-0.5f*x*u.f*u.f);return u.f;}
void estimatedGravityDirection(float w,float x,float y,float z,float *gx,float *gy,float *gz){if(gx)*gx=2.0f*(x*z-w*y);if(gy)*gy=2.0f*(w*x+y*z);if(gz)*gz=w*w-x*x-y*y+z*z;}
static void normalizeQuaternion(void){float n=sqrtf(qw*qw+qx*qx+qy*qy+qz*qz);if(n>0.0f){qw/=n;qx/=n;qy/=n;qz/=n;}else{qw=1.0f;qx=qy=qz=0.0f;}estimatedGravityDirection(qw,qx,qy,qz,&gravityX,&gravityY,&gravityZ);}
void sensfusion6Init(void){if(sensfusion6IsInit)return;qw=1.0f;qx=qy=qz=0.0f;gravityX=gravityY=0.0f;gravityZ=1.0f;integralFBx=integralFBy=integralFBz=0.0f;baseZacc=0.0f;sensfusion6IsCalibrated=false;sensfusion6IsInit=true;syncFusionLog();}
bool sensfusion6Test(void){return sensfusion6IsInit;}
void sensfusion6UpdateQ(float gx,float gy,float gz,float ax,float ay,float az,float dt){sensfusion6Init();if(dt<0.0f)dt=0.0f;bool valid=(ax!=0.0f||ay!=0.0f||az!=0.0f);gx*=D2R;gy*=D2R;gz*=D2R;
#ifdef CONFIG_IMU_MADGWICK_QUATERNION
integralFBx=integralFBy=integralFBz=0.0f;
#else
if(twoKi==0.0f){integralFBx=integralFBy=integralFBz=0.0f;}if(valid){float n=ax*ax+ay*ay+az*az;if(n>0.0f){float r=invSqrt(n);ax*=r;ay*=r;az*=r;float ex,ey,ez;estimatedGravityDirection(qw,qx,qy,qz,&gravityX,&gravityY,&gravityZ);ex=ay*gravityZ-az*gravityY;ey=az*gravityX-ax*gravityZ;ez=ax*gravityY-ay*gravityX;if(twoKi>0.0f){integralFBx+=twoKi*ex*dt;integralFBy+=twoKi*ey*dt;integralFBz+=twoKi*ez*dt;gx+=integralFBx;gy+=integralFBy;gz+=integralFBz;}gx+=twoKp*ex;gy+=twoKp*ey;gz+=twoKp*ez;}}}
#endif
float h=0.5f*dt,ow=qw,ox=qx,oy=qy,oz=qz;qw+=(-ox*gx-oy*gy-oz*gz)*h;qx+=(ow*gx+oy*gz-oz*gy)*h;qy+=(ow*gy-ox*gz+oz*gx)*h;qz+=(ow*gz+ox*gy-oy*gx)*h;normalizeQuaternion();if(valid&&!sensfusion6IsCalibrated){baseZacc=ax*gravityX+ay*gravityY+az*gravityZ;sensfusion6IsCalibrated=true;}syncFusionLog();}
void sensfusion6GetEulerRPY(float *r,float *p,float *y){normalizeQuaternion();float gx=gravityX;float roll=atan2f(2.0f*(qw*qx+qy*qz),1.0f-2.0f*(qx*qx+qy*qy))*R2D;float pitch=asinf(clampf(2.0f*(qw*qy-qz*qx),-1.0f,1.0f))*R2D;float yaw=capAngle(atan2f(2.0f*(qw*qz+qx*qy),1.0f-2.0f*(qy*qy+qz*qz))*R2D);(void)gx;if(r)*r=roll;if(p)*p=pitch;if(y)*y=yaw;syncFusionLog();}
void sensfusion6GetQuaternion(float *w,float *x,float *y,float *z){if(w)*w=qw;if(x)*x=qx;if(y)*y=qy;if(z)*z=qz;}
float sensfusion6GetAccZ(float x,float y,float z){normalizeQuaternion();return x*gravityX+y*gravityY+z*gravityZ;}
float sensfusion6GetAccZWithoutGravity(float x,float y,float z){return sensfusion6GetAccZ(x,y,z)-baseZacc;}
void powerDistributionLegacy(uint16_t t,int16_t r,int16_t p,int16_t y,MotorPower *o){if(!o)return;int32_t rr=(int32_t)r/2,pp=(int32_t)p/2;o->m1=(int32_t)t-rr+pp+y;o->m2=(int32_t)t-rr-pp-y;o->m3=(int32_t)t+rr-pp+y;o->m4=(int32_t)t+rr+pp-y;}
void powerDistributionForceTorque(float t,float x,float y,float z,float arm,float k,float m[4]){if(!m)return;float a=0.707106781f*arm,r=a!=0.0f?0.25f*x/a:0.0f,p=a!=0.0f?0.25f*y/a:0.0f,w=k!=0.0f?0.25f*z/k:0.0f,q=0.25f*t;m[0]=fmaxf(0.0f,q-r+p+w);m[1]=fmaxf(0.0f,q-r-p-w);m[2]=fmaxf(0.0f,q+r-p+w);m[3]=fmaxf(0.0f,q+r+p-w);}
void powerDistributionForce(const float f[4],uint16_t p[4]){if(!f||!p)return;for(int i=0;i<4;i++)p[i]=(uint16_t)lroundf(clampf(f[i],0.0f,1.0f)*65535.0f);}
void powerDistribution(const ControlData *c,MotorPower *m){if(!c||!m)return;if(c->controlMode==controlModeLegacy)powerDistributionLegacy(c->thrust,c->roll,c->pitch,c->yaw,m);else if(c->controlMode==controlModeForce){uint16_t p[4];powerDistributionForce(c->normalizedForces,p);m->m1=p[0];m->m2=p[1];m->m3=p[2];m->m4=p[3];}else if(c->controlMode==controlModeForceTorque){float f[4];powerDistributionForceTorque(c->thrustSi,c->torque.x,c->torque.y,c->torque.z,CRAZYFLIE_ARM_LENGTH_M,CRAZYFLIE_THRUST_TO_TORQUE,f);m->m1=(int32_t)lroundf(f[0]/CRAZYFLIE_MAX_MOTOR_FORCE_N*65535.0f);m->m2=(int32_t)lroundf(f[1]/CRAZYFLIE_MAX_MOTOR_FORCE_N*65535.0f);m->m3=(int32_t)lroundf(f[2]/CRAZYFLIE_MAX_MOTOR_FORCE_N*65535.0f);m->m4=(int32_t)lroundf(f[3]/CRAZYFLIE_MAX_MOTOR_FORCE_N*65535.0f);}}
int32_t capMinThrust(int32_t v,int32_t idle){return v<idle?idle:v;}
PowerCapResult powerDistributionCap(int32_t m[4],int32_t max,int32_t idle){PowerCapResult r={false,0};if(!m)return r;int32_t high=m[0];for(int i=1;i<4;i++)if(m[i]>high)high=m[i];if(high>max){r.isCapped=true;r.reduction=high-max;for(int i=0;i<4;i++)m[i]=capMinThrust(m[i]-r.reduction,idle);}return r;}
float batteryCompensation(float s,float o,float a){return o+a*(s-o);}
uint16_t motorsCompensateBatteryVoltage(uint16_t t,float n,float a){if(a<=0.0f)return t;return pwm((int32_t)lroundf((float)t*n/a));}
static void pidInit(PidObject *p,float kp){memset(p,0,sizeof(*p));p->kp=kp;p->initialized=true;}
static float pidUpdate(PidObject *p,float actual,float desired,bool reset){float e=desired-actual;float d=pidDt>0.0f?(e-p->prevError)/pidDt:0.0f;if(reset)p->integral=0.0f;p->integral+=e*pidDt;p->prevError=e;p->output=p->kp*e+p->ki*p->integral+p->kd*d+p->kff*desired;return p->output;}
static void pidReset(PidObject *p,float actual){if(!p)return;p->integral=0.0f;p->prevError=actual;p->output=0.0f;}
void attitudeControllerInit(float d){if(fusionReady)return;if(d>0.0f)pidDt=d;pidInit(&pidRoll,4.0f);pidInit(&pidPitch,4.0f);pidInit(&pidYaw,4.0f);pidInit(&pidRollRate,1.0f);pidInit(&pidPitchRate,1.0f);pidInit(&pidYawRate,1.0f);fusionReady=true;}
void attitudeControllerCorrectRatePID(float ra,float rd,float pa,float pd,float ya,float yd){attitudeControllerInit(pidDt);pidRollRate.output=saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidRollRate,ra,rd,false)));pidPitchRate.output=saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidPitchRate,pa,pd,false)));pidYawRate.output=saturateSignedInt16((int32_t)lroundf(pidUpdate(&pidYawRate,ya,yd,false)));}
void attitudeControllerCorrectAttitudePID(float ra,float rd,float pa,float pd,float ya,float yd){pidUpdate(&pidRoll,ra,rd,false);pidUpdate(&pidPitch,pa,pd,false);pidUpdate(&pidYaw,ya,yd,true);}
void attitudeControllerResetAllPID(float r,float p,float y){pidReset(&pidRoll,r);pidReset(&pidPitch,p);pidReset(&pidYaw,y);pidReset(&pidRollRate,0);pidReset(&pidPitchRate,0);pidReset(&pidYawRate,0);}
void attitudeControllerResetRollAttitudePID(float r){pidReset(&pidRoll,r);}
void attitudeControllerResetPitchAttitudePID(float p){pidReset(&pidPitch,p);}
void attitudeControllerGetActuatorOutput(int16_t *r,int16_t *p,int16_t *y){if(r)*r=saturateSignedInt16((int32_t)lroundf(pidRollRate.output));if(p)*p=saturateSignedInt16((int32_t)lroundf(pidPitchRate.output));if(y)*y=saturateSignedInt16((int32_t)lroundf(pidYawRate.output));}
uint16_t positionControllerUpdate(const Setpoint *s,const State *st){if(!s||!st)return 0;float e=(s->position.z-st->position.z)*10000.0f+(s->velocity.z-st->velocity.z)*1000.0f;return pwm((int32_t)s->thrust+(int32_t)lroundf(e));}
void controllerPid(const SensorData *s,const Setpoint *sp,const State *st,ControlData *c,float lim,float dt){if(!s||!sp||!st||!c)return;memset(c,0,sizeof(*c));c->controlMode=controlModeLegacy;attitudeControllerInit(dt);if(sp->thrust==0){attitudeControllerResetAllPID(st->attitude.roll,st->attitude.pitch,st->attitude.yaw);desiredYaw=st->attitude.yaw;return;}float rd=sp->attitude.roll,pd=sp->attitude.pitch;if(sp->mode.yaw==modeVelocity)desiredYaw=capAngle(desiredYaw+sp->attitudeRate.yaw*dt);else if(sp->mode.quat==modeAbs)desiredYaw=atan2f(2.0f*(sp->attitudeQuaternion.w*sp->attitudeQuaternion.z+sp->attitudeQuaternion.x*sp->attitudeQuaternion.y),1.0f-2.0f*(sp->attitudeQuaternion.y*sp->attitudeQuaternion.y+sp->attitudeQuaternion.z*sp->attitudeQuaternion.z))*R2D;else if(sp->mode.yaw==modeAbs)desiredYaw=sp->attitude.yaw;float yd=desiredYaw;if(lim!=0.0f)yd=capAngle(st->attitude.yaw+clampf(capAngle(yd-st->attitude.yaw),-lim,lim));if(sp->mode.roll==modeVelocity){pidRoll.output=sp->attitudeRate.roll;attitudeControllerResetRollAttitudePID(st->attitude.roll);}if(sp->mode.pitch==modeVelocity){pidPitch.output=sp->attitudeRate.pitch;attitudeControllerResetPitchAttitudePID(st->attitude.pitch);}if(sp->mode.roll!=modeVelocity||sp->mode.pitch!=modeVelocity)attitudeControllerCorrectAttitudePID(st->attitude.roll,rd,st->attitude.pitch,pd,st->attitude.yaw,yd);attitudeControllerCorrectRatePID(s->gyro.x,sp->mode.roll==modeVelocity?sp->attitudeRate.roll:pidRoll.output,-s->gyro.y,sp->mode.pitch==modeVelocity?sp->attitudeRate.pitch:pidPitch.output,s->gyro.z,sp->mode.yaw==modeVelocity?sp->attitudeRate.yaw:pidYaw.output);attitudeControllerGetActuatorOutput(&c->roll,&c->pitch,&c->yaw);c->yaw=(int16_t)-c->yaw;c->thrust=sp->mode.z==modeDisable?sp->thrust:positionControllerUpdate(sp,st);}
void rotateYaw(float r,float p,float y,float *rp,float *pp){float a=y*D2R;if(rp)*rp=r*cosf(a)-p*sinf(a);if(pp)*pp=r*sinf(a)+p*cosf(a);}
void crtpCommanderRpytDecodeSetpoint(const CommanderCrtpLegacyValues *v,Setpoint *s,bool alt,bool ph,bool ps,StabilizationType sr,StabilizationType sp,StabilizationType sy,YawMode ym){if(!v||!s)return;memset(s,0,sizeof(*s));if(v->thrust==0)thrustLocked=false;if(alt){s->mode.z=modeVelocity;s->velocity.z=((float)v->thrust-32767.0f)/32767.0f;commanderModeSet=true;s->thrust=0;}else{s->mode.z=modeDisable;commanderModeSet=false;s->thrust=(thrustLocked||v->thrust<MIN_THRUST)?0:(v->thrust>MAX_THRUST?MAX_THRUST:v->thrust);}float r=v->roll,p=v->pitch;if(ym==PLUSMODE)rotateYaw(r,p,45.0f,&r,&p);else if(ym==CAREFREE)supervisorConditionBits|=SUPERVISOR_CB_DECK_FAULT;if(ph){s->mode.x=s->mode.y=modeVelocity;s->mode.roll=s->mode.pitch=modeDisable;s->velocity.x=p/30.0f;s->velocity.y=r/30.0f;s->attitude.roll=s->attitude.pitch=0;return;}if(ps&&v->thrust!=0){s->mode.x=s->mode.y=s->mode.z=modeAbs;s->mode.roll=s->mode.pitch=modeDisable;s->mode.yaw=modeAbs;s->position.x=-p;s->position.y=r;s->position.z=v->thrust/1000.0f;s->attitude.yaw=v->yaw;s->thrust=0;return;}s->mode.roll=sr==RATE?modeVelocity:modeAbs;s->mode.pitch=sp==RATE?modeVelocity:modeAbs;s->mode.yaw=sy==RATE?modeVelocity:modeAbs;if(sr==RATE)s->attitudeRate.roll=r;else s->attitude.roll=r;if(sp==RATE)s->attitudeRate.pitch=p;else s->attitude.pitch=p;if(sy==RATE)s->attitudeRate.yaw=-v->yaw;else s->attitude.yaw=v->yaw;}
void supervisorInit(void){if(supervisorReady)return;supervisorState=supervisorStateLocked;supervisorConditionBits=0;armed=false;crashed=false;flightSeen=false;armingStarted=false;motorBadStarted=false;supervisorReady=true;}
bool supervisorCanFly(void){return supervisorState==supervisorStateReadyToFly||supervisorState==supervisorStateFlying||supervisorState==supervisorStateWarningLevelOut||supervisorState==supervisorStateLanded;}
bool supervisorCanArm(void){return supervisorState==supervisorStatePreFlChecksPassed;}
bool supervisorIsArmed(void){return armed;}
bool supervisorIsCrashed(void){return crashed;}
static bool allowedState(SupervisorState s){return s==supervisorStateArming||s==supervisorStateReadyToFly||s==supervisorStateFlying||s==supervisorStateWarningLevelOut||s==supervisorStateLanded;}
bool supervisorRequestArming(bool on){if(!on){armed=false;armingStarted=false;supervisorConditionBits&=~SUPERVISOR_CB_ARMED;return true;}if(armed)return true;if(!supervisorCanArm())return false;armed=true;armingStarted=true;armingStartTick=hostTick;supervisorState=supervisorStateArming;supervisorConditionBits|=SUPERVISOR_CB_ARMED;return true;}
bool supervisorRequestCrashRecovery(bool rec){if(rec){if(supervisorState==supervisorStateExceptFreeFall||(supervisorConditionBits&SUPERVISOR_CB_IS_TUMBLED))return false;crashed=false;supervisorConditionBits&=~SUPERVISOR_CB_CRASHED;return true;}crashed=true;supervisorState=supervisorStateCrashed;supervisorConditionBits|=SUPERVISOR_CB_CRASHED;return true;}
bool supervisorAreMotorsAllowedToRun(void){return allowedState(supervisorState);}
uint16_t supervisorGetInfoBitfield(void){uint16_t b=0;if(supervisorCanArm())b|=1u;if(armed)b|=2u;if(autoArm)b|=4u;if(supervisorCanFly())b|=8u;if(supervisorConditionBits&SUPERVISOR_CB_IS_FLYING)b|=16u;if(supervisorConditionBits&SUPERVISOR_CB_IS_TUMBLED)b|=32u;if(supervisorState==supervisorStateLocked)b|=64u;if(crashed)b|=128u;return b;}
bool isFlyingCheck(const uint32_t *m,uint32_t idle,uint32_t t){if(!m)return false;for(int i=0;i<4;i++)if(m[i]>idle){lastFlightTick=t;flightSeen=true;return true;}return flightSeen&&(uint32_t)(t-lastFlightTick)<IS_FLYING_HYSTERESIS_THRESHOLD;}
bool isTumbledCheck(float x,float y,float z,float cg,float ff,float tz,float uz,uint32_t tm,uint32_t um,bool en,uint32_t t,bool *free){bool falling=ff>0.0f&&fabsf(x)<ff&&fabsf(y)<ff&&fabsf(z)<ff;if(free)*free=falling;if(cg>0.0f&&fabsf(sqrtf(x*x+y*y+z*z)-1.0f)>cg){crashed=true;supervisorConditionBits|=SUPERVISOR_CB_CRASHED;}if(!en){supervisorConditionBits&=~SUPERVISOR_CB_IS_TUMBLED;return false;}if(falling){supervisorConditionBits|=SUPERVISOR_CB_FREE_FALL;supervisorState=supervisorStateExceptFreeFall;tumbleStartTick=0;return false;}supervisorConditionBits&=~SUPERVISOR_CB_FREE_FALL;if(z<tz){if(!tiltStarted){tiltStarted=true;tumbleStartTick=t;}uint32_t d=z<uz?um:tm;if((uint32_t)(t-tumbleStartTick)>=d)supervisorConditionBits|=SUPERVISOR_CB_IS_TUMBLED;}else{tiltStarted=false;tumbleStartTick=0;supervisorConditionBits&=~SUPERVISOR_CB_IS_TUMBLED;}return (supervisorConditionBits&SUPERVISOR_CB_IS_TUMBLED)!=0;}
bool checkEmergencyStopWatchdog(uint32_t t,uint32_t n){return n==0||(uint32_t)(t-n)<=DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT;}
bool supervisorIsPreflightTimeout(SupervisorState s,uint32_t a,uint32_t t,uint32_t d){return s==supervisorStatePreFlChecksPassed&&a!=0&&(uint32_t)(t-a)>=d;}
bool supervisorIsLandingTimeout(uint32_t a,uint32_t t,uint32_t d){return a!=0&&(uint32_t)(t-a)>=d;}
uint32_t updateAndPopulateConditions(bool c,bool p,bool w){if(c||p)supervisorConditionBits|=SUPERVISOR_CB_EMERGENCY_STOP;else supervisorConditionBits&=~SUPERVISOR_CB_EMERGENCY_STOP;if(w)supervisorConditionBits|=SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;else supervisorConditionBits&=~SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;return supervisorConditionBits;}
void supervisorOverrideSetpoint(Setpoint *s,uint32_t bits,SupervisorState st){if(!s)return;if(st==supervisorStateWarningLevelOut){s->mode.x=s->mode.y=modeDisable;s->mode.roll=s->mode.pitch=modeAbs;s->mode.yaw=modeVelocity;s->attitude.roll=s->attitude.pitch=0;s->attitudeRate.yaw=0;return;}if(allowedState(st)&&bits==0)return;memset(s,0,sizeof(*s));}
bool isRPMatArmingValid(const int32_t *r,int32_t lo,int32_t hi){if(!r)return false;for(int i=0;i<4;i++)if(r[i]<lo||r[i]>hi)return false;return true;}
bool isMotorsNotResponding(const int32_t *r,int32_t th,uint32_t d,bool can,uint32_t t){if(!can||!r){motorBadStarted=false;return false;}bool low=false;for(int i=0;i<4;i++)if(r[i]<th)low=true;if(!low){motorBadStarted=false;return false;}if(!motorBadStarted){motorBadStarted=true;motorBadStartTick=t;}return (uint32_t)(t-motorBadStartTick)>=d;}
void supervisorSetSensorData(const SensorData *s){if(s)supervisorSensors=*s;}
void supervisorSetMotorRatios(const uint32_t *m,uint32_t idle){if(m)memcpy(motorRatios,m,sizeof(motorRatios));idleRatio=idle;}
void supervisorSetMotorRPMs(const int32_t *r){if(r)memcpy(motorRPMs,r,sizeof(motorRPMs));}
void supervisorConfigureSafety(float c,float f,float t,float u,uint32_t mt,uint32_t mu,bool e){crashGs=c;freeFallLimit=f;tiltLimit=t;upsideLimit=u;tiltDuration=mt;upsideDuration=mu;tumbleEnabled=e;}
void supervisorConfigureArming(bool a,uint32_t d){autoArm=a;spinupDuration=d;}
void supervisorUpdate(uint32_t step){if(!RATE_DO_EXECUTE(RATE_SUPERVISOR,step))return;uint32_t now=hostTick;bool ff=false;bool tumb=isTumbledCheck(supervisorSensors.acc.x,supervisorSensors.acc.y,supervisorSensors.acc.z,crashGs,freeFallLimit,tiltLimit,upsideLimit,tiltDuration,upsideDuration,tumbleEnabled,now,&ff);if(tumb)supervisorConditionBits|=SUPERVISOR_CB_IS_TUMBLED;if(isFlyingCheck(motorRatios,idleRatio,now))supervisorConditionBits|=SUPERVISOR_CB_IS_FLYING;else supervisorConditionBits&=~SUPERVISOR_CB_IS_FLYING;if(ff)supervisorConditionBits|=SUPERVISOR_CB_FREE_FALL;if(autoArm&&supervisorState==supervisorStatePreFlChecksPassed)supervisorRequestArming(true);bool bad=isMotorsNotResponding(motorRPMs,0,motorBadDuration,supervisorCanFly(),now);if(bad)supervisorConditionBits|=SUPERVISOR_CB_MOTORS_NOT_RESPONDING;else if(!supervisorCanFly())supervisorConditionBits&=~SUPERVISOR_CB_MOTORS_NOT_RESPONDING;if(supervisorState==supervisorStateArming&&armingStarted&&spinupDuration>0&&(uint32_t)(now-armingStartTick)>=spinupDuration)supervisorConditionBits|=SUPERVISOR_CB_SPINUP_TIMEOUT;else if(supervisorState!=supervisorStateArming){armingStarted=false;armingStartTick=0;supervisorConditionBits&=~SUPERVISOR_CB_SPINUP_TIMEOUT;}supervisorLog.info=supervisorGetInfoBitfield();supervisorLog.accNorm=sqrtf(supervisorSensors.acc.x*supervisorSensors.acc.x+supervisorSensors.acc.y*supervisorSensors.acc.y+supervisorSensors.acc.z*supervisorSensors.acc.z);}
bool estimatorEnqueue(const EstimatorMeasurement *m){if(!m||estimatorCount>=16)return false;estimatorQueue[estimatorTail]=*m;estimatorTail=(uint8_t)((estimatorTail+1u)%16u);estimatorCount++;return true;}
bool estimatorDequeue(EstimatorMeasurement *m){if(!m||estimatorCount==0)return false;*m=estimatorQueue[estimatorHead];estimatorHead=(uint8_t)((estimatorHead+1u)%16u);estimatorCount--;return true;}
void estimatorComplementary(uint32_t step){EstimatorMeasurement m;while(estimatorDequeue(&m)){if(m.type==MeasurementTypeGyroscope)estSensors.gyro=(Axis3f){m.data[0],m.data[1],m.data[2]};else if(m.type==MeasurementTypeAcceleration)estSensors.acc=(Axis3f){m.data[0],m.data[1],m.data[2]};else if(m.type==MeasurementTypeBarometer){estSensors.baroPressure=m.data[0];estSensors.baroTemperature=m.data[1];estSensors.baroAsl=m.data[2];}else estSensors.tofRange=m.data[0];}if(RATE_DO_EXECUTE(RATE_250_HZ,step)){sensfusion6UpdateQ(estSensors.gyro.x,estSensors.gyro.y,estSensors.gyro.z,estSensors.acc.x,estSensors.acc.y,estSensors.acc.z,1.0f/250.0f);sensfusion6GetEulerRPY(&estimatorState.attitude.roll,&estimatorState.attitude.pitch,&estimatorState.attitude.yaw);estimatorState.attitudeQuaternion=(Quaternion){qx,qy,qz,qw};estimatorState.acc=estSensors.acc;estimatorState.acc.z=sensfusion6GetAccZWithoutGravity(estSensors.acc.x,estSensors.acc.y,estSensors.acc.z);estimatorState.velocity.z+=estimatorState.acc.z/250.0f;}if(RATE_DO_EXECUTE(RATE_100_HZ,step)){estimatorState.position.x+=estimatorState.velocity.x/100.0f;estimatorState.position.y+=estimatorState.velocity.y/100.0f;estimatorState.position.z+=estimatorState.velocity.z/100.0f;}stateEstimate.roll=estimatorState.attitude.roll;stateEstimate.pitch=estimatorState.attitude.pitch;stateEstimate.yaw=estimatorState.attitude.yaw;stateEstimate.qx=qx;stateEstimate.qy=qy;stateEstimate.qz=qz;stateEstimate.qw=qw;gyro=(Axis3Log){estSensors.gyro.x,estSensors.gyro.y,estSensors.gyro.z};acc=(Axis3Log){estSensors.acc.x,estSensors.acc.y,estSensors.acc.z};baro=(BaroLog){estSensors.baroAsl,estSensors.baroTemperature,estSensors.baroPressure};}
bool commanderSetSetpoint(const Setpoint *s,int p){if(!s)return false;if(p==COMMANDER_PRIORITY_DISABLE||p>=commanderPriority){activeSetpoint=*s;commanderPriority=p;commanderLastUpdate=hostTick;return true;}return false;}
void commanderRelaxPriority(void){commanderPriority=COMMANDER_PRIORITY_LOWEST;}
uint32_t commanderGetInactivityTime(void){return hostTick-commanderLastUpdate;}
int commanderGetActivePriority(void){return commanderPriority;}
void stabilizerInit(void){static bool done;if(done)return;done=true;sensfusion6Init();attitudeControllerInit(1.0f/500.0f);supervisorInit();crtpInit();estimatorHead=estimatorTail=estimatorCount=0;memset(&estimatorState,0,sizeof(estimatorState));}
bool stabilizerSubmitHighLevelSetpoint(const Setpoint *s){if(!s)return false;highSetpoint=*s;highPending=true;return true;}
void stabilizerTask(void){stabilizerInit();if(highPending){commanderSetSetpoint(&highSetpoint,COMMANDER_PRIORITY_HIGHLEVEL);highPending=false;}if(healthShallWeRunTest()){healthRunTests(&estSensors);return;}estimatorComplementary(hostTick);supervisorUpdate(hostTick);Setpoint s=activeSetpoint;supervisorOverrideSetpoint(&s,supervisorConditionBits,supervisorState);ControlData c;controllerPid(&estSensors,&s,&estimatorState,&c,0.0f,1.0f/500.0f);MotorPower mp;powerDistribution(&c,&mp);int32_t v[4]={mp.m1,mp.m2,mp.m3,mp.m4};powerDistributionCap(v,65535,0);if(!supervisorCanFly()||!supervisorAreMotorsAllowedToRun()||supervisorConditionBits!=0){v[0]=v[1]=v[2]=v[3]=0;}motor=(MotorLog){pwm(v[0]),pwm(v[1]),pwm(v[2]),pwm(v[3])};hostTick++;}
static uint32_t quatcompress(const Quaternion *q){if(!q)return 0;float a[4]={q->x,q->y,q->z,q->w};int largest=0;for(int i=1;i<4;i++)if(fabsf(a[i])>fabsf(a[largest]))largest=i;float sign=a[largest]<0.0f?-1.0f:1.0f;uint32_t out=(uint32_t)largest;int shift=2;for(int i=0;i<4;i++)if(i!=largest){float v=clampf(a[i]*sign,-0.70710678f,0.70710678f);uint32_t n=(uint32_t)lroundf((v+0.70710678f)*1023.0f/(2.0f*0.70710678f));out|=(n&1023u)<<shift;shift+=10;}return out;}
void compressState(const State *s,const SensorData *n,CompressedState *o){if(!s||!n||!o)return;for(int i=0;i<3;i++){float p=((const float *)&s->position)[i];float v=((const float *)&s->velocity)[i];o->position_mm[i]=(int32_t)lroundf(p*1000.0f);o->velocity_mms[i]=(int32_t)lroundf(v*1000.0f);}o->acceleration_mms2[0]=(int32_t)lroundf(n->acc.x*9810.0f);o->acceleration_mms2[1]=(int32_t)lroundf(n->acc.y*9810.0f);o->acceleration_mms2[2]=(int32_t)lroundf((n->acc.z+1.0f)*9810.0f);o->gyro_millirad_s[0]=n->gyro.x*D2R*1000.0f;o->gyro_millirad_s[1]=-n->gyro.y*D2R*1000.0f;o->gyro_millirad_s[2]=n->gyro.z*D2R*1000.0f;o->quatCompressed=quatcompress(&s->attitudeQuaternion);}
bool rateSupervisorValidate(uint32_t r){return r>=997u&&r<=1003u;}
void rateSupervisorTask(void){rateTick+=2000u;}
void healthRequestPropTest(void){propRequest=true;}
void healthRequestBatteryTest(void){batRequest=true;}
bool healthShallWeRunTest(void){if(propRequest){propRequest=false;healthTestState=configureAcc;healthSampleCount=0;healthMotor=0;motorPass=0;return true;}if(batRequest){batRequest=false;healthTestState=testBattery;healthTick=0;minLoadedVoltage=idleVoltage;return true;}return healthTestState!=testDone;}
float variance(const float *b,int n){if(!b||n<=0)return 0.0f;float sum=0,sq=0;for(int i=0;i<n;i++){sum+=b[i];sq+=b[i]*b[i];}return sq-sum*sum/(float)n;}
bool evaluatePropTest(float lo,float hi,float v,uint8_t m){bool ok=hi==0.0f||(v>=lo&&v<=hi);if(m<4){if(ok)motorPass|=(uint8_t)(1u<<m);else motorPass&=(uint8_t)~(1u<<m);}return ok;}
void healthRunTests(const SensorData *s){if(healthTestState==configureAcc){motor=(MotorLog){0,0,0,0};healthSampleCount=0;healthTestState=measureNoiseFloor;}else if(healthTestState==measureNoiseFloor){if(s&&healthSampleCount<100)healthSamples[healthSampleCount++]=s->acc.z;if(healthSampleCount>=100)healthTestState=measureProp;}else if(healthTestState==measureProp){motorTestCount++;motor=(MotorLog){0,0,0,0};evaluatePropTest(0.0f,0.0f,variance(healthSamples,100),healthMotor);healthMotor++;if(healthMotor>=4)healthTestState=evaluatePropResult;}else if(healthTestState==evaluatePropResult)healthTestState=testDone;else if(healthTestState==testBattery){healthTick++;if(healthTick==1)motor=(MotorLog){30000,30000,30000,30000};else if(healthTick<50){if(s&&s->baroPressure<minLoadedVoltage)minLoadedVoltage=s->baroPressure;}else{motor=(MotorLog){0,0,0,0};batterySag=idleVoltage-minLoadedVoltage;batteryPass=(batterySag<=0.5f);healthTestState=evaluateBatResult;}}else if(healthTestState==evaluateBatResult)healthTestState=testDone;healthLog.motorPass=motorPass;healthLog.batteryPass=batteryPass;healthLog.batterySag=batterySag;healthLog.motorTestCount=motorTestCount;}
typedef struct {CrtpPacket q[16];uint16_t head,tail,count;bool created;} RxQueue;
static RxQueue rxq[CRTP_NBR_OF_PORTS];
static CrtpPacket txq[CRTP_TX_QUEUE_SIZE],pendingPacket;
static uint16_t txHead=0,txTail=0,txCount=0;
static CrtpPortCallback callbacks[CRTP_NBR_OF_PORTS];
static CrtpLink *currentLink;
static bool crtpReady=false,pendingValid=false;
static uint32_t retryAt=0,rxCount=0,txCountStats=0,lastStats=0;
static bool nopSend(CrtpPacket *p){(void)p;return false;}
static bool nopReceive(CrtpPacket *p){(void)p;return false;}
static bool nopConnected(void){return true;}
static void nopEnable(bool e){(void)e;}
static void nopReset(void){}
static CrtpLink nopLink={nopSend,nopReceive,nopConnected,nopEnable,nopReset};
void crtpInit(void){if(crtpReady)return;memset(rxq,0,sizeof(rxq));memset(callbacks,0,sizeof(callbacks));txHead=txTail=txCount=0;currentLink=&nopLink;pendingValid=false;crtpReady=true;}
void crtpInitTaskQueue(uint8_t p){crtpInit();if(p>=CRTP_NBR_OF_PORTS)return;if(rxq[p].created)supervisorConditionBits|=SUPERVISOR_CB_DECK_FAULT;else rxq[p].created=true;}
bool crtpSendPacket(const CrtpPacket *p){crtpInit();if(!p||txCount>=CRTP_TX_QUEUE_SIZE)return false;txq[txTail]=*p;txTail=(uint16_t)((txTail+1u)%CRTP_TX_QUEUE_SIZE);txCount++;txCountStats++;return true;}
bool crtpSendPacketBlock(const CrtpPacket *p){return crtpSendPacket(p);}
bool crtpReceivePacket(uint8_t p,CrtpPacket *o){if(p>=CRTP_NBR_OF_PORTS||!o||!rxq[p].created||rxq[p].count==0)return false;*o=rxq[p].q[rxq[p].head];rxq[p].head=(uint16_t)((rxq[p].head+1u)%CRTP_RX_QUEUE_SIZE);rxq[p].count--;return true;}
bool crtpReceivePacketBlock(uint8_t p,CrtpPacket *o){return crtpReceivePacket(p,o);}
bool crtpReceivePacketWait(uint8_t p,CrtpPacket *o,uint32_t wait){uint32_t start=hostTick;do{if(crtpReceivePacket(p,o))return true;crtpRxTask();if(hostTick-start>=wait)break;hostTick++;}while(true);return false;}
void crtpRxTask(void){crtpInit();if(currentLink==&nopLink||!currentLink||!currentLink->receivePacket)return;CrtpPacket p;if(!currentLink->receivePacket(&p))return;rxCount++;if(p.port<CRTP_NBR_OF_PORTS){if(rxq[p.port].created&&rxq[p.port].count<CRTP_RX_QUEUE_SIZE){rxq[p.port].q[rxq[p.port].tail]=p;rxq[p.port].tail=(uint16_t)((rxq[p.port].tail+1u)%CRTP_RX_QUEUE_SIZE);rxq[p.port].count++;}if(callbacks[p.port])callbacks[p.port](&p);}}
void crtpTxTask(void){crtpInit();if(hostTick<retryAt)return;if(!pendingValid){if(txCount==0)return;pendingPacket=txq[txHead];txHead=(uint16_t)((txHead+1u)%CRTP_TX_QUEUE_SIZE);txCount--;pendingValid=true;}if(currentLink!=&nopLink&&currentLink&&currentLink->sendPacket&&currentLink->sendPacket(&pendingPacket))pendingValid=false;else retryAt=hostTick+10u;}
void crtpSetLink(CrtpLink *l){crtpInit();if(currentLink&&currentLink->setEnable)currentLink->setEnable(false);currentLink=l?l:&nopLink;if(currentLink->setEnable)currentLink->setEnable(true);}
void crtpReset(void){crtpInit();txHead=txTail=txCount=0;pendingValid=false;if(currentLink&&currentLink->reset)currentLink->reset();}
bool crtpIsConnected(void){crtpInit();return currentLink&&currentLink->isConnected?currentLink->isConnected():true;}
uint32_t crtpGetFreeTxQueuePackets(void){crtpInit();return CRTP_TX_QUEUE_SIZE-txCount;}
void crtpRegisterPortCB(uint8_t p,CrtpPortCallback c){if(p<CRTP_NBR_OF_PORTS)callbacks[p]=c;}
void updateStats(void){if((uint32_t)(hostTick-lastStats)>=500u){rxCount=0;txCountStats=0;lastStats=hostTick;}}
uint8_t deckDiscovery(DeckInfo *d,uint8_t c){if(!d||c==0)return 0;return 0;}