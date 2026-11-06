// HAB-RW: single-axis yaw reaction wheel. Nano + BNO085 + Pololu 25D 20.4:1 (979.6 cpr x4) + G2 18v17

#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define FW_VERSION "HAB-RW 1.2.0-ground"

#ifndef BUILD_COMMISSIONING
#define BUILD_COMMISSIONING 0  // 1 = ground commissioning only, control loop not compiled in. 0 = closed-loop build
#endif

#define SIGN_COMMISSIONED           1  // drive commands refused until the 'C' sign check passes
#define HEADING_RATE_SIGN_VERIFIED  1
const int8_t MOTOR_DIR_SIGN  = 1;
const int8_t WHEEL_AXIS_SIGN = 1;
const int8_t BIAS_SIGN       = 1;
#if SIGN_COMMISSIONED && !HEADING_RATE_SIGN_VERIFIED
#error "SIGN_COMMISSIONED=1 requires HEADING_RATE_SIGN_VERIFIED=1 (commissioning step 1 must have passed)"
#endif

#define USE_WATCHDOG       0
#define USE_BNO_INT        1
#define USE_DRIVER_FLT     0
#define USE_VBAT           0
#define USE_CS             0
#define USE_VERTICAL_RATE  1
#define USE_BNO_HASRESET   1
#define PARSE_ALL_REPORTS  1

const uint32_t SERIAL_BAUD  = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint8_t  BNO_ADDR     = 0x4A;
const float    VBAT_SCALE   = 0.0f;

const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 3;
const uint8_t PIN_DIR   = 4;
const uint8_t PIN_PWM   = 5;
const uint8_t PIN_SLP   = 6;
const uint8_t PIN_FLT   = 7;
const uint8_t PIN_BNO_INT = 8;

const float RPM_TO_RADS = 0.10471976f;
const float J_LOCKED = 0.01925852f;  // kg m^2, bifilar pendulum, wheel locked
const float J_WHEEL  = 3.189772e-4f;  // kg m^2, CAD
const float ENC_CPR  = 979.616f;
const float A_MAP    = 1.8477f;  // rpm per PWM count, linear fit over PWM 25-65 @ 11.37 V
const float B_MAP    = -7.140f;
const float TAU_UP   = 0.0365f;  // s, first-order fits, spin-up / spin-down
const float TAU_DN   = 0.025f;

// bias momentum: idle at 75 rpm, torque by speeding up / slowing down, never reverse
// +/-30 rpm band = only +/-3 deg/s of body-rate authority (J_w/J_b = 0.0166)
const float W_BIAS_RPM = 75.0f;
const float W_SOFT_LO_RPM = 45.0f;
const float W_SOFT_HI_RPM = 105.0f;
const float W_HARD_HI_RPM = 115.0f;
const float W_UNDER_RPM = 35.0f;
const uint32_t UNDER_TIME_MS = 500;
const float W_LIMIT_MARGIN_RPM = 8.0f;
const float W_LIMIT_HYST_RPM = 5.0f;
const int16_t PWM_RUN_MIN = 25;
const int16_t PWM_HOLD_MAX = 62;
const float PWM_SLEW_PER_TICK = 2.0f;
const float KI_TRIM = 0.5f;
const float TRIM_MAX = 8.0f;
const float REF_LEAD_MAX_RPM = 5.0f;
const float ALPHA_MAX_RPMPS = 60.0f;
const int16_t PWM_KICK_START = 30;
const int16_t PWM_KICK_END = 48;
const uint32_t KICK_RAMP_MS = 300;
const uint32_t KICK_MAX_MS = 600;
const int16_t KICK_NOMOTION_COUNTS = 5;
const int16_t KICK_DIRCHECK_COUNTS = 20;
const float SPINUP_HANDOVER_RPM = 55.0f;
const float SPINUP_RAMP_RPMPS = 30.0f;
const uint8_t SPINUP_RETRIES = 2;
const float SPINDOWN_RAMP_RPMPS = 30.0f;
const float K_THETA = 0.8f;  // 1/s, heading loop
const float T_RATE = 0.25f;  // s, inner rate loop: K_rate = J_b / T_RATE
const float R_MAX_DPS = 2.0f;
const float RDOT_MAX_DPS2 = 4.0f;
const float LIMITED_RMAX_SCALE = 0.5f;
const float H_FILTER_TAU_S = 0.3f;
const uint32_t DT_INNER_US = 10000;
const uint8_t  OUTER_DIV = 4;
const uint32_t TELEM_PERIOD_US = 100000;
const uint32_t IMU_POLL_MIN_US = 2000;
const uint16_t GYRO_REPORT_MS = 10;
const uint16_t GRV_REPORT_MS = 20;
const uint32_t GYRO_STALE_US = 40000;
const uint32_t ATT_STALE_US = 100000;
const uint32_t LOOP_LATE_US = 40000;
const uint32_t LOOP_MISS_US = 15000;
const uint16_t ENC_INVALID_BURST = 5;
const uint16_t ENC_INVALID_TOTAL = 20;
const float ENC_PLAUS_RPM = 25.0f;
const uint32_t ENC_PLAUS_MS = 300;
const int16_t STOPPED_COUNTS = 2;
const float TILT_SPEC_DEG = 5.0f;
const float TILT_FAULT_DEG = 15.0f;
const float HEADING_JUMP_DEG = 5.0f;
const float QUAT_NORM_TOL = 0.1f;
const uint16_t STACK_MIN_FREE = 128;
const float GYRO_MAX_DPS = 500.0f;
const uint32_t I2C_SLOW_US = 20000;
const uint32_t WIRE_TIMEOUT_US = 10000;
const uint32_t SELF_CHECK_TIMEOUT_MS = 10000;
const uint8_t AUTO_RECOVERY_MAX = 3;
const float CAPTURE_RATE_DPS = 0.3f;
const uint32_t CAPTURE_STILL_MS = 3000;
const uint32_t CAPTURE_TIMEOUT_MS = 60000;
const uint32_t PROFILE_MIN_DELAY_MS = 20000;
const float TARGET_STEP_DEG = 5.0f;
const float TARGET_OFFSET_MAX_DEG = 30.0f;
const float RATE_STEP_DPS = 0.5f;
const uint32_t RATE_STEP_MS = 2000;
const uint32_t CONFIRM_MS = 5000;
const float CAPTURE_LPF_S = 0.5f;
const float COMM_ROT_DEG = 20.0f;
const uint32_t COMM_ROT_TIMEOUT_MS = 45000;
const float COMM_RATIO_LO = 0.8f;
const float COMM_RATIO_HI = 1.2f;
const uint32_t COMM_STILL_MS = 2000;
const uint32_t COMM_BASE_MS = 500;
const uint32_t COMM_KICK_MAX_MS = 500;
const int16_t COMM_MOTION_COUNTS = 10;
const int16_t PWM_COMM_PLATEAU = 30;
const uint32_t COMM_PLATEAU_MS = 250;
const uint32_t COMM_AVG_MS = 100;
const uint32_t COMM_COAST_MS = 500;
const float COMM_MAX_RPM = 80.0f;
const float COMM_MIN_DR_DPS = 1.0f;
const float COMM_GAIN_TOL = 0.5f;
const uint32_t COMM_READY_TIMEOUT_MS = 10000;
const float SIGNMON_MIN_DPS2 = 1.0f;
const float SIGNMON_MEAS_MIN_DPS2 = 0.5f;
const uint8_t SIGNMON_WINDOW = 12;
const uint8_t SIGNMON_TRIP = 10;
const uint8_t SIGNMON_GAP_RESET = 3;
const float NORESP_MIN_DPS2 = 2.0f;
const float NORESP_FRAC = 0.2f;
const uint8_t NORESP_WINDOW = 24;
const uint8_t NORESP_TRIP = 20;
const uint32_t SAT_REST_MS = 5000;
const uint32_t PIN_FAULT_MS = 2000;
const uint32_t SUMMARY_SETTLE_MS = 10000;
const float RATIO_MIN_DEG = 2.0f;
const float RATIO_LO = 0.7f;
const float RATIO_HI = 1.3f;
const uint32_t RATIO_WINDOW_MS = 3000;
const uint8_t RATIO_TRIP_COUNT = 2;
const float DIVERGE_DEG = 10.0f;
const uint32_t DIVERGE_MS = 2000;
const uint32_t DIVERGE_SAT_MS = 10000;
const float DIVERGE_GROW_DEG = 1.0f;
const float RATE_FLAG_DPS = 6.0f;

const int8_t PROFILE[][2] PROGMEM = { {30, 0}, {30, 5}, {30, 0}, {30, -5}, {30, 0} };
const uint8_t PROFILE_LEN = sizeof(PROFILE) / sizeof(PROFILE[0]);

enum : uint8_t { ST_BOOT_SAFE = 0, ST_SENSOR_INIT, ST_SELF_CHECK, ST_DISARMED, ST_COMMISSION, ST_SPINUP,
                 ST_BIAS_IDLE, ST_RATE_TEST, ST_CAPTURE, ST_ARMED_HOLD, ST_LIMITED, ST_SPINDOWN,
                 ST_LATCHED_SAFE, ST_RECOVERY };
const uint32_t F_GYRO_STALE = 1UL << 0, F_ATT_STALE = 1UL << 1, F_IMU_RESET = 1UL << 2, F_ATT_INVALID = 1UL << 3,
  F_ATT_JUMP = 1UL << 4, F_TILT = 1UL << 5, F_IMU_INCONSIST = 1UL << 6, F_ENC_INVALID = 1UL << 7,
  F_ENC_PLAUS = 1UL << 8, F_OVERSPEED = 1UL << 9, F_UNDERSPEED = 1UL << 10, F_STALL = 1UL << 11,
  F_DIR_MISMATCH = 1UL << 12, F_LOOP_LATE = 1UL << 13, F_DRIVER_FLT = 1UL << 14, F_SIGN_FAULT = 1UL << 15,
  F_DIVERGE = 1UL << 16, F_STACK = 1UL << 17, F_UNEXPECTED_RESET = 1UL << 18, F_IMU_INIT = 1UL << 19,
  F_INTERNAL = 1UL << 20, F_NO_RESPONSE = 1UL << 21;
const uint32_t F_STICKY = F_STACK | F_DIR_MISMATCH | F_INTERNAL;
const uint32_t F_IMU_ANY = F_GYRO_STALE | F_ATT_STALE | F_IMU_RESET | F_ATT_INVALID | F_ATT_JUMP | F_IMU_INIT;
const uint16_t FL_LIM_POS = 1 << 0, FL_LIM_NEG = 1 << 1, FL_SAT = 1 << 2, FL_TILT_WARN = 1 << 3,
  FL_WREF_CLAMP = 1 << 4, FL_PWM_CLAMP = 1 << 5, FL_RATE_FLAG = 1 << 6, FL_TARGET_VALID = 1 << 7,
  FL_SIGNMON_ACTIVE = 1 << 8, FL_PROFILE = 1 << 9, FL_SIGN_UNCOMMISSIONED = 1 << 10;

static inline bool isDriveState(uint8_t s) { return s >= ST_COMMISSION && s <= ST_SPINDOWN; }
static inline bool isControlState(uint8_t s) { return s >= ST_RATE_TEST && s <= ST_LIMITED; }
static inline bool isRunningState(uint8_t s) { return s >= ST_BIAS_IDLE && s <= ST_LIMITED; }

uint8_t resetFlagsR2 __attribute__((section(".noinit")));
uint8_t resetFlagsMcusr __attribute__((section(".noinit")));
void earlyInit0(void) __attribute__((naked, used, section(".init0")));
void earlyInit0(void) { __asm__ __volatile__("sts %0, r2\n" : "=m"(resetFlagsR2) :); }
// motor pins forced low in .init3, before the C runtime. nothing drives at boot
void earlyInit3(void) __attribute__((naked, used, section(".init3")));
void earlyInit3(void) {
  resetFlagsMcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
  PORTD &= (uint8_t)~((1 << PD4) | (1 << PD5) | (1 << PD6));
  DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6);
}

BNO080 imu;

volatile int32_t encCount = 0;
volatile uint16_t encInvalid = 0;
volatile uint8_t encPrev = 0;
const int8_t QDEC_TABLE[16] PROGMEM = { 0, 1, -1, 2,  -1, 0, 2, 1,  1, 2, 0, -1,  2, -1, 1, 0 };

float gxs = 0, gys = 0, gzs = 0;
float R31 = 0, R32 = 0, R33 = 1;
float psiMeas = 0, psiPrev = 0, cosTilt = 1, rMeas = 0;
const float COS_TILT_SPEC = 0.99619470f;
const float COS_TILT_FAULT = 0.96592583f;
uint32_t tGyro = 0, tAtt = 0, tImuPoll = 0, tLastGyroInt = 0;
bool gyroSeen = false, attSeen = false;
uint8_t gyroSeq = 0, grvSeq = 0;
uint16_t gyroGaps = 0, grvGaps = 0, imuUnknown = 0;
float rmIntR = 0, rmDpsi = 0;
uint32_t rmT0 = 0;
bool rmActive = false;
uint8_t rmBad = 0;

uint8_t state = ST_BOOT_SAFE;
uint32_t faults = 0;
uint16_t flags = 0;
uint32_t tInner = 0;
uint8_t outerDiv = 0;
uint16_t deadlineMisses = 0;
uint32_t maxIntervalUs = 0, passMaxUs = 0;
uint16_t stackFree = 0;
uint16_t invLast = 0;
uint8_t invDiv = 0;
uint8_t fltCount = 0;
uint16_t invBase = 0;
uint16_t i2cErr = 0, imuBad = 0;
uint8_t firstFault = 0xFF, autoRecoveries = 0;
uint32_t tLastTick = 0;
char deferredPrint = 0;

int32_t ringC[6]; uint32_t ringT[6];
uint8_t ringI = 0, ringN = 0;
float wMeas = 0;
uint8_t blockI = 0, overBlocks = 0;
int32_t slowC[6];
uint8_t slowI = 0, slowN = 0, slowDiv = 0;
bool wheelStopped = false;
float wAlphaEst = 0;
uint32_t tTelemPrev = 1;
float underMs = 0, plausMs = 0;
float wPred = 0, wRm = 0;
float predInc[6], rmInc[6];
float wPredWin = 0, wRmWin = 0;
float wRef = 0, trim = 0, uCont = 0, sdAcc = 0;
bool clampedHi = false, clampedLo = false, slewedUp = false, slewedDn = false;
int16_t uApplied = 0;
bool slpOn = false;
uint8_t lastDirDriven = 0xFF;
uint16_t polarityChanges = 0;
float psiTarget = 0, targetOffset = 0, rCmd = 0, rdDes = 0, alphaRef = 0, eHead = 0;
bool targetValid = false;
float Hf = 0; bool Hinit = false;
float rLoH = -1, rHiH = 1;
float rateTestCmd = 0; uint32_t rateStepT0 = 0; bool rateStepActive = false;
uint8_t suPhase = 0, suRetries = 0;
uint32_t suT0 = 0, suWaitT0 = 0; bool suWaiting = false;
int32_t suC0 = 0;
uint32_t capT0 = 0, capMinDelayMs = 0;
float capStillMs = 0, rLp = 0, capBlkRef = 0, capBlkSum = 0, capLastMean = 0;
uint8_t capBlkN = 0; bool capBlkStill = true, capHaveMean = false;
float smRdAcc = 0, smRAcc = 0, smPrevRdBlk = 0, smPrevRdBlk2 = 0, smRblk[3];
uint8_t smGap = 0, nrQ = 0;
uint32_t nrMask = 0;
uint8_t smN = 0, smRblkN = 0, smQ = 0;
uint16_t smMask = 0;
uint32_t dvTRef = 0; float dvERef = 0, dvPeak = 0; bool dvRebase = true;
uint32_t pinT0 = 0, satrT0 = 0; bool pinActive = false, satrActive = false;
uint32_t rsSettleT0 = 0;
uint8_t cmPhase = 0, cmBad = 0; bool cmAwaitY = false, cmBaseNoisy = false;
uint32_t cmReadyT0 = 0;
uint32_t cmT0 = 0; int32_t cmC0 = 0, cmDc = 0;
float cmPsiRel = 0, cmIntR = 0, cmMaxPos = 0, cmMaxNeg = 0, cmPsiLast = 0, cmStillMs = 0;
float cmR0 = 0, cmRp = 0, cmWp = 0; uint16_t cmN = 0, cmNp = 0;
char pendingCmd = 0; uint32_t pendingT = 0;
uint32_t tTelem = 0, tDiag = 0; uint8_t telField = 0;
uint8_t evCode[4]; int16_t evArg[4]; uint8_t evHead = 0, evTail = 0;
uint8_t profStep = 0; uint32_t profT0 = 0; bool profileActive = false;
struct RunSummary { uint8_t magic; uint16_t dur_s; uint16_t peakErr_cdeg; uint16_t tIn2_t40; uint16_t tIn3_t40;
                    int16_t wMin_drpm; int16_t wMax_drpm; uint16_t tLim_t40; uint16_t tSat_t40; uint16_t tEval_t40;
                    uint32_t faults; uint8_t firstFault; uint8_t endReason; };
const uint8_t SUMMARY_MAGIC = 0x5C;
RunSummary rs;
uint32_t rsT0 = 0;
bool rsActive = false;
uint8_t rsWriteIdx = 0xFF;

const int EE_SESSION = 0;
const int EE_VERSION = 1;
const int EE_SUMMARY = 8;
const uint8_t SESSION_ACTIVE = 0xA5;

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float sgnf(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }
__attribute__((noinline)) static float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a <= -PI) a += TWO_PI;
  return a;
}
static int32_t encRead() { uint8_t s = SREG; cli(); int32_t c = encCount; SREG = s; return c; }
static uint16_t encInvRead() { uint8_t s = SREG; cli(); uint16_t c = encInvalid; SREG = s; return c; }

enum : uint8_t { EV_FAULT = 1, EV_STATE, EV_CAPTURED, EV_LIMITED, EV_REFUSED, EV_CONFIRM, EV_COMM, EV_PROFILE,
                 EV_CAPTURE_TIMEOUT, EV_STICKY };
__attribute__((noinline)) static void pushEvent(uint8_t code, int16_t arg) {
  uint8_t n = (evHead + 1) & 3;
  if (n == evTail) {
    if (code != EV_FAULT) return;
    evTail = (evTail + 1) & 3;
  }
  evCode[evHead] = code; evArg[evHead] = arg; evHead = n;
}

void encoderISR() {  // x4 quadrature, integer math only
  uint8_t s = (PIND >> 2) & 0x03;
  int8_t d = (int8_t)pgm_read_byte(&QDEC_TABLE[(encPrev << 2) | s]);
  if (d == 2) encInvalid++;
  else encCount += d;
  encPrev = s;
}

#define SLP_LOW()  (PORTD &= (uint8_t)~(1 << PD6))
#define SLP_HIGH() (PORTD |= (1 << PD6))
#define DIR_SET(h) do { if (h) PORTD |= (1 << PD4); else PORTD &= (uint8_t)~(1 << PD4); } while (0)
// cutting drive at 75 rpm hands the wheel's momentum back to the payload, ~7.5 deg/s
__attribute__((noinline)) static void motorCoast() {
  SLP_LOW();
  analogWrite(PIN_PWM, 0);
  slpOn = false;
  uApplied = 0;
}
__attribute__((noinline, unused)) static void motorApply(int16_t u) {
  if (u == 0) { motorCoast(); return; }
  uint8_t d = ((int16_t)u * MOTOR_DIR_SIGN > 0) ? 1 : 0;
  if (lastDirDriven != 0xFF && d != lastDirDriven) polarityChanges++;
  lastDirDriven = d;
  DIR_SET(d);
  int16_t m = u < 0 ? -u : u;
  if (m > 255) m = 255;
  analogWrite(PIN_PWM, (uint8_t)m);
  SLP_HIGH();
  uApplied = u;
  slpOn = true;
}
#if BUILD_COMMISSIONING
static void motorApplyRaw(uint8_t dirLevel, int16_t pwm) {
  DIR_SET(dirLevel);
  analogWrite(PIN_PWM, (uint8_t)pwm);
  SLP_HIGH();
  uApplied = pwm;
  slpOn = true;
}
#endif

__attribute__((noinline)) static void sessionMark(bool active) { EEPROM.update(EE_SESSION, active ? SESSION_ACTIVE : 0x00); }

static void summaryFinish(uint8_t reason);

// any fault -> coast + latch. R clears it, sticky faults need a power cycle
__attribute__((noinline)) static void latchFault(uint32_t bit) {
  uint32_t nw = bit & ~faults;
  faults |= bit;
  if (nw) {
    uint8_t i = 0;
    while (i < 32 && !(nw & (1UL << i))) i++;
    if (firstFault == 0xFF) firstFault = i;
    pushEvent(EV_FAULT, i);
  }
  if (state != ST_LATCHED_SAFE) {
    motorCoast();
    if (rsActive) summaryFinish(2);
    state = ST_LATCHED_SAFE;
    targetValid = false;
    profileActive = false;
    pendingCmd = 0;
  }
}

static void imuHandleGyro(uint32_t now, int16_t rx, int16_t ry, int16_t rz, uint8_t seq) {
  const int16_t GLIM = (int16_t)(GYRO_MAX_DPS * DEG_TO_RAD * 512.0f);
  if (gyroSeen && seq == gyroSeq) { imuBad++; return; }
  if (rx > GLIM || rx < -GLIM || ry > GLIM || ry < -GLIM || rz > GLIM || rz < -GLIM) { imuBad++; return; }
  if (gyroSeen && (uint8_t)(seq - gyroSeq) != 1) gyroGaps += (uint8_t)(seq - gyroSeq - 1);
  gyroSeq = seq;
  gxs = rx * (1.0f / 512.0f);
  gys = ry * (1.0f / 512.0f);
  gzs = rz * (1.0f / 512.0f);
#if USE_VERTICAL_RATE
  rMeas = R31 * gxs + R32 * gys + R33 * gzs;
#else
  rMeas = (R33 >= 0 ? gzs : -gzs);
#endif
  if (gyroSeen) {
    float dt = (float)(now - tLastGyroInt) * 1e-6f;
    if (dt > 0.05f) dt = 0.05f;
    rmIntR += rMeas * dt;
    if (state == ST_COMMISSION && cmPhase == 1) cmIntR += rMeas * dt;
  }
  tLastGyroInt = now;
  tGyro = now;
  gyroSeen = true;
}

static void imuHandleGrv(uint32_t now, int16_t ri, int16_t rj, int16_t rk, int16_t rr, uint8_t seq) {
  if (attSeen && seq == grvSeq) { imuBad++; return; }
  if (attSeen && (uint8_t)(seq - grvSeq) != 1) grvGaps += (uint8_t)(seq - grvSeq - 1);
  grvSeq = seq;
  float qx = ri * (1.0f / 16384.0f), qy = rj * (1.0f / 16384.0f);
  float qz = rk * (1.0f / 16384.0f), qw = rr * (1.0f / 16384.0f);
  float n2 = qw * qw + qx * qx + qy * qy + qz * qz;
  if (!(fabs(n2 - 1.0f) <= QUAT_NORM_TOL)) {
    if (isDriveState(state)) latchFault(F_ATT_INVALID);
    return;
  }
  float inv = 1.0f / sqrt(n2);
  qw *= inv; qx *= inv; qy *= inv; qz *= inv;
  float R11 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float R21 = 2.0f * (qx * qy + qw * qz);
  R31 = 2.0f * (qx * qz - qw * qy);
  R32 = 2.0f * (qy * qz + qw * qx);
  R33 = 1.0f - 2.0f * (qx * qx + qy * qy);
  float psi = atan2(R21, R11);
  cosTilt = fabs(R33);
  if (attSeen) {
    float dpsi = wrapPi(psi - psiPrev);
    float dt = (float)(now - tAtt) * 1e-6f;
    float allow = (HEADING_JUMP_DEG + 60.0f * dt) * DEG_TO_RAD;
    if (fabs(dpsi) > allow && isDriveState(state)) latchFault(F_ATT_JUMP);
    rmDpsi += dpsi;
  }
  psiPrev = psi;
  psiMeas = psi;
  tAtt = now;
  attSeen = true;
}

#if PARSE_ALL_REPORTS
static void imuParsePacket(uint32_t now) {
  if (imu.shtpHeader[1] & 0x80) { i2cErr++; return; }
  uint16_t len = (((uint16_t)imu.shtpHeader[1] << 8) | imu.shtpHeader[0]) & 0x7FFF;
  uint8_t ch = imu.shtpHeader[2];
  if (len <= 4) return;
  len -= 4;
  if (len > MAX_PACKET_SIZE) { i2cErr++; return; }
  if (ch != CHANNEL_REPORTS && ch != CHANNEL_WAKE_REPORTS) return;
  const uint8_t *d = imu.shtpData;
  uint16_t i = 0;
  while (i < len) {
    uint8_t id = d[i], rl;
    switch (id) {
      case 0xFB: case 0xFA: rl = 5; break;
      case SENSOR_REPORTID_GYROSCOPE: rl = 10; break;
      case SENSOR_REPORTID_GAME_ROTATION_VECTOR: rl = 12; break;
      case 0x01: case 0x03: case 0x04: case 0x06: rl = 10; break;
      case 0x05: case 0x09: rl = 14; break;
      case 0x07: rl = 16; break;
      default: imuUnknown++; return;
    }
    if ((uint16_t)(i + rl) > len) { imuUnknown++; return; }
    if (id == SENSOR_REPORTID_GYROSCOPE) {
      imuHandleGyro(now, (int16_t)((uint16_t)d[i + 5] << 8 | d[i + 4]), (int16_t)((uint16_t)d[i + 7] << 8 | d[i + 6]),
                    (int16_t)((uint16_t)d[i + 9] << 8 | d[i + 8]), d[i + 1]);
    } else if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      imuHandleGrv(now, (int16_t)((uint16_t)d[i + 5] << 8 | d[i + 4]), (int16_t)((uint16_t)d[i + 7] << 8 | d[i + 6]),
                   (int16_t)((uint16_t)d[i + 9] << 8 | d[i + 8]), (int16_t)((uint16_t)d[i + 11] << 8 | d[i + 10]), d[i + 1]);
    }
    i += rl;
  }
}
#endif

static void imuService(uint32_t now) {
  if ((uint32_t)(now - tImuPoll) < IMU_POLL_MIN_US) return;
  tImuPoll = now;
#if USE_BNO_INT
  if (digitalRead(PIN_BNO_INT) == HIGH) return;
#endif
#if PARSE_ALL_REPORTS
  memset(imu.shtpData, 0, MAX_PACKET_SIZE);
  imu.shtpHeader[0] = imu.shtpHeader[1] = 0;
  Wire.clearWireTimeoutFlag();
  uint32_t tr0 = micros();
  bool got = imu.receivePacket();
  if (Wire.getWireTimeoutFlag() || (uint32_t)(micros() - tr0) > I2C_SLOW_US) i2cErr++;
  else if (got) imuParsePacket(micros());
#else
  uint16_t id = imu.getReadings();
  uint32_t t = micros();
  if (id == SENSOR_REPORTID_GYROSCOPE)
    imuHandleGyro(t, imu.getRawGyroX(), imu.getRawGyroY(), imu.getRawGyroZ(), (uint8_t)(gyroSeq + 1));
  else if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
    imuHandleGrv(t, (int16_t)(imu.getQuatI() * 16384.0f), (int16_t)(imu.getQuatJ() * 16384.0f),
                 (int16_t)(imu.getQuatK() * 16384.0f), (int16_t)(imu.getQuatReal() * 16384.0f), (uint8_t)(grvSeq + 1));
#endif
#if USE_BNO_HASRESET
  if (imu.hasReset()) {
    targetValid = false;
    if (isDriveState(state) || autoRecoveries >= AUTO_RECOVERY_MAX) latchFault(F_IMU_RESET);
    else {
      faults |= F_IMU_RESET; pushEvent(EV_FAULT, 2);
      if (state == ST_DISARMED || state == ST_SELF_CHECK) { autoRecoveries++; state = ST_RECOVERY; }
    }
  }
#endif
}

static void i2cBusClear() {
  pinMode(SDA, INPUT_PULLUP); pinMode(SCL, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    pinMode(SCL, OUTPUT); digitalWrite(SCL, LOW); delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP); delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW); delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP); delayMicroseconds(5);
}

static void i2cStart() {
  i2cBusClear();
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
}

__attribute__((noinline)) static bool imuFresh(uint32_t now) {
  return gyroSeen && attSeen && (uint32_t)(now - tGyro) <= GYRO_STALE_US && (uint32_t)(now - tAtt) <= ATT_STALE_US;
}

static bool imuInit() {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
#if USE_WATCHDOG
    wdt_reset();
#endif
    if (imu.begin(BNO_ADDR, Wire)) {
#if USE_WATCHDOG
      wdt_reset();
#endif
      imu.enableGyro(GYRO_REPORT_MS);
      imu.enableGameRotationVector(GRV_REPORT_MS);
#if USE_BNO_HASRESET
      imu.hasReset();
#endif
      gyroSeen = attSeen = false;
      return true;
    }
    delay(50);
  }
  return false;
}

static void wheelEstimatorTick(uint32_t now) {
  int32_t c = encRead();
  ringI = (ringI + 1) % 6;
  ringC[ringI] = c; ringT[ringI] = now;
  if (ringN < 6) ringN++;
  if (ringN >= 6) {
    uint8_t j = (ringI + 1) % 6;
    float dtw = (float)(now - ringT[j]) * 1e-6f;
    if (dtw > 0) wMeas = (float)(c - ringC[j]) / ENC_CPR * TWO_PI / dtw;
  }
  if (++blockI >= 5) {
    blockI = 0;
    if (fabs(wMeas) > W_HARD_HI_RPM * RPM_TO_RADS) overBlocks++; else overBlocks = 0;
    if (overBlocks >= 2) latchFault(F_OVERSPEED);
  }
  if (++invDiv >= 100) {
    invDiv = 0;
    uint16_t inv = encInvRead();
    if ((uint16_t)(inv - invLast) > ENC_INVALID_BURST && isDriveState(state)) latchFault(F_ENC_INVALID);
    invLast = inv;
  }
  if (++slowDiv >= 10) {
    slowDiv = 0;
    slowI = (slowI + 1) % 6;
    slowC[slowI] = c;
    if (slowN < 6) slowN++;
    if (slowN >= 6) {
      int32_t dc = c - slowC[(slowI + 1) % 6];
      wheelStopped = (dc <= STOPPED_COUNTS && dc >= -STOPPED_COUNTS);
    }
  }
}

static void modelsTick(float dt) {
  if (slpOn && abs(uApplied) >= PWM_RUN_MIN) {
    float tgt = sgnf((float)uApplied) * (A_MAP * abs(uApplied) + B_MAP) * RPM_TO_RADS;
    float tau = (fabs(tgt) > fabs(wPred) && tgt * wPred >= 0) ? TAU_UP : TAU_DN;
    wPred += (tgt - wPred) * (dt < tau ? dt / tau : 1.0f);
  } else {
    wPred = wMeas;
  }
  float tau = (fabs(wRef) > fabs(wRm)) ? TAU_UP : TAU_DN;
  wRm += (wRef - wRm) * (dt < tau ? dt / tau : 1.0f);
  predInc[ringI] = wPred * dt;
  rmInc[ringI] = wRm * dt;
  if (ringN >= 6) {
    uint8_t j = (ringI + 1) % 6;
    float dtw = (float)(ringT[ringI] - ringT[j]) * 1e-6f;
    float sp = 0, sr = 0;
    for (uint8_t k = 0; k < 5; k++) { uint8_t m = (ringI + 6 - k) % 6; sp += predInc[m]; sr += rmInc[m]; }
    if (dtw > 0) { wPredWin = sp / dtw; wRmWin = sr / dtw; }
  } else {
    wPredWin = wPred; wRmWin = wRm;
  }
}

#if !BUILD_COMMISSIONING
__attribute__((noinline)) static void actuatorMap(float dt) {
  float wbRpm = BIAS_SIGN * wRef / RPM_TO_RADS;
  float uc = (wbRpm - B_MAP) / A_MAP + trim;
  clampedLo = uc < PWM_RUN_MIN;
  clampedHi = uc > PWM_HOLD_MAX;
  uc = clampf(uc, PWM_RUN_MIN, PWM_HOLD_MAX);
  float prev = uCont > 0 ? uCont : uc;
  slewedUp = uc > prev + PWM_SLEW_PER_TICK;
  slewedDn = uc < prev - PWM_SLEW_PER_TICK;
  uc = clampf(uc, prev - PWM_SLEW_PER_TICK, prev + PWM_SLEW_PER_TICK);
  uCont = uc;
  sdAcc += uc;
  int16_t uq = (int16_t)(sdAcc + 0.5f);
  sdAcc -= uq;
  sdAcc = clampf(sdAcc, -1.0f, 1.0f);
  if (uq < PWM_RUN_MIN) uq = PWM_RUN_MIN;
  if (uq > PWM_HOLD_MAX) uq = PWM_HOLD_MAX;
  if (clampedHi || clampedLo) flags |= FL_PWM_CLAMP; else flags &= ~FL_PWM_CLAMP;
  motorApply(BIAS_SIGN * uq);
  float eRpm = BIAS_SIGN * (wMeas - wRmWin) / RPM_TO_RADS;
  float dtrim = -KI_TRIM * eRpm * dt;
  if ((dtrim > 0 && (clampedHi || slewedUp)) || (dtrim < 0 && (clampedLo || slewedDn))) dtrim = 0;
  trim = clampf(trim + dtrim, -TRIM_MAX, TRIM_MAX);
}

__attribute__((noinline)) static void runningChecks(float dt) {
  float wb = BIAS_SIGN * wMeas / RPM_TO_RADS;
  if (wb < W_UNDER_RPM) { underMs += dt * 1000.0f; if (underMs >= UNDER_TIME_MS) latchFault(F_UNDERSPEED); }
  else underMs = 0;
  if (fabs(wMeas - wPredWin) > ENC_PLAUS_RPM * RPM_TO_RADS) {
    plausMs += dt * 1000.0f;
    if (plausMs >= ENC_PLAUS_MS) latchFault(F_ENC_PLAUS);
  } else plausMs = 0;
  if (slpOn && isRunningState(state) && (int16_t)sgnf((float)uApplied) != BIAS_SIGN) latchFault(F_INTERNAL);
}

static void resetControl() {
  Hinit = false;
  smMask = 0; smQ = 0; smN = 0; smRblkN = 0; smRdAcc = smRAcc = 0; smPrevRdBlk = smPrevRdBlk2 = 0; smGap = 0;
  nrMask = 0; nrQ = 0; pinActive = false; satrActive = false;
  rLp = rMeas;
  dvRebase = true;
  rCmd = 0; rdDes = 0;
  rmActive = false; rmBad = 0;
}

static void updateLimited() {
  float wb = BIAS_SIGN * wMeas / RPM_TO_RADS;
  bool nearL = (wb < W_SOFT_LO_RPM + W_LIMIT_MARGIN_RPM) || (wb > W_SOFT_HI_RPM - W_LIMIT_MARGIN_RPM);
  bool clr = (wb > W_SOFT_LO_RPM + W_LIMIT_MARGIN_RPM + W_LIMIT_HYST_RPM) &&
             (wb < W_SOFT_HI_RPM - W_LIMIT_MARGIN_RPM - W_LIMIT_HYST_RPM);
  bool sat = !(rLoH <= 0.0f && 0.0f <= rHiH);
  bool tiltw = cosTilt < COS_TILT_SPEC;
  flags &= ~(FL_SAT | FL_TILT_WARN | FL_LIM_POS | FL_LIM_NEG);
  if (sat) flags |= FL_SAT;
  if (tiltw) flags |= FL_TILT_WARN;
  if (rHiH < R_MAX_DPS * DEG_TO_RAD) flags |= FL_LIM_POS;
  if (rLoH > -R_MAX_DPS * DEG_TO_RAD) flags |= FL_LIM_NEG;
  if (state == ST_ARMED_HOLD && (nearL || sat || tiltw)) { state = ST_LIMITED; pushEvent(EV_LIMITED, 1); }
  else if (state == ST_LIMITED && clr && !sat && !tiltw) { state = ST_ARMED_HOLD; pushEvent(EV_LIMITED, 0); }
}

static void captureTick(uint32_t now) {
  rLp += (rMeas - rLp) * (0.04f / CAPTURE_LPF_S);
  bool still = fabs(rLp) < CAPTURE_RATE_DPS * DEG_TO_RAD && cosTilt >= COS_TILT_SPEC;
  capStillMs = still ? capStillMs + 40.0f : 0.0f;
  if (capBlkN == 0) { capBlkRef = psiMeas; capBlkSum = 0; }
  capBlkSum += wrapPi(psiMeas - capBlkRef);
  capBlkStill = capBlkStill && still;
  if (++capBlkN >= 25) {
    capHaveMean = capBlkStill;
    if (capBlkStill) capLastMean = wrapPi(capBlkRef + capBlkSum / 25.0f);
    capBlkN = 0; capBlkStill = true;
    uint32_t el = (now - capT0) / 1000UL;
    if (capStillMs >= CAPTURE_STILL_MS && capHaveMean && el >= capMinDelayMs) {
      psiTarget = capLastMean;
      targetOffset = 0;
      targetValid = true;
      state = ST_ARMED_HOLD;
      dvRebase = true;
      pushEvent(EV_CAPTURED, (int16_t)(psiTarget * (RAD_TO_DEG * 100.0f)));
      rsT0 = now; rsActive = true;
      rs.peakErr_cdeg = 0; rs.tIn2_t40 = rs.tIn3_t40 = 0; rs.tLim_t40 = 0; rs.tSat_t40 = 0; rs.tEval_t40 = 0;
      rsSettleT0 = now;
      rs.wMin_drpm = 32767; rs.wMax_drpm = -32768; rs.faults = 0;
      if (profileActive) { profStep = 0; profT0 = now; }
    }
  }
  if ((uint32_t)(now - capT0) / 1000UL > CAPTURE_TIMEOUT_MS + capMinDelayMs && state == ST_CAPTURE) {
    state = ST_BIAS_IDLE;
    profileActive = false;
    pushEvent(EV_CAPTURE_TIMEOUT, 0);
  }
}

static void profileTick(uint32_t now) {
  if (!profileActive) return;
  uint32_t el = (now - profT0) / 1000UL;
  uint8_t holdS = (uint8_t)pgm_read_byte(&PROFILE[profStep][0]);
  if (el >= (uint32_t)holdS * 1000UL) {
    profStep++;
    profT0 = now;
    if (profStep >= PROFILE_LEN) { profileActive = false; flags &= ~FL_PROFILE; summaryFinish(1); return; }
    targetOffset = clampf((int8_t)pgm_read_byte(&PROFILE[profStep][1]) * DEG_TO_RAD,
                          -TARGET_OFFSET_MAX_DEG * DEG_TO_RAD, TARGET_OFFSET_MAX_DEG * DEG_TO_RAD);
    dvRebase = true;
    rsSettleT0 = now;
    pushEvent(EV_PROFILE, profStep);
  }
}

static void outerLoop(uint32_t now) {
  float rc;
  if (state == ST_ARMED_HOLD || state == ST_LIMITED) {
    profileTick(now);
    eHead = wrapPi(psiTarget + targetOffset - psiMeas);
    float rmax = R_MAX_DPS * DEG_TO_RAD * (state == ST_LIMITED ? LIMITED_RMAX_SCALE : 1.0f);
    rc = clampf(K_THETA * eHead, -rmax, rmax);
  } else if (state == ST_RATE_TEST) {
    if (rateStepActive && (uint32_t)(now - rateStepT0) >= RATE_STEP_MS * 1000UL) { rateTestCmd = 0; rateStepActive = false; }
    rc = rateTestCmd;
    eHead = 0;
  } else {
    rc = 0;
    eHead = 0;
    captureTick(now);
  }
  rCmd = (rLoH <= rHiH) ? clampf(rc, rLoH, rHiH) : 0.0f;
}

static void innerRateLoop(float dt) {
  float rmax2 = RDOT_MAX_DPS2 * DEG_TO_RAD;
  rdDes = clampf((rCmd - rMeas) / T_RATE, -rmax2, rmax2);
  float amax = ALPHA_MAX_RPMPS * RPM_TO_RADS;
  // conservation of angular momentum: wheel accel = -(J_b/J_w) * desired body accel
  alphaRef = clampf(-WHEEL_AXIS_SIGN * (J_LOCKED / J_WHEEL) * rdDes, -amax, amax);
  wRef += alphaRef * dt;
  float wb = BIAS_SIGN * wRef;
  float lo = W_SOFT_LO_RPM * RPM_TO_RADS, hi = W_SOFT_HI_RPM * RPM_TO_RADS;
  bool cl = false;
  if (wb < lo) { wb = lo; cl = true; }
  if (wb > hi) { wb = hi; cl = true; }
  float wLag = BIAS_SIGN * (wMeas + (wRm - wRmWin));
  float lead = REF_LEAD_MAX_RPM * RPM_TO_RADS;
  if (wb > wLag + lead) { wb = wLag + lead; cl = true; }
  if (wb < wLag - lead) { wb = wLag - lead; cl = true; }
  wb = clampf(wb, lo, hi);
  wRef = BIAS_SIGN * wb;
  if (cl) flags |= FL_WREF_CLAMP; else flags &= ~FL_WREF_CLAMP;
}

static void monitorsTick(uint32_t now, bool runOuter) {
  if (fabs(rMeas) > RATE_FLAG_DPS * DEG_TO_RAD) flags |= FL_RATE_FLAG; else flags &= ~FL_RATE_FLAG;
  bool following = !(flags & (FL_WREF_CLAMP | FL_PWM_CLAMP));
  if ((state == ST_RATE_TEST || state == ST_CAPTURE) && (flags & FL_WREF_CLAMP) &&
      fabs(rdDes) >= 0.999f * RDOT_MAX_DPS2 * DEG_TO_RAD) {
    if (!pinActive) { pinActive = true; pinT0 = now; }
    else if ((uint32_t)(now - pinT0) >= PIN_FAULT_MS * 1000UL) { latchFault(F_SIGN_FAULT); return; }
  } else pinActive = false;
  if ((state == ST_RATE_TEST || state == ST_CAPTURE) && (flags & FL_SAT)) {
    if (!satrActive) { satrActive = true; satrT0 = now; }
    else if ((uint32_t)(now - satrT0) >= SAT_REST_MS * 1000UL) { latchFault(F_SIGN_FAULT); return; }
  } else satrActive = false;
  smRdAcc += rdDes; smRAcc += rMeas;
  if (++smN >= 5) {
    float rdBlk = smRdAcc / smN, rBlk = smRAcc / smN;
    smRdAcc = smRAcc = 0; smN = 0;
    smRblk[0] = smRblk[1]; smRblk[1] = smRblk[2]; smRblk[2] = rBlk;
    if (smRblkN < 3) smRblkN++;
    float x = smPrevRdBlk2;
    if (smRblkN >= 3 && following && fabs(x) >= SIGNMON_MIN_DPS2 * DEG_TO_RAD) {
      smGap = 0;
      float y = (smRblk[2] - smRblk[0]) * 10.0f;
      uint8_t bad = (x * y < 0 && fabs(y) >= SIGNMON_MEAS_MIN_DPS2 * DEG_TO_RAD) ? 1 : 0;
      smMask = ((smMask << 1) | bad) & ((1U << SIGNMON_WINDOW) - 1U);
      if (smQ < SIGNMON_WINDOW) smQ++;
      flags |= FL_SIGNMON_ACTIVE;
      uint8_t nb = 0; for (uint8_t k = 0; k < SIGNMON_WINDOW; k++) nb += (smMask >> k) & 1;
      if (smQ >= SIGNMON_TRIP && nb >= SIGNMON_TRIP) { latchFault(F_SIGN_FAULT); return; }
      if (fabs(x) >= NORESP_MIN_DPS2 * DEG_TO_RAD) {
        uint8_t nr = (fabs(y) < NORESP_FRAC * fabs(x)) ? 1 : 0;
        nrMask = ((nrMask << 1) | nr) & ((1UL << NORESP_WINDOW) - 1UL);
        if (nrQ < NORESP_WINDOW) nrQ++;
        uint8_t nn = 0; for (uint8_t k = 0; k < NORESP_WINDOW; k++) nn += (nrMask >> k) & 1;
        if (nrQ >= NORESP_TRIP && nn >= NORESP_TRIP) { latchFault(F_NO_RESPONSE); return; }
      }
    } else if (++smGap >= SIGNMON_GAP_RESET) {
      smMask = 0; smQ = 0; smGap = SIGNMON_GAP_RESET;
    }
    smPrevRdBlk2 = smPrevRdBlk;
    smPrevRdBlk = rdBlk;
  }
  if (!runOuter) return;
  if (!rmActive) { rmActive = true; rmT0 = now; rmIntR = 0; rmDpsi = 0; }
  if (fabs(rmIntR) >= RATIO_MIN_DEG * DEG_TO_RAD || fabs(rmDpsi) >= RATIO_MIN_DEG * DEG_TO_RAD) {
    float ratio = (fabs(rmIntR) > 1e-6f) ? rmDpsi / rmIntR : 1e6f;
    if (ratio < RATIO_LO || ratio > RATIO_HI) { if (++rmBad >= RATIO_TRIP_COUNT) { latchFault(F_IMU_INCONSIST); return; } }
    else rmBad = 0;
    rmT0 = now; rmIntR = 0; rmDpsi = 0;
  } else if ((uint32_t)(now - rmT0) >= RATIO_WINDOW_MS * 1000UL) {
    rmT0 = now; rmIntR = 0; rmDpsi = 0;
  }
  if (state == ST_ARMED_HOLD || state == ST_LIMITED) {
    float ae = fabs(eHead);
    if (dvRebase) { dvRebase = false; dvTRef = now; dvERef = ae; dvPeak = ae; }
    if (ae > dvPeak) dvPeak = ae;
    uint32_t win = (flags & FL_SAT) ? DIVERGE_SAT_MS : DIVERGE_MS;
    if ((uint32_t)(now - dvTRef) >= win * 1000UL) {
      if (dvPeak > DIVERGE_DEG * DEG_TO_RAD && dvPeak > dvERef + DIVERGE_GROW_DEG * DEG_TO_RAD) { latchFault(F_DIVERGE); return; }
      dvTRef = now; dvERef = dvPeak; dvPeak = ae;
    }
  } else dvRebase = true;
  if (cosTilt < COS_TILT_FAULT) latchFault(F_TILT);
}

static void controlTick(uint32_t now, float dt, bool runOuter) {
  float Hraw = J_LOCKED * rMeas + WHEEL_AXIS_SIGN * J_WHEEL * wMeas;
  if (!Hinit) { Hf = Hraw; Hinit = true; }
  else Hf += (Hraw - Hf) * (dt < H_FILTER_TAU_S ? dt / H_FILTER_TAU_S : 1.0f);
  float k = (float)(BIAS_SIGN * WHEEL_AXIS_SIGN);
  float Hb = k * Hf;
  float lo = W_SOFT_LO_RPM * RPM_TO_RADS, hi = W_SOFT_HI_RPM * RPM_TO_RADS;
  float a = k * (Hb - J_WHEEL * hi) / J_LOCKED, b = k * (Hb - J_WHEEL * lo) / J_LOCKED;
  rLoH = a < b ? a : b; rHiH = a < b ? b : a;
  updateLimited();
  if (runOuter) outerLoop(now);
  if (state == ST_LATCHED_SAFE) return;
  innerRateLoop(dt);
  actuatorMap(dt);
  runningChecks(dt);
  if (state == ST_LATCHED_SAFE) return;
  monitorsTick(now, runOuter);
}

#endif

#if !BUILD_COMMISSIONING
static void enterSpinup(uint32_t now) {
  state = ST_SPINUP; suPhase = 1; suT0 = now; suC0 = encRead(); suRetries = 0; suWaiting = false;
  invBase = encInvRead();
  trim = 0; sdAcc = 0; uCont = 0;
}

static void spinupTick(uint32_t now, float dt) {
  if (suPhase == 1) {
    if (suWaiting) {
      if ((uint32_t)(now - suWaitT0) < 1000000UL) { motorCoast(); return; }
      suWaiting = false; suT0 = now; suC0 = encRead();
    }
    uint32_t el = (now - suT0) / 1000UL;
    float frac = clampf((float)el / KICK_RAMP_MS, 0.0f, 1.0f);
    int16_t mag = (int16_t)(PWM_KICK_START + (PWM_KICK_END - PWM_KICK_START) * frac + 0.5f);
    motorApply(BIAS_SIGN * mag);
    int32_t dc = encRead() - suC0;
    if ((dc >= KICK_DIRCHECK_COUNTS || dc <= -KICK_DIRCHECK_COUNTS) && (int8_t)sgnf((float)dc) != BIAS_SIGN) {
      latchFault(F_DIR_MISMATCH); return;
    }
    if (BIAS_SIGN * wMeas >= SPINUP_HANDOVER_RPM * RPM_TO_RADS) {
      suPhase = 2; wRef = wMeas; wRm = wMeas; wPred = wMeas;
      uCont = (float)(abs(uApplied) > PWM_KICK_START ? abs(uApplied) : PWM_KICK_START);
      trim = 0;
      return;
    }
    if (el >= KICK_RAMP_MS && dc < KICK_NOMOTION_COUNTS && dc > -KICK_NOMOTION_COUNTS) {
      motorCoast();
      if (suRetries < SPINUP_RETRIES) { suRetries++; suWaiting = true; suWaitT0 = now; }
      else latchFault(F_STALL);
      return;
    }
    if (el >= KICK_MAX_MS) latchFault(F_STALL);
  } else {
    float tgt = BIAS_SIGN * W_BIAS_RPM * RPM_TO_RADS;
    float step = SPINUP_RAMP_RPMPS * RPM_TO_RADS * dt;
    wRef += clampf(tgt - wRef, -step, step);
    actuatorMap(dt);
    runningChecks(dt);
    if (state != ST_SPINUP) return;
    if (fabs(wRef - tgt) < 0.5f * RPM_TO_RADS && fabs(wMeas - tgt) < 5.0f * RPM_TO_RADS) {
      state = ST_BIAS_IDLE; pushEvent(EV_STATE, ST_BIAS_IDLE);
    }
  }
}

static void spindownTick(float dt) {
  if (suPhase == 0) {
    float tgt = BIAS_SIGN * W_SOFT_LO_RPM * RPM_TO_RADS;
    float step = SPINDOWN_RAMP_RPMPS * RPM_TO_RADS * dt;
    wRef += clampf(tgt - wRef, -step, step);
    actuatorMap(dt);
    runningChecks(dt);
    if (state != ST_SPINDOWN) return;
    if (fabs(wRef - tgt) < 0.5f * RPM_TO_RADS) { suPhase = 1; motorCoast(); }
  } else {
    motorCoast();
    if (wheelStopped) { state = ST_DISARMED; targetValid = false; sessionMark(false); pushEvent(EV_STATE, ST_DISARMED); }
  }
}
#endif

#if BUILD_COMMISSIONING
static void commissionAbort(uint8_t reason) {
  motorCoast();
  state = ST_DISARMED; cmPhase = 0; cmAwaitY = false;
  pushEvent(EV_COMM, 100 + reason);
}

static void commissionTick(uint32_t now, float dt, bool runOuter) {
  uint32_t el = (now - cmT0) / 1000UL;
  switch (cmPhase) {
    case 1: {
      motorCoast();
      if (runOuter) {
        float d = wrapPi(psiMeas - cmPsiLast); cmPsiLast = psiMeas; cmPsiRel += d;
        if (cosTilt < COS_TILT_FAULT) { commissionAbort(1); return; }
        if (fabs(cmIntR) > 5.0f * DEG_TO_RAD) {
          float ratio = cmPsiRel / cmIntR;
          if (ratio < COMM_RATIO_LO || ratio > COMM_RATIO_HI) { if (cmBad < 255) cmBad++; }
        }
        if (cmIntR > cmMaxPos) cmMaxPos = cmIntR;
        if (cmIntR < cmMaxNeg) cmMaxNeg = cmIntR;
        bool done = cmMaxPos >= COMM_ROT_DEG * DEG_TO_RAD && cmMaxNeg <= -COMM_ROT_DEG * DEG_TO_RAD &&
                    fabs(cmIntR) < 5.0f * DEG_TO_RAD;
        if (done) {
          if (cmBad > 2) { commissionAbort(2); return; }
          cmPhase = 2; cmT0 = now; cmStillMs = 0;
          pushEvent(EV_COMM, 1);
        } else if (el > COMM_ROT_TIMEOUT_MS) { commissionAbort(3); return; }
      }
    } break;
    case 2:
      motorCoast();
      cmStillMs = (fabs(rMeas) < CAPTURE_RATE_DPS * DEG_TO_RAD) ? cmStillMs + dt * 1000.0f : 0;
      if (cmStillMs >= COMM_STILL_MS && wheelStopped) {
        if (!cmAwaitY) { cmAwaitY = true; cmReadyT0 = now; pushEvent(EV_COMM, 2); }
        else if ((uint32_t)(now - cmReadyT0) > COMM_READY_TIMEOUT_MS * 1000UL) { commissionAbort(10); return; }
      } else {
        cmAwaitY = false;
        if (el > 60000UL) { commissionAbort(4); return; }
      }
      break;
    case 3:
      motorCoast();
      cmR0 += rMeas; cmN++;
      if (fabs(rMeas) > CAPTURE_RATE_DPS * DEG_TO_RAD) { commissionAbort(11); return; }
      if (el >= COMM_BASE_MS) { cmR0 /= (cmN ? cmN : 1); cmPhase = 4; cmT0 = now; cmC0 = encRead(); }
      break;
    case 4: {
      float frac = clampf((float)el / KICK_RAMP_MS, 0.0f, 1.0f);
      motorApplyRaw(1, (int16_t)(PWM_KICK_START + (PWM_KICK_END - PWM_KICK_START) * frac + 0.5f));
      int32_t dc = encRead() - cmC0;
      if (dc >= COMM_MOTION_COUNTS || dc <= -COMM_MOTION_COUNTS) { cmPhase = 5; cmT0 = now; cmRp = cmWp = 0; cmNp = 0; }
      else if (el >= COMM_KICK_MAX_MS) { commissionAbort(5); return; }
    } break;
    case 5:
      motorApplyRaw(1, PWM_COMM_PLATEAU);
      if (fabs(wMeas) > COMM_MAX_RPM * RPM_TO_RADS) { commissionAbort(6); return; }
      if (el >= COMM_PLATEAU_MS - COMM_AVG_MS) { cmRp += rMeas; cmWp += wMeas; cmNp++; }
      if (el >= COMM_PLATEAU_MS) {
        cmRp /= (cmNp ? cmNp : 1); cmWp /= (cmNp ? cmNp : 1);
        cmDc = encRead() - cmC0;
        motorCoast();
        cmPhase = 6; cmT0 = now;
      }
      break;
    case 6:
      motorCoast();
      if (el >= COMM_COAST_MS) {
        float rEnd = rMeas;
        float dr = cmRp - cmR0;
        float gain = fabs(dr) / (fabs(cmWp) > 1e-6f ? fabs(cmWp) : 1e-6f);
        float gainNom = J_WHEEL / J_LOCKED;
        bool coastBack = ((rEnd - cmRp) * dr < 0) && fabs(rEnd - cmRp) >= 0.3f * fabs(dr);
        cmPhase = 7;
        if (fabs(dr) < COMM_MIN_DR_DPS * DEG_TO_RAD) { commissionAbort(7); cmPhase = 8; }
        else if (fabs(gain / gainNom - 1.0f) > COMM_GAIN_TOL) { commissionAbort(8); cmPhase = 8; }
        else if (!coastBack) { commissionAbort(9); cmPhase = 8; }
        else { state = ST_DISARMED; pushEvent(EV_COMM, 3); }
        (void)rEnd;
      }
      break;
  }
}
#endif

#if !BUILD_COMMISSIONING
static void summaryTick25Hz(uint32_t now) {
  if (!rsActive) return;
  if ((uint32_t)(now - rsSettleT0) >= SUMMARY_SETTLE_MS * 1000UL) {
    float ae = fabs(eHead) * RAD_TO_DEG;
    if (ae * 100.0f > rs.peakErr_cdeg) rs.peakErr_cdeg = (uint16_t)clampf(ae * 100.0f, 0, 65000);
    bool authority = (state == ST_ARMED_HOLD) && !(flags & FL_SAT);
    if (rs.tEval_t40 < 65535) rs.tEval_t40++;
    if (authority && ae <= 2.0f && rs.tIn2_t40 < 65535) rs.tIn2_t40++;
    if (authority && ae <= 3.0f && rs.tIn3_t40 < 65535) rs.tIn3_t40++;
  }
  if (state == ST_LIMITED && rs.tLim_t40 < 65535) rs.tLim_t40++;
  if ((flags & FL_SAT) && rs.tSat_t40 < 65535) rs.tSat_t40++;
  int16_t wd = (int16_t)(BIAS_SIGN * wMeas * (10.0f / RPM_TO_RADS));
  if (wd < rs.wMin_drpm) rs.wMin_drpm = wd;
  if (wd > rs.wMax_drpm) rs.wMax_drpm = wd;
}

static void summaryFinish(uint8_t reason) {
  if (!rsActive) return;
  rsActive = false;
  rs.magic = SUMMARY_MAGIC;
  rs.dur_s = (uint16_t)((micros() - rsT0) / 1000000UL);
  rs.faults = faults;
  rs.firstFault = firstFault;
  rs.endReason = reason;
  EEPROM.update(EE_SUMMARY, 0x00);
  rsWriteIdx = 1;
}

static void summaryWriteService() {
  if (rsWriteIdx == 0xFF) return;
  const uint8_t *b = (const uint8_t *)&rs;
  if (rsWriteIdx < sizeof(RunSummary)) { EEPROM.update(EE_SUMMARY + rsWriteIdx, b[rsWriteIdx]); rsWriteIdx++; }
  else { EEPROM.update(EE_SUMMARY, rs.magic); rsWriteIdx = 0xFF; }
}
#else
static void summaryFinish(uint8_t) {}
static void summaryWriteService() {}
#endif

static void stateMachineTick(uint32_t now, float dt, bool runOuter) {
  switch (state) {
#if BUILD_COMMISSIONING
    case ST_COMMISSION: commissionTick(now, dt, runOuter); break;
#else
    case ST_SPINUP: spinupTick(now, dt); break;
    case ST_BIAS_IDLE: {
      float tgt = BIAS_SIGN * W_BIAS_RPM * RPM_TO_RADS, step = SPINUP_RAMP_RPMPS * RPM_TO_RADS * dt;
      wRef += clampf(tgt - wRef, -step, step);
      actuatorMap(dt);
      runningChecks(dt);
    } break;
    case ST_RATE_TEST: case ST_CAPTURE: case ST_ARMED_HOLD: case ST_LIMITED:
      controlTick(now, dt, runOuter);
      if (runOuter && (state == ST_ARMED_HOLD || state == ST_LIMITED)) summaryTick25Hz(now);
      break;
    case ST_SPINDOWN: spindownTick(dt); break;
#endif
    default:
      if (slpOn) motorCoast();
      break;
  }
}

static void faultChecksFast(uint32_t now) {
  if (isDriveState(state)) {
    if (!gyroSeen || (uint32_t)(now - tGyro) > GYRO_STALE_US) latchFault(F_GYRO_STALE);
    if (!attSeen || (uint32_t)(now - tAtt) > ATT_STALE_US) latchFault(F_ATT_STALE);
    if ((uint16_t)(encInvRead() - invBase) > ENC_INVALID_TOTAL) latchFault(F_ENC_INVALID);
  }
#if USE_DRIVER_FLT
  if (digitalRead(PIN_FLT) == LOW) { if (++fltCount >= 2) { if (isDriveState(state)) latchFault(F_DRIVER_FLT); else faults |= F_DRIVER_FLT; } }
  else fltCount = 0;
#endif
}

static void innerTick(uint32_t now, float dt) {
  wheelEstimatorTick(now);
  modelsTick(dt);
  bool runOuter = (++outerDiv >= OUTER_DIV);
  if (runOuter) outerDiv = 0;
  stateMachineTick(now, dt, runOuter);
}

static uint32_t selfCheckT0 = 0;
static uint8_t selfCheckFail = 0;

static void enterSelfCheck(uint32_t now) {
  state = ST_SELF_CHECK; selfCheckT0 = now; invBase = encInvRead();
}

static void selfCheckTick(uint32_t now) {
  uint8_t fail = 0;
  if (!imuFresh(now)) fail |= 1;
  if (cosTilt < COS_TILT_FAULT) fail |= 2;
  if (encInvRead() != invBase) fail |= 4;
  if (!wheelStopped) fail |= 8;
  selfCheckFail = fail;
  uint32_t el = now - selfCheckT0;
  if (el < 1500000UL) return;
  if (fail == 0) {
    if (faults & F_UNEXPECTED_RESET) state = ST_LATCHED_SAFE;
    else state = ST_DISARMED;
    pushEvent(EV_STATE, state);
  } else if (el > SELF_CHECK_TIMEOUT_MS * 1000UL && (fail & 7)) {
    if (fail & 1) faults |= F_IMU_INIT;
    if (fail & 2) faults |= F_TILT;
    if (fail & 4) faults |= F_ENC_INVALID;
    pushEvent(EV_FAULT, (fail & 1) ? 19 : ((fail & 2) ? 5 : 7));
    state = ST_LATCHED_SAFE;
  }
}

static bool canBlockPrint() { return !isDriveState(state); }

static void printHelp() {
  if (!canBlockPrint()) { pushEvent(EV_REFUSED, '?'); return; }
#if BUILD_COMMISSIONING
  Serial.println(F("#HELP COMMISSIONING: C(+Y) D R T ?"));
#else
  Serial.println(F("#HELP W spin G rate A arm P prof +- step S spindown (all +Y) D disarm R T E ?"));
#endif
  Serial.println(F("#HELP UPPERCASE keys only; type nothing else"));
}

static void printStatus() {
  if (!canBlockPrint()) { pushEvent(EV_STATE, state); return; }
  Serial.print(F("#STATUS " FW_VERSION " state=")); Serial.print(state);
  Serial.print(F(" SIGN_COMMISSIONED=")); Serial.print(SIGN_COMMISSIONED);
  Serial.print(F(" MDS=")); Serial.print(MOTOR_DIR_SIGN);
  Serial.print(F(" SA=")); Serial.print(WHEEL_AXIS_SIGN);
  Serial.print(F(" BIAS=")); Serial.print(BIAS_SIGN);
  Serial.print(F(" faults=0x")); Serial.print(faults, HEX);
  Serial.print(F(" first=")); Serial.print(firstFault);
  Serial.print(F(" selfcheck_fail=0x")); Serial.print(selfCheckFail, HEX);
  Serial.println();
}

#if !BUILD_COMMISSIONING
static void printSummary() {
  if (!canBlockPrint()) { pushEvent(EV_REFUSED, 'E'); return; }
  RunSummary s; EEPROM.get(EE_SUMMARY, s);
  if (s.magic != SUMMARY_MAGIC) { Serial.println(F("#SUMMARY none")); return; }
  Serial.print(F("#SUMMARY dur_s=")); Serial.print(s.dur_s);
  Serial.print(F(" peak_err_cdeg=")); Serial.print(s.peakErr_cdeg);
  Serial.print(F(" t_eval_x40ms=")); Serial.print(s.tEval_t40);
  Serial.print(F(" t_in2deg_x40ms=")); Serial.print(s.tIn2_t40);
  Serial.print(F(" t_in3deg_x40ms=")); Serial.print(s.tIn3_t40);
  Serial.print(F(" w_min_drpm=")); Serial.print(s.wMin_drpm);
  Serial.print(F(" w_max_drpm=")); Serial.print(s.wMax_drpm);
  Serial.print(F(" t_limited_x40ms=")); Serial.print(s.tLim_t40);
  Serial.print(F(" t_saturated_x40ms=")); Serial.print(s.tSat_t40);
  Serial.print(F(" first_fault_bit=")); Serial.print(s.firstFault);
  Serial.print(F(" end=")); Serial.print(s.endReason);
  Serial.print(F(" faults=0x")); Serial.print(s.faults, HEX); Serial.println();
}

#endif
#if BUILD_COMMISSIONING
static void printCommissionResult() {
  float dr = cmRp - cmR0;
  int8_t mds = cmDc > 0 ? 1 : -1;
  int8_t sa = (dr * cmWp > 0) ? -1 : 1;
  float gain = fabs(dr) / (fabs(cmWp) > 1e-6f ? fabs(cmWp) : 1e-6f) / (J_WHEEL / J_LOCKED);
  Serial.println(F("#COMM ==== GROUND COMMISSIONING RESULT (nothing was saved) ===="));
  Serial.print(F("#COMM heading/rate check: PASS (bad samples ")); Serial.print(cmBad); Serial.println(F(")"));
  Serial.print(F("#COMM DIR HIGH -> encoder counts ")); Serial.println(cmDc);
  Serial.print(F("#COMM plateau wheel_rpm=")); Serial.print(cmWp / RPM_TO_RADS, 1);
  Serial.print(F(" body dr_dps=")); Serial.print(dr * RAD_TO_DEG, 2);
  Serial.print(F(" gain/expected=")); Serial.println(gain, 2);
  Serial.println(F("#COMM Copy into section 1 of the sketch, re-upload, then run C again to confirm MATCH:"));
  Serial.print(F("const int8_t MOTOR_DIR_SIGN  = ")); Serial.print(mds); Serial.println(';');
  Serial.print(F("const int8_t WHEEL_AXIS_SIGN = ")); Serial.print(sa); Serial.println(';');
  Serial.println(F("#define HEADING_RATE_SIGN_VERIFIED  1"));
  Serial.println(F("#define SIGN_COMMISSIONED           1"));
  Serial.print(F("#COMM compiled constants: "));
  Serial.println((mds == MOTOR_DIR_SIGN && sa == WHEEL_AXIS_SIGN) ? F("MATCH") : F("MISMATCH (edit + re-upload)"));
}
#endif

static void executeCommand(char c, uint32_t now) {
  switch (c) {
#if BUILD_COMMISSIONING
    case 'C':
      if (state == ST_DISARMED && faults == 0 && wheelStopped && imuFresh(now) && cosTilt >= COS_TILT_FAULT) {
        Serial.println(F("#COMM step1: motor OFF. Rotate >=20 deg each way through start, return, let go (45 s)."));
        invBase = encInvRead();
        state = ST_COMMISSION; cmPhase = 1; cmT0 = now; cmPsiRel = cmIntR = cmMaxPos = cmMaxNeg = 0; cmBad = 0;
        cmPsiLast = psiMeas; cmAwaitY = false; cmR0 = 0; cmN = 0;
      } else pushEvent(EV_REFUSED, c);
      break;
#else
    case 'C': pushEvent(EV_REFUSED, 'C'); break;
    case 'W':
      if (state == ST_DISARMED && SIGN_COMMISSIONED && faults == 0 && wheelStopped && imuFresh(now)) { sessionMark(true); enterSpinup(now); }
      else pushEvent(EV_REFUSED, c);
      break;
    case 'G':
      if (state == ST_BIAS_IDLE && SIGN_COMMISSIONED && faults == 0 && imuFresh(now)) { resetControl(); state = ST_RATE_TEST; rateTestCmd = 0; }
      else pushEvent(EV_REFUSED, c);
      break;
    case 'A': case 'P':
      if ((state == ST_BIAS_IDLE || state == ST_RATE_TEST) && SIGN_COMMISSIONED && faults == 0 && imuFresh(now)) {
        if (state == ST_BIAS_IDLE) resetControl();
        state = ST_CAPTURE; rLp = rMeas; capT0 = now; capStillMs = 0;
        capMinDelayMs = (c == 'P') ? PROFILE_MIN_DELAY_MS : 0;
        capBlkN = 0; capBlkStill = true; capHaveMean = false;
        profileActive = (c == 'P');
        if (profileActive) flags |= FL_PROFILE; else flags &= ~FL_PROFILE;
        EEPROM.update(EE_SUMMARY, 0x00);
      } else pushEvent(EV_REFUSED, c);
      break;
    case 'S':
      if (isRunningState(state)) { if (rsActive) summaryFinish(3); profileActive = false; targetValid = false; state = ST_SPINDOWN; suPhase = 0; }
      else pushEvent(EV_REFUSED, c);
      break;
    case '+': case '-': {
      float s = (c == '+') ? 1.0f : -1.0f;
      if ((state == ST_ARMED_HOLD || state == ST_LIMITED) && targetValid && !profileActive) {
        targetOffset = clampf(targetOffset + s * TARGET_STEP_DEG * DEG_TO_RAD,
                              -TARGET_OFFSET_MAX_DEG * DEG_TO_RAD, TARGET_OFFSET_MAX_DEG * DEG_TO_RAD);
        dvRebase = true;
        rsSettleT0 = now;
      } else if (state == ST_RATE_TEST) {
        rateTestCmd = s * RATE_STEP_DPS * DEG_TO_RAD; rateStepT0 = now; rateStepActive = true;
      } else pushEvent(EV_REFUSED, c);
    } break;
#endif
  }
}

static void serviceCommands(uint32_t now) {
  uint8_t n = 0;
  while (Serial.available() > 0 && n++ < 4) {
    char c = (char)Serial.read();
    if (c == 'd' || c == 'r' || c == 't' || c == 'e') c -= 32;
    switch (c) {
      case '\r': case '\n': case ' ': break;
      case 'D':
        if (isDriveState(state)) {
          motorCoast();
          if (rsActive) summaryFinish(3);
          state = ST_DISARMED; targetValid = false; profileActive = false;
          cmPhase = 0; cmAwaitY = false; sessionMark(false);
          pushEvent(EV_STATE, ST_DISARMED);
        }
        pendingCmd = 0;
        break;
      case 'R':
        pendingCmd = 0;
        if (state == ST_LATCHED_SAFE && wheelStopped) {
          bool imuFlt = (faults & F_IMU_ANY) != 0;
          if (faults & F_STICKY) { pushEvent(EV_STICKY, (int16_t)(faults & F_STICKY)); break; }
          faults = 0; firstFault = 0xFF; autoRecoveries = 0;
          sessionMark(false);
          if (imuFlt) state = ST_RECOVERY; else enterSelfCheck(now);
        } else pushEvent(EV_REFUSED, 'R');
        break;
#if !BUILD_COMMISSIONING
      case 'E': deferredPrint = 'E'; pendingCmd = 0; break;
#endif
      case 'Y':
        if (state == ST_COMMISSION && cmPhase == 2 && cmAwaitY && cmStillMs >= COMM_STILL_MS && wheelStopped) {
          cmAwaitY = false; cmPhase = 3; cmT0 = now; cmR0 = 0; cmN = 0;
        } else if (pendingCmd && (uint32_t)(now - pendingT) <= CONFIRM_MS * 1000UL) {
          executeCommand(pendingCmd, now);
        }
        pendingCmd = 0;
        break;
      case 'W': case 'C': case 'A': case 'G': case 'P': case 'S': case '+': case '-':
        pendingCmd = c; pendingT = now; pushEvent(EV_CONFIRM, c);
        break;
      case 'T': case '?': deferredPrint = c; pendingCmd = 0; break;
      default: pendingCmd = 0; break;
    }
  }
}

static void printHeader() {
  Serial.println(F("#HDR,t_us,state,tgt_cdeg,psi_cdeg,err_cdeg,r_mdps,rcmd_mdps,w_drpm,wref_drpm,walpha_rpmps,"
                   "enc,inv,taucmd_uNm,alphacmd_rpmps,u_pwm,mode,authp_mdps,authn_mdps,H_uNms,gyro_age_ms,"
                   "att_age_ms,loopmax_us,faults_hex,flags_hex,tilt_cdeg,trim_cpwm,stack_free,miss,gyro_gaps,grv_gaps,vbat_mv,"
                   "maxint_us,i2c_err,imu_bad,first_fault,pol_changes"));
}

static bool printEvent() {
  if (evHead == evTail) return false;
  if (Serial.availableForWrite() < 40) return true;
  Serial.print(F("#EVT,")); Serial.print(millis()); Serial.print(',');
  uint8_t c = evCode[evTail]; int16_t a = evArg[evTail];
  switch (c) {
    case EV_FAULT: Serial.print(F("FAULT_BIT,")); Serial.println(a); break;
    case EV_STATE: Serial.print(F("STATE,")); Serial.println(a); break;
    case EV_CAPTURED: Serial.print(F("CAPTURED_cdeg,")); Serial.println(a); break;
    case EV_LIMITED: Serial.print(F("LIMITED,")); Serial.println(a); break;
    case EV_REFUSED: Serial.print(F("REFUSED,")); Serial.println((char)a); break;
    case EV_CONFIRM: Serial.print(F("CONFIRM_Y,")); Serial.println((char)a); break;
    case EV_COMM: Serial.print(F("COMM,")); Serial.println(a); break;
    case EV_PROFILE: Serial.print(F("PROFILE_STEP,")); Serial.println(a); break;
    case EV_CAPTURE_TIMEOUT: Serial.println(F("CAPTURE_TIMEOUT,0")); break;
    case EV_STICKY: Serial.print(F("STICKY_POWERCYCLE,")); Serial.println(a); break;
    default: Serial.println(a); break;
  }
  evTail = (evTail + 1) & 3;
  return true;
}

static int32_t telemValue(uint8_t f, uint32_t now) {
  switch (f) {
    case 1: return state;
    case 2: return targetValid ? (int32_t)(wrapPi(psiTarget + targetOffset) * (RAD_TO_DEG * 100.0f)) : 0;
    case 3: return (int32_t)(psiMeas * (RAD_TO_DEG * 100.0f));
    case 4: return (int32_t)(eHead * (RAD_TO_DEG * 100.0f));
    case 5: return (int32_t)(rMeas * (RAD_TO_DEG * 1000.0f));
    case 6: return (int32_t)(rCmd * (RAD_TO_DEG * 1000.0f));
    case 7: return (int32_t)(wMeas * (10.0f / RPM_TO_RADS));
    case 8: return (int32_t)(wRef * (10.0f / RPM_TO_RADS));
    case 9: {
      float a = (wMeas - wAlphaEst) * (1.0f / RPM_TO_RADS) * 1e6f / (float)(now - tTelemPrev);
      wAlphaEst = wMeas; tTelemPrev = now;
      return (int32_t)a;
    }
    case 10: return encRead();
    case 11: return encInvRead();
    case 12: return (int32_t)(J_LOCKED * rdDes * 1e6f);
    case 13: return (int32_t)(alphaRef * (1.0f / RPM_TO_RADS));
    case 14: return uApplied;
    case 15: return slpOn ? (state == ST_SPINUP && suPhase == 1 ? 2 : 1) : 0;
    case 16: return (int32_t)(rHiH * (RAD_TO_DEG * 1000.0f));
    case 17: return (int32_t)(rLoH * (RAD_TO_DEG * 1000.0f));
    case 18: return (int32_t)((J_LOCKED * rMeas + WHEEL_AXIS_SIGN * J_WHEEL * wMeas) * 1e6f);
    case 19: return gyroSeen ? (int32_t)((now - tGyro) / 1000UL) : -1;
    case 20: return attSeen ? (int32_t)((now - tAtt) / 1000UL) : -1;
    case 21: return (int32_t)passMaxUs;
    case 22: return (int32_t)faults;
    case 23: return flags | (SIGN_COMMISSIONED ? 0 : FL_SIGN_UNCOMMISSIONED) | (targetValid ? FL_TARGET_VALID : 0);
    case 24: return (int32_t)(atan2(sqrt(R31 * R31 + R32 * R32), cosTilt) * (RAD_TO_DEG * 100.0f));
    case 25: return (int32_t)(trim * 100.0f);
    case 26: return stackFree;
    case 27: return deadlineMisses;
    case 28: return gyroGaps;
    case 29: return grvGaps;
#if USE_VBAT
    case 30: return (int32_t)(analogRead(A1) * VBAT_SCALE * 1000.0f);
#else
    case 30: return -1;
#endif
    case 31: return (int32_t)maxIntervalUs;
    case 32: return i2cErr;
    case 33: return imuBad;
    case 34: return (firstFault == 0xFF) ? -1 : firstFault;
    case 35: return polarityChanges;
  }
  return 0;
}

// 10 Hz CSV, one field per pass so Serial never stalls the 100 Hz loop
static void telemetryService(uint32_t now) {
  if (telField == 0) {
    if (printEvent()) return;
    if (deferredPrint) {
      char c = deferredPrint; deferredPrint = 0;
      if (c == 'T') printStatus();
      else if (c == '?') printHelp();
#if !BUILD_COMMISSIONING
      else if (c == 'E') printSummary();
#endif
      return;
    }
    if ((uint32_t)(now - tTelem) < TELEM_PERIOD_US) return;
    tTelem = now;
    if (Serial.availableForWrite() < 16) return;
    Serial.print(now);
    telField = 1;
    return;
  }
  while (telField <= 35 && Serial.availableForWrite() >= 14) {
    Serial.print(',');
    if (telField == 22 || telField == 23) Serial.print((uint32_t)telemValue(telField, now), HEX);
    else Serial.print(telemValue(telField, now));
    telField++;
  }
  if (telField > 35 && Serial.availableForWrite() >= 2) { Serial.print('\n'); telField = 0; passMaxUs = 0; maxIntervalUs = 0; }
}

extern uint8_t _end;
extern uint8_t __stack;
static void stackPaint() {
  uint8_t *p = &_end;
  uint8_t *sp = (uint8_t *)SP;
  while (p < sp - 48) *p++ = 0xC5;
}
static uint16_t stackFreeScan() {
  uint8_t *p = &_end;
  uint16_t n = 0;
  while (p <= &__stack && *p == 0xC5) { p++; n++; }
  return n;
}

static void diagnostics10Hz(uint32_t now) {
  static uint8_t d = 0;
  if (++d >= 10) {
    d = 0;
    stackFree = stackFreeScan();
    if (stackFree < STACK_MIN_FREE) latchFault(F_STACK);
  }
  if (state == ST_SELF_CHECK) selfCheckTick(now);
  if (state == ST_RECOVERY) {
    motorCoast();
#if USE_WATCHDOG
    wdt_disable();
#endif
    Wire.end();
    i2cStart();
    bool ok = imuInit();
#if USE_WATCHDOG
    wdt_enable(WDTO_250MS);
#endif
    tInner = tLastTick = micros();
    if (ok) { faults &= ~F_IMU_ANY; enterSelfCheck(micros()); }
    else { faults |= F_IMU_INIT; pushEvent(EV_FAULT, 19); state = ST_LATCHED_SAFE; }
  }
#if BUILD_COMMISSIONING
  if (state == ST_DISARMED && cmPhase == 7) { printCommissionResult(); cmPhase = 0; }
  if (state == ST_DISARMED && cmPhase == 8) {
    Serial.print(F("#COMM ABORT evidence: enc_counts=")); Serial.print(cmDc);
    Serial.print(F(" plateau_rpm=")); Serial.print(cmWp / RPM_TO_RADS, 1);
    Serial.print(F(" dr_dps=")); Serial.print((cmRp - cmR0) * RAD_TO_DEG, 2);
    Serial.println(F("  (see docs/ground_test_procedure.md, commissioning abort codes)"));
    cmPhase = 0;
  }
#endif
  summaryWriteService();
  if (pendingCmd && (uint32_t)(now - pendingT) > CONFIRM_MS * 1000UL) pendingCmd = 0;
}

void setup() {
  pinMode(PIN_SLP, OUTPUT); digitalWrite(PIN_SLP, LOW);  // SLP low = coast. G2 brakes while the Nano is in reset -> needs an SLP pull-down
  pinMode(PIN_PWM, OUTPUT); digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, LOW);
  stackPaint();
  state = ST_BOOT_SAFE;
  Serial.begin(SERIAL_BAUD);
  Serial.println();
  Serial.println(F("#BOOT " FW_VERSION " motor disabled"));
  Serial.print(F("#BOOT reset_mcusr=0x")); Serial.print(resetFlagsMcusr, HEX);
  Serial.print(F(" reset_r2=0x")); Serial.println(resetFlagsR2, HEX);
#if !SIGN_COMMISSIONED
  Serial.println(F("#BOOT SIGN_COMMISSIONED=0: W G A P refused"));
#endif
  if (EEPROM.read(EE_SESSION) == SESSION_ACTIVE) {
    faults |= F_UNEXPECTED_RESET;
    Serial.println(F("#BOOT UNEXPECTED_RESET: ack with R"));
  }
  EEPROM.update(EE_VERSION, 1);
#if USE_DRIVER_FLT
  pinMode(PIN_FLT, INPUT_PULLUP);
#endif
#if USE_BNO_INT
  pinMode(PIN_BNO_INT, INPUT_PULLUP);
#endif
  pinMode(PIN_ENC_A, INPUT); pinMode(PIN_ENC_B, INPUT);
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  EIFR = (1 << INTF0) | (1 << INTF1);
  encPrev = (PIND >> 2) & 0x03;
  interrupts();
  state = ST_SENSOR_INIT;
  i2cStart();
  if (!imuInit()) {
    faults |= F_IMU_INIT;
    Serial.println(F("#BOOT IMU init FAILED; R retries"));
  }
  printHeader();
  printHelp();
  if (faults & F_IMU_INIT) state = ST_LATCHED_SAFE;
  else enterSelfCheck(micros());
  tInner = tLastTick = micros();
#if USE_WATCHDOG
  wdt_enable(WDTO_250MS);
#endif
}

void loop() {
  uint32_t passStart = micros();
#if USE_WATCHDOG
  wdt_reset();
#endif
  uint32_t now = passStart;
  faultChecksFast(now);
  if ((uint32_t)(now - tInner) >= DT_INNER_US) {
    tInner += DT_INNER_US;
    if ((uint32_t)(now - tInner) >= DT_INNER_US) tInner = now;
    uint32_t interval = now - tLastTick;
    tLastTick = now;
    if (interval > maxIntervalUs) maxIntervalUs = interval;
    if (interval > LOOP_MISS_US) deadlineMisses++;
    if (interval > LOOP_LATE_US && isDriveState(state)) latchFault(F_LOOP_LATE);
    float dt = (float)(interval < 20000UL ? interval : 20000UL) * 1e-6f;
    innerTick(now, dt);
    if ((uint32_t)(now - tDiag) >= 100000UL) { tDiag = now; diagnostics10Hz(now); }
  }
  serviceCommands(now);
#if USE_BNO_INT
  if (digitalRead(PIN_BNO_INT) == LOW) imuService(micros());
  else telemetryService(micros());
#else
  imuService(micros());
  telemetryService(micros());
#endif
  uint32_t passUs = micros() - passStart;
  if (passUs > passMaxUs) passMaxUs = passUs;
}
