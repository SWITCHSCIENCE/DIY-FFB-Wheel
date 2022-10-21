#include "Joystick.h"

#define DEBUG 0
#define ENABLE_FFB 1
#define EXISTS_OPTIONS 1

const float OFFSET = -6;  // Offset for Neutral
const int LOCK2LOCK = 1080;
const int LOCK2LOCK_HALF = LOCK2LOCK / 2;
const int TOTAL_GAIN = 100;                      // max 100%
const float FFBMaxForce = 30000;                 // 15A
const float ElectroMagneticForceCanceler = 180;  // 0.18A
const float DumperFactor = 75;                   // 200rpm -> -10000

// const unsigned long OutputCycle = 16666;  // 16.666ms
const unsigned long OutputCycle = 20000;  // 20ms
const int16_t xMax = 16384;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_MULTI_AXIS, 16, 1,
                   true, false, true,    // X,Y,Z
                   false, false, false,  // Rx,Ry,Rz
                   false, true,          // Rudder, Throttle,
                   true, true, true);    // Accelerator, Brake, Steering;

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

typedef struct {
  float Velocity;
  float Current;
  float Angle;
} Record;

bool sequentialMode = true;

int16_t debugV1 = 0;
int16_t debugV1min = 0;
int16_t debugV1max = 0;

void updateValues(int16_t *values) {
  const int pins[] = {A0, A1, A2, A3, A4, A5};
  const int16_t toMax[] = {4, 2, 32767, 32767, 32767, 32767};
  const int16_t toMin[] = {0, -1, 0, 0, 0, 0};
  static int16_t maxValues[] = {-32767, -32767, -32767, -32767, -32767, -32767};
  static int16_t minValues[] = {32767, 32767, 32767, 32767, 32767, 32767};
  static int16_t margins[] = {0, 0, 10, 5, 10, 5};
  for (int i = 0; i < 6; i++) {
    int16_t v = analogRead(pins[i]);
    if (v > maxValues[i]) maxValues[i] = v;
    if (v < minValues[i]) minValues[i] = v;
    int16_t diff = maxValues[i] - minValues[i];
    if (diff < 50) {
      values[i] = 0;
    } else {
      if (i == 0) {
        sequentialMode = false;
        Joystick.setButton(6, false);
        Joystick.setButton(7, false);
      }
      int16_t margin = diff * margins[i] / 100;
      if (v > maxValues[i] - margin) v = maxValues[i] - margin;
      values[i] = map(v, minValues[i] + margin, maxValues[i] - margin, toMin[i],
                      toMax[i]);
    }
  }
}

int16_t be2_to_int16(const uint8_t *data) {
  return ((uint16_t)data[0] << 8) | ((uint16_t)data[1] << 0);
}

Record decode(uint8_t *b) {
  Record rec;
  rec.Velocity = (float)be2_to_int16(&b[0]);
  rec.Current = (float)be2_to_int16(&b[2]);
  rec.Angle = ((float)((uint16_t)(b[4] & 0x7f) << 8 | ((uint16_t)b[5] << 0))) *
              360 / 32768;
  return rec;
}

void sendCmd(int32_t id, uint8_t *data, size_t len) {
  static uint8_t buff[13];
  buff[0] = len;
  buff[1] = (id >> 24) & 0xff;
  buff[2] = (id >> 16) & 0xff;
  buff[3] = (id >> 8) & 0xff;
  buff[4] = (id >> 0) & 0xff;
  memset(&buff[5], 0, 8);
  memcpy(&buff[5], data, len);
  Serial1.write(buff, sizeof(buff));
}

void sendValue(int16_t v) {
  uint16_t v16 = (uint16_t)v;
  static uint8_t buff[13];
  buff[0] = 2;
  buff[1] = 0;
  buff[2] = 0;
  buff[3] = 0;
  buff[4] = 0x32;
  buff[5] = v16 >> 8 & 0xff;
  buff[6] = v16 >> 0 & 0xff;
  buff[7] = 0;
  buff[8] = 0;
  buff[9] = 0;
  buff[10] = 0;
  buff[11] = 0;
  buff[12] = 0;
  Serial1.write(buff, sizeof(buff));
}

void printFrame(uint32_t id, uint8_t *buff, size_t len) {
  static char b[7];
  sprintf(b, "%03X", id);
  Serial.print("id:");
  Serial.print(b);
  Serial.print(" raw:");
  for (int i = 0; i < len; i++) {
    sprintf(b, "%02X", buff[i]);
    Serial.print(b);
  }
  Serial.println();
}

void SetupDDT() {
  const int config = 2;
  const int reset = 3;
  Serial1.begin(500000);
  Serial1.setTimeout(100);
  pinMode(config, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(config, 0);
  digitalWrite(reset, 1);
  delay(100);
  digitalWrite(reset, 0);
  delay(100);
  digitalWrite(config, 1);
  delay(100);
  Serial1.print("AT+MODE=PROTOL\r");
  Serial.print(Serial1.readString());
  delay(100);
  Serial1.print("AT+CAN=500,0,NDTF\r");
  Serial.print(Serial1.readString());
  delay(100);
  Serial1.print("AT+CANFLT=0,0,OFF\r");
  Serial.print(Serial1.readString());
  delay(100);
  Serial1.print("AT+UART=500000,8,1,NONE,NFC\r");
  Serial.print(Serial1.readString());
  delay(100);
  digitalWrite(config, 0);
  digitalWrite(reset, 1);
  delay(100);
  digitalWrite(reset, 0);
  delay(100);
  uint32_t id;
  uint8_t buff[16];
  size_t len;
  Serial.println("0x109");
  uint8_t b109[] = {1};
  sendCmd(0x109, b109, sizeof(b109));
  id = readFrame(buff, &len);
  printFrame(id, buff, len);
  /*
  Serial.print("0x107: ");
  uint8_t b107[] = {0x01, 0x01, 0x03, 0x04, 0xaa};
  sendCmd(0x107, b107, sizeof(b107));
  id = readFrame(buff, &len);
  printFrame(id, buff, len);
  */
  Serial.print("0x106: ");
  uint8_t b106[] = {0x80};
  sendCmd(0x106, b106, sizeof(b106));
  id = readFrame(buff, &len);
  printFrame(id, buff, len);
  Serial.print("0x105: ");
  // Control Mode
  // 0:Voltage-Open
  // 1:Current Loop
  // 2:Velocity Loop
  // 3:Angle Loop
  uint8_t b105[] = {0x00};
  sendCmd(0x105, b105, sizeof(b105));
  id = readFrame(buff, &len);
  printFrame(id, buff, len);
}

int getShift(int16_t x, int16_t y) {
  switch (x) {
    case 0:
      switch (y) {
        default:
          return 1;
        case 0:
          return 0;
        case -1:
          return 2;
      }
    case 1:
      switch (y) {
        default:
          return 3;
        case 0:
          return 0;
        case -1:
          return 4;
      }
    case 2:
      switch (y) {
        default:
          return 5;
        case 0:
          return 0;
        case -1:
          return 6;
      }
    default:
      switch (y) {
        default:
          return 7;
        case 0:
          return 0;
        case -1:
          return -1;
      }
  }
}

#define N 13

uint32_t readFrame(uint8_t *data, size_t *len) {
  *len = 0;
  static uint8_t buff[N];
  static uint8_t index = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    // Serial.println(b, HEX);
    buff[index++] = b;
    index %= N;
    if (b == 0 && buff[(index + N - 2) % N] == 0x55) {
      uint8_t ri = (index + N - 13) % N;
      if (buff[ri] != 8) continue;
      uint32_t id = 0;
      for (int i = 0; i < 4; i++) {
        id <<= 8;
        id |= (uint32_t)(buff[(ri + i + 1) % N]);
      }
      for (int i = 0; i < 8; i++) {
        *(data++) = buff[(ri + i + 5) % N];
        (*len)++;
      }
      return id;
    }
  }
  return 0;
}

void request(bool force = false) {
  static unsigned long requested = 0;
  static uint8_t b[] = {0x01, 0x01, 0x02, 0x04, 0x55};
  unsigned long now = micros();
  if (force || now - requested > OutputCycle) {
    sendCmd(0x107, b, sizeof(b));
    requested = now;
  }
}

void setup() {
#if DEBUG > 0
  Serial.begin(230400);
  while (!Serial) delay(100);
#endif
#if ENABLE_PID > 0
  Input = 0;
  Setpoint = 0;
  myPID.SetOutputLimits(-LockMaxForce, LockMaxForce);
  // myPID.SetSampleTimeUs(10000);
  myPID.SetMode(myPID.Control::automatic);
  myPID.SetTunings(Kp, Ki, Kd);
  // myPID.SetDerivativeMode(QuickPID::dMode::dOnError);
#endif
  SetupDDT();
  Serial.println("start!");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  // pinMode(A6, INPUT);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Joystick.setXAxisRange(-xMax, xMax);
  Joystick.setYAxisRange(-32767, 32767);
  Joystick.setZAxisRange(0, 32767);
  Joystick.setRxAxisRange(-32767, 32767);
  Joystick.setRyAxisRange(-32767, 32767);
  Joystick.setRzAxisRange(-32767, 32767);
  Joystick.setRudderRange(0, 32767);
  Joystick.setThrottleRange(0, 32767);
  Joystick.setAcceleratorRange(0, 32767);
  Joystick.setBrakeRange(0, 32767);
  Joystick.setSteeringRange(-xMax, xMax);

  // set X Axis gains
  mygains[0].totalGain = TOTAL_GAIN;  // 0-100
  mygains[0].springGain = 100;        // 0-100
  mygains[0].constantGain = 100;      // 0-100
  mygains[0].rampGain = 100;          // 0-100
  mygains[0].squareGain = 100;        // 0-100
  mygains[0].sineGain = 100;          // 0-100
  mygains[0].triangleGain = 100;      // 0-100
  mygains[0].sawtoothdownGain = 100;  // 0-100
  mygains[0].sawtoothupGain = 100;    // 0-100
  mygains[0].springGain = 100;        // 0-100
  mygains[0].damperGain = 100;        // 0-100
  mygains[0].frictionGain = 100;      // 0-100
  mygains[0].customGain = 100;        // 0-100
  // enable gains REQUIRED
  Joystick.setGains(mygains);
  Joystick.begin(false);
}

void loop() {
  static double angle = 0;
  static float offsetAngle = 0;
  static float oldAngle = 0;
  static uint8_t buff[8];
  static int shift = 0;
  static size_t len;
  static int cnt = 0;

  int16_t x =
      map(-angle * 8, -LOCK2LOCK_HALF * 8, LOCK2LOCK_HALF * 8, -xMax, xMax);
  uint32_t tm = micros();
  static uint32_t last = 0;
  if (tm - last > 500) {
    // Steering wheel
    Joystick.setXAxis(x);
    Joystick.setSteering(x);
    // Send HID data to PC
    Joystick.sendState();

    // caculate forces
    myeffectparams[0].springMaxPosition = xMax;
    myeffectparams[0].springPosition = x;
    Joystick.setEffectParams(myeffectparams);
    Joystick.getForce(forces);
    last = tm;
  }

  cnt++;

  request();
  uint32_t id = readFrame(buff, &len);
  if (id == 151) {
    // Serial.println(micros());
    Record rec = decode(buff);
    if (oldAngle < 90 && rec.Angle > 270) {
      offsetAngle -= 360;
    }
    if (oldAngle > 270 && rec.Angle < 90) {
      offsetAngle += 360;
    }
    oldAngle = rec.Angle;
    angle = offsetAngle + rec.Angle + OFFSET;

#if EXISTS_OPTIONS > 0
    static int16_t values[6];

    updateValues(values);

    if (sequentialMode) {
      switch (values[1]) {
        default:
          Joystick.setButton(6, true);
          Joystick.setButton(7, false);
          break;
        case 0:
          Joystick.setButton(6, false);
          Joystick.setButton(7, false);
          break;
        case -1:
          Joystick.setButton(6, false);
          Joystick.setButton(7, true);
          break;
      }
    } else {
      shift = getShift(values[0], values[1]);
      for (int i = 0; i < 7; i++) {
        Joystick.setButton(i + 8, i + 1 == shift);
      }
      Joystick.setButton(7 + 8, shift == -1);
    }
    Joystick.setHatSwitch(0, -1);

    // Handbrake
    Joystick.setZAxis(values[2]);
    // Joystick.setRudder(values[2]);

    // Throttle
    // Joystick.setZAxis(values[3]);
    Joystick.setThrottle(values[3]);

    // Brake
    // Joystick.setRzAxis(values[4]);
    Joystick.setBrake(values[4]);

    // Clutch
    // Joystick.setRyAxis(values[5]);
    Joystick.setAccelerator(values[5]);
#endif

    // Recv HID-PID data from PC
    float FFBForce = (float)(forces[0]) / 255 * FFBMaxForce;
    float output = FFBForce;
    output += ElectroMagneticForceCanceler * rec.Velocity;
    output -= rec.Velocity * DumperFactor;
    if (output > 32767) output = 32767;
    if (output < -32767) output = -32767;
#if DEBUG > 0
    Serial.print(millis());
    Serial.print(": a:");
    Serial.print(rec.Angle);
    Serial.print(": c:");
    Serial.print(rec.Current);
    Serial.print(": v:");
    Serial.print(rec.Velocity);
    Serial.print(": X:");
    Serial.print(values[0]);
    Serial.print(": Y:");
    Serial.print(values[1]);
    Serial.print(": Shift:");
    Serial.print(shift);
    Serial.print(": A:");
    Serial.print(angle);
    Serial.print(" FFB:");
    Serial.print(FFBForce);
    Serial.print(" Output:");
    Serial.print(output);
    Serial.print(" Count:");
    Serial.print(cnt);
    Serial.println();
#endif
    cnt = 0;
    sendValue((int16_t)(output));
  }
}