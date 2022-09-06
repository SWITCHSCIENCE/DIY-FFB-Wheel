#include "Joystick.h"
#include "QuickPID.h"

#define DEBUG 0
#define EXISTS_OPTIONS 0

// for Lock PID
const float Kp = 20, Ki = 0, Kd = 1.6;
float Setpoint, Input, Output;
QuickPID myPID(&Input, &Output, &Setpoint);

const float OFFSET = -6;  // Offset for Neutral
const int LOCK2LOCK = 540;
const int LOCK2LOCK_HALF = LOCK2LOCK / 2;
const float CenteringForce = 1;           // 0.001A
const float FFBMaxForce = 5000;           // 5A
const float LockMaxForce = 10000;         // 10A
const unsigned long OutputCycle = 20000;  // 20ms
const int16_t xMax = 512;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_MULTI_AXIS, 16, 1,
                   true, false, true,    // X,Y,Z
                   false, true, true,    // Rx,Ry,Rz
                   false, true,          // Rudder, Throttle,
                   false, false, true);  // Accelerator, Brake, Steering;

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

typedef struct {
  float Verocity;
  float Current;
  float Angle;
} Record;

bool sequentialMode = true;

void updateValues(int16_t *values) {
  const int pins[] = {A0, A1, A2, A3, A4, A5};
  const int16_t toMax[] = {4, 2, 32767, 32767, 32767, 32767};
  const int16_t toMin[] = {0, -1, 0, 0, 0, 0};
  static int16_t maxValues[7] = {-32767, -32767, -32767,
                                 -32767, -32767, -32767};
  static int16_t minValues[7] = {32767, 32767, 32767, 32767, 32767, 32767};
  for (int i = 0; i < 6; i++) {
    int16_t v = analogRead(pins[i]);
    if (v > maxValues[i]) maxValues[i] = v;
    if (v < minValues[i]) minValues[i] = v;
    if (maxValues[i] - minValues[i] < 50) {
      values[i] = 0;
    } else {
      if (i == 0) {
        sequentialMode = false;
      }
      values[i] = map(v, minValues[i], maxValues[i], toMin[i], toMax[i]);
    }
  }
}

int16_t be2_to_int16(const uint8_t *data) {
  return ((uint16_t)data[0] << 8) | ((uint16_t)data[1] << 0);
}

Record decode(uint8_t *b) {
  Record rec;
  rec.Verocity = (float)be2_to_int16(&b[0]);
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
  static uint8_t buff[7];
  buff[0] = 2;
  buff[1] = 0;
  buff[2] = 0;
  buff[3] = 0;
  buff[4] = 0x32;
  buff[5] = v16 >> 8 & 0xff;
  buff[6] = v16 >> 0 & 0xff;
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
  Serial1.setTimeout(50);
  pinMode(config, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(config, 0);
  digitalWrite(reset, 1);
  delay(500);
  digitalWrite(reset, 0);
  delay(500);
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
  delay(200);
  digitalWrite(config, 0);
  digitalWrite(reset, 1);
  delay(500);
  digitalWrite(reset, 0);
  delay(500);
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
  uint8_t b105[] = {0x01};
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
#endif
  Input = 0;
  Setpoint = 0;
  myPID.SetOutputLimits(-LockMaxForce, LockMaxForce);
  myPID.SetSampleTimeUs(5000);
  myPID.SetMode(myPID.Control::automatic);
  myPID.SetTunings(Kp, Ki, Kd);
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
  Joystick.setYAxisRange(0, 32767);
  Joystick.setZAxisRange(0, 32767);
  Joystick.setRxAxisRange(-32767, 32767);
  Joystick.setRyAxisRange(-32767, 32767);
  Joystick.setRzAxisRange(0, 32767);
  Joystick.setRudderRange(0, 32767);
  Joystick.setThrottleRange(0, 32767);
  Joystick.setAcceleratorRange(0, 32767);
  Joystick.setBrakeRange(0, 32767);
  // Steering wheel
  // Joystick.setSteeringRange(-LOCK2LOCK_HALF * 60, LOCK2LOCK_HALF * 60);
  Joystick.setSteeringRange(-xMax, xMax);
  // set X Axis gains
  mygains[0].totalGain = 50;    // 0-100
  mygains[0].springGain = 100;  // 0-100
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
  size_t len;

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
  Joystick.setHatSwitch(0, 8);

  // Handbrake
  Joystick.setZAxis(values[2]);
  // Joystick.setRudder(values[2]);

  // Throttle
  // Joystick.setZAxis(values[3]);
  Joystick.setThrottle(values[3]);

  // Brake
  Joystick.setRzAxis(values[4]);
  // Joystick.setBrake(values[4]);

  // Clutch
  Joystick.setRyAxis(values[5]);
  // Joystick.setAccelerator(values[5]);
#endif

  // Recv HID-PID data from PC and caculate forces
  // Steering wheel
  int16_t x = map(-angle, -LOCK2LOCK_HALF, LOCK2LOCK_HALF, -xMax, xMax);
  Joystick.setXAxis(x);
  Joystick.setSteering(x);

  // Send HID data to PC
  Joystick.sendState();

  myeffectparams[0].springMaxPosition = xMax;
  myeffectparams[0].springPosition = x;
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  float FFBForce = (float)(forces[0]) / 255 * FFBMaxForce;

  Input = angle;
  float Offset = 0.0;
  if (angle > LOCK2LOCK_HALF) {
    Setpoint = LOCK2LOCK_HALF;
    myPID.SetOutputLimits(-LockMaxForce, LockMaxForce);
    Offset = -CenteringForce;
  } else if (angle < -LOCK2LOCK_HALF) {
    Setpoint = -LOCK2LOCK_HALF;
    myPID.SetOutputLimits(-LockMaxForce, LockMaxForce);
    Offset = CenteringForce;
  } else {
    Setpoint = 0;
    myPID.SetOutputLimits(-CenteringForce, CenteringForce);
  }
  myPID.Compute();
  float LockForce = Output + Offset;
  float output = LockForce + FFBForce;
#if DEBUG > 0
  Serial.print("A:");
  Serial.print(angle);
  Serial.print(" FFB:");
  Serial.print(FFBForce);
  Serial.print(" LOCK:");
  Serial.print(LockForce);
  Serial.print(" Output:");
  Serial.print(output);
  Serial.println();
#endif

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
#if DEBUG > 0
    sendValue(0);
#else
    sendValue((int16_t)(output));
#endif
  }
}