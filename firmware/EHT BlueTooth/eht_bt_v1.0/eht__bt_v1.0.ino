//C++ script for EHT V1.0 hardware | BlueTooth version

#include <Wire.h>
#include <math.h>
#include <BleGamepad.h>

#define SDA_PIN  8
#define SCL_PIN  9
#define IMU_ADDR 0x69

TwoWire I2C_IMU = TwoWire(0);
BleGamepad bleGamepad("EHT V1.0", "DIY", 100);

// ── Accelerometer correction gain (0=gyro only, 1=acc only) ─
#define ALPHA_ACC 0.02f

volatile float q0=1, q1=0, q2=0, q3=0;
portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

float gx_off=0, gy_off=0, gz_off=0;

void imuWrite(byte reg, byte val) {
  I2C_IMU.beginTransmission(IMU_ADDR);
  I2C_IMU.write(reg); I2C_IMU.write(val);
  I2C_IMU.endTransmission(); delay(10);
}

int16_t imuRead16(byte reg) {
  I2C_IMU.beginTransmission(IMU_ADDR);
  I2C_IMU.write(reg); I2C_IMU.endTransmission(false);
  I2C_IMU.requestFrom(IMU_ADDR, (uint8_t)2);
  if (I2C_IMU.available() >= 2) {
    byte lo = I2C_IMU.read();
    byte hi = I2C_IMU.read();
    return (int16_t)(hi << 8 | lo);
  }
  return 0;
}

void calibrateGyro() {
  Serial.println("Gyro calibration (keep still for 2s)...");
  float sx=0, sy=0, sz=0;
  for (int i=0; i<200; i++) {
    sx += imuRead16(0x0C) / 16.4f;
    sy += imuRead16(0x0E) / 16.4f;
    sz += imuRead16(0x10) / 16.4f;
    delay(10);
  }
  gx_off=sx/200.0f; gy_off=sy/200.0f; gz_off=sz/200.0f;
  Serial.printf("Gyro offsets → X:%.3f Y:%.3f Z:%.3f\n", gx_off, gy_off, gz_off);
}

// ── High frequency task on Core 0 ───────────────────────────
// Integrates gx+gy+gz + accelerometer correction into quaternion
void imuTask(void* pvParameters) {
  unsigned long lastUs = micros();
  for (;;) {
    unsigned long now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;

    if (dt <= 0 || dt > 0.005f) continue;

    // Gyro reading (rad/s)
    float gx = (imuRead16(0x0C)/16.4f - gx_off) * DEG_TO_RAD;
    float gy = (imuRead16(0x0E)/16.4f - gy_off) * DEG_TO_RAD;
    float gz = (imuRead16(0x10)/16.4f - gz_off) * DEG_TO_RAD;

    // Accelerometer reading
    float ax = imuRead16(0x12) / 16384.0f;
    float ay = imuRead16(0x14) / 16384.0f;
    float az = imuRead16(0x16) / 16384.0f;

    portENTER_CRITICAL(&qMux);
    float lq0=q0, lq1=q1, lq2=q2, lq3=q3;
    portEXIT_CRITICAL(&qMux);

    // ── Accelerometer correction (Mahony) ─────────────────
    // Estimated gravity vector from current quaternion
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an > 0.5f && an < 1.5f) {
      ax/=an; ay/=an; az/=an;

      // Expected gravity in sensor frame
      float vx = 2*(lq1*lq3 - lq0*lq2);
      float vy = 2*(lq0*lq1 + lq2*lq3);
      float vz = lq0*lq0 - lq1*lq1 - lq2*lq2 + lq3*lq3;

      // Error = cross product between measured and estimated gravity
      float ex = ay*vz - az*vy;
      float ey = az*vx - ax*vz;
      float ez = ax*vy - ay*vx;

      // Proportional correction applied to gyro rates
      gx += ALPHA_ACC * ex;
      gy += ALPHA_ACC * ey;
      gz += ALPHA_ACC * ez;
    }

    // ── Quaternion integration (full gx gy gz) ────────────
    float h = 0.5f * dt;
    float nq0 = lq0 + h*(-lq1*gx - lq2*gy - lq3*gz);
    float nq1 = lq1 + h*( lq0*gx + lq2*gz - lq3*gy);
    float nq2 = lq2 + h*( lq0*gy - lq1*gz + lq3*gx);
    float nq3 = lq3 + h*( lq0*gz + lq1*gy - lq2*gx);

    // Normalize
    float n = sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
    nq0/=n; nq1/=n; nq2/=n; nq3/=n;

    portENTER_CRITICAL(&qMux);
    q0=nq0; q1=nq1; q2=nq2; q3=nq3;
    portEXIT_CRITICAL(&qMux);
  }
}

void setup() {
  Serial.begin(115200);
  I2C_IMU.begin(SDA_PIN, SCL_PIN, 400000);

  imuWrite(0x7E, 0xB6); delay(100); // Soft reset
  imuWrite(0x7E, 0x11); delay(100); // Accelerometer normal mode
  imuWrite(0x7E, 0x15); delay(100); // Gyro normal mode
  Serial.println("BMI160 OK");

  calibrateGyro();

  // Launch IMU task on Core 0 (loop() runs on Core 1)
  xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 2, NULL, 0);

  bleGamepad.begin();
  Serial.println("BLE started...");
}

void loop() {
  if (!bleGamepad.isConnected()) { delay(10); return; }

  // ── Thread-safe quaternion read ───────────────────────
  portENTER_CRITICAL(&qMux);
  float lq0=q0, lq1=q1, lq2=q2, lq3=q3;
  portEXIT_CRITICAL(&qMux);

  // ── Euler angles from quaternion ──────────────────────
  float pitch = asinf(constrain(2*(lq0*lq2 - lq3*lq1), -1.0f, 1.0f)) * RAD_TO_DEG;
  float roll  = atan2f(2*(lq0*lq1 + lq2*lq3),
                       1 - 2*(lq1*lq1 + lq2*lq2)) * RAD_TO_DEG;
  float yaw   = atan2f(2*(lq0*lq3 + lq1*lq2),
                       1 - 2*(lq2*lq2 + lq3*lq3)) * RAD_TO_DEG;

  // ── Map angles to BLE Gamepad range (0..32767) ────────
  int16_t out_pitch = (int16_t)map(pitch, -90.0f,  90.0f,  0, 32767);
  int16_t out_roll  = (int16_t)map(roll, -180.0f, 180.0f,  0, 32767);
  int16_t out_yaw   = (int16_t)map(yaw,  -180.0f, 180.0f,  0, 32767);

  bleGamepad.setX(out_pitch);
  bleGamepad.setY(out_roll);
  bleGamepad.setZ(out_yaw);
  bleGamepad.sendReport();

  Serial.printf("P:%6.1f° R:%6.1f° Y:%6.1f°\n", pitch, roll, yaw);
  delay(10);
}
