#include <Wire.h>
#include <Adafruit_MLX90393.h>

Adafruit_MLX90393 sensor;

// hardcoded calibration 
const float X_OFFSET = -47.5498f;
const float Y_OFFSET = -23.2500f;
const float X_SCALE  =  0.00010226f;
const float Y_SCALE  =  0.00010226f;

// angle tracking
float _prev_raw    = 0.0f;
float _angle       = 0.0f;
bool  _initialised = false;

void setup() {
  //Establish I2C serial comm
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!sensor.begin_I2C()) {
    Serial.println("ERROR: Sensor not found!");
    while (1) { delay(10); }
  }

  //Set MLX90393 settings -- reference data sheet
  //https://cdn-learn.adafruit.com/downloads/pdf/mlx90393-wide-range-3-axis-magnetometer.pdf
  //Melaxis datasheet
  sensor.setOversampling(MLX90393_OSR_3);
  sensor.setFilter(MLX90393_FILTER_5);
  sensor.setGain(MLX90393_GAIN_1X);
  sensor.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  Serial.println("angle_deg,x_uT,y_uT,loop_us");
}

void loop() {
  float x, y, z;
  //return number of microseconds
  unsigned long t0 = micros();
  //read x,y,z data
  if (!sensor.readData(&x, &y, &z)) return;

  //adjust read data by calibrated offset/scale 
  float xn = (x - X_OFFSET) * X_SCALE;
  float yn = (y - Y_OFFSET) * Y_SCALE;

  //calculate angle degrees based on x/y values
  float raw = atan2f(yn, xn) * 180.0f / M_PI;

  //set start point to where system reads at powerup
  if (!_initialised) {
    _prev_raw    = raw;
    _initialised = true;
  }

  //calculations for determining angle
  float diff = raw - _prev_raw;
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;

  _prev_raw = raw;

  if (fabsf(diff) < 90.0f) {
    _angle -= diff;
  }

  //just reading initially has angle off by 1/4 of a mechanical rotation,
  //divided initial angle by 1.54 then modified the angle degree to scale
  //the calculation to measure one full mechanical rotation per 360 degrees
  float angle_deg = fmodf(_angle, 360.0f);
  if (angle_deg < 0.0f) angle_deg += 360.0f;

  angle_deg = angle_deg / 1.54f;

  angle_deg = fmodf(_angle / 1.54f, 360.0f);
  if (angle_deg < 0.0f) angle_deg += 360.0f;

  unsigned long elapsed = micros() - t0;

  //print data
  Serial.print(angle_deg, 2);  Serial.print(",");
  Serial.print(x, 2);          Serial.print(",");
  Serial.print(y, 2);          Serial.print(",");
  Serial.println(elapsed);
}