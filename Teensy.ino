// Titan Rocket Base line code for the Teensy system.

// This current build is a work of the Adafruit telemetry system.

#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX lsm6ds;
Adafruit_ADXL375 adxl(375); //Older package, needs an identifer.

//Create our Packagized data
struct FlightData {
  uint32_t timestamp; // time in milliseconds
  float altitude; // converted from bmp pressure
  float accelX_highG; // horizontal?
  float accelZ_highG; // Vertical ?
  float gyroX; // LSM6DSOX spin check and orientation following
  float gyroY;
  float gyroZ;
  uint8_t state; // 0-Pad, 1-Boost, 2-Coast, 3-Seperation, (Rest depends on stage 1 or 2)
};

// Create a single instance of the packet
FlightData currentPacket;

//Logic for time keeping
unsigned long lastTelemTime = 0;
const long Tele_Rate = 20; // Runs loop every 20 ms which is (50 Hz)
float BaseAGLPressure = 0;
float sum = 0;
// standard Arduino setup()
void setup() {
  Serial.begin(115200);
  Wire.begin(); //set up for "wire" infustructure. This is some low level teensy processer work. 
    //Using wire we automatically set pin 18/19 to I2C pins.
  if (!bmp.begin_I2C()) {
    serial_print("BMP Error");
    while(1);
  } 
  for(int i=0; i<10; i++) { //logic for getting the pressure on the rail
    bmp.performReading();
    sum += bmp.pressure; // Pascals
    delay(50);
  }

  BaseAGLPressure = (sum / 10.0) / 100.0;
  Serial.print("Pad Pressure Calibrated to: ");
  Serial.println(BaseAGLPressure);

  if (!lsm6ds.begin_I2C()) {
    serial_print("LSM Error");
    while(1);
  }
  if (!adxl.begin()) {
    serial_print("ADXL Error");
    while(1);
  }
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTelemTime >= Tele_Rate) {
    lastTelemTime = currentTime;

  // BMP Data fetch
  bmp.performReading();
  currentPacket.altitude = bmp.readAltitude(BaseAGLPressure); // 1013.25 is standard sea level pressure i think (millibars)
  currentPacket.timestamp = currentTime;

  sensors_event_t event; //Weird ADAfruit code
  adxl.getEvent(&event);

  currentPacket.accelX_highG = event.acceleration.x;
  currentPacket.accelZ_highG = event.accleration.z;

  }
}