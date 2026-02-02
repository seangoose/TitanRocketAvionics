// Titan Rocket Base line code for the Teensy system.

// This current build is a work of the Adafruit telemetry system.


// Libraries
#include <Wire.h>// Automatically pulls the I2C wires to I2
#include <SPI.h> // Automatically pulls hte SPI pins to SPI
#include <RH_RF95.h> //OG Radio Head library, look into this if bored
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
const unsigned int Stage = 1; //Define stage up here, there will be stage dependant code later

//Sensor Set up
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX lsm6ds;
Adafruit_ADXL375 adxl(375); //Older ADAfruit set up package, needs an identifer.

//Create our Packagized data
struct FlightData {
  uint32_t timestamp; // time in milliseconds
  float altitude; // converted from bmp pressure
  float accelXThrust; // horizontal?
  float accelZLateral; // Vertical ?
  float gyroRoll; // LSM6DSOX spin check and orientation following
  float gyroPitch; // Both pitch and yaw relate to the rockets tilt around the Z axis or think of the xz, and yz planes
  float gyroYaw;
  uint8_t state; // 0-Pad, 1-Boost, 2-Coast, 3-Seperation, (Rest depends on stage 1 or 2)
  uint8_t Stage; //Stage 1 or 2
};

//LoRa specific pins
#define LoRaCS 37
#define LoRaRST 9
#define LoRaGO 8
#define RF95_FEQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//DFR0515 Specific Pins
#define DFRCS 36

//Mosfets Gate Pins
#define Gate1 2
#define Gate2 3

// This is creating an instance of the flight data packet for later use
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

  //Radio Set UP
  //Use RST to make sure the radio has boot up correcrtly
  pinMode(LoRaRST, OUTPUT);
  digitalWrite(LoRaRST, HIGH);
  delay(10);
  digitalWrite(LoRaRST, LOW); // Pulling RST to low makes the radio reset
  delay(10);
  digitalWrite(LoRaRST, HIGH); // Pulling back to high ensures the radio stays on.
  // Continued intillization of Radio
  if (!rf95.init()) {
    Serial.println("LoRa set up failed");
    while(1);
  }

  //Here we set the radio power to 23 dBm. The false and 23 are from the radios documentation, I don't really understand this like that
  rf95.setTxPower(23, false);
  //false has to do with the PA_BOOST pin whatever that is

  //Set up Sensors
  if (!bmp.begin_I2C()) {
    serial_print("BMP Error");
    while(1);
  } 
  for(int i=0; i<20; i++) { //logic for getting the pressure on the rail
    bmp.performReading();
    sum += bmp.pressure; // Pascals
    delay(20);
  }
  //Calibrate the pressure on the pad
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

  FlightData.state = 0; // setting equal to for On the ground 
}

//Start main loop by reading data
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTelemTime >= Tele_Rate) {
    lastTelemTime = currentTime;

  // Sensor data goes: Update Sensor -> Read Sensor Data -> Fill Pakcet

  // BMP Data fetch
  bmp.performReading(); //
  currentPacket.altitude = bmp.readAltitude(BaseAGLPressure) * 3.28084; // BMP returns height in meters
  currentPacket.timestamp = currentTime;


  //Adxl Data
  sensors_event_t event; //Weird ADAfruit code
  adxl.getEvent(&event);
  currentPacket.accelThrust = event.acceleration.x; //Becuase of how its mounted this is our vertical accel
  currentPacket.accelZLateral = event.accleration.z; // Perpendicular accel

  //LSM Data
  sensors_event_t lsm_accel, lsm_gyro, lsm_temp;

  lsm6ds.getEvent(&lsm_accel, &lsm_gyro, &lsm_temp);

  currentPacket.gyroRoll = lsm_gyro.gyro.x;
  currentPacket.gyroPitch = lsm_gyro.y;
  currentPacket.gyroYaw = lsm_gyro.z;

  //Finally send the packet of flight data
  rf95.send((unit_8t))


  //Debug of flgiht data
  Serial.print("+T"); Serial.print(currentPacket.timestamp);
  Serial.print(" | Alt: "); Serial.print(currentPacket.altitude);
  Serial.print(" | Thrust(G): "); Serial.print(currentPacket.accelThrust / 9.8); //In Gs
  Serial.print(" | Spin: "); Serial.println(currentPacket.gyroRoll);

  }
}
