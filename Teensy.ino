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
#define Stage 1 //Define stage up here, there will be stage dependant code later

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
#define LoRaINT 8
#define RF95_FREQ 915.0 // EDIT THIS TO CHANGE FREQUENCY
// Bandwidth (seen below) Determines the head room of frequency we use, so with 125kHz, 915 would actually be 914.9375 - 915.0625
// You should be looking into getting a HAM by now, you should use that brain of yours to figure something out about bandwidth, signal drift, etc.
// Don't interfere with your own telemetry gang if that wans't obvious enough. Go at least 500 kHz different
// For FCC compliance stay in the USA ISM Band, 902-928 MHz, and don't conflict with GPS freq.
RH_RF95 rf95(LoRaCS, LoRaINT);

//DFR0515 Specific Pins
#define DFRCS 36

//Mosfets Gate Pins
#define Gate1 2
#define Gate2 3

#define GATE1_DURATION 1000
#define GATE2_DURATION 1000
#define WATER_DURATION 3000

#if Stage == 1
  #define GATE1_FIRE_TIME 1000
#elif Stage == 2
  #define GATE1_FIRE_TIME 3000
  #define GATE2_FIRE_TIME 1000
#endif

// For sensor change tracking

float prev_altitude = 0;
float altitude_velocity = 0;
float prev_accel_mag = 0;

// for stage seperation

bool separation_detected = false;
unsigned long separation_time = 0;

// for apogee 
bool apogee_detected = false; 

// This is creating an instance of the flight data packet for later use
FlightData currentPacket;

//Logic for time keeping
unsigned long lastTelemTime = 0;
const long Tele_Rate = 100; // Runs loop every 100 ms which is (10 Hz)
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
  if (!rf95.setFrequency(RF95_FREQ)) {
  Serial.println("LoRa frequency set failed");
  while(1);

  }
  //Here we set the radio power to 23 dBm. The false and 23 are from the radios documentation, I don't really understand this like that
  rf95.setTxPower(23, false);
  //false has to do with the PA_BOOST pin whatever that is
  rf95.setSpreadFactor(8);
  rf95.setSignalBandwidth(125E3);
  rf95.setCodingRate4(5);

  //Set up Sensors
  if (!bmp.begin_I2C()) {
    Serial.print("BMP Error");
    while(1);
  } 
  //BMP config, stuff straight out of the BMP3 Bosch code
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  for(int i=0; i<20; i++) { //logic for getting the pressure on the rail
    bmp.performReading();
    sum += bmp.pressure; // Pascals
    delay(20);
  }
  //Calibrate the pressure on the pad
  BaseAGLPressure = (sum / 20.0) / 100.0;
  Serial.print("Pad Pressure Calibrated to: ");
  Serial.println(BaseAGLPressure);

  if (!lsm6ds.begin_I2C()) {
    Serial.print("LSM Error");
    while(1);
  }

  // if (currentPacket.Stage == 1) {

  // enum RocketState {
  //   PAD          = 0,  // On ground, armed
  //   BOOST        = 1,  // Motor firing (accel > 3g)
  //   COAST        = 2,  // Post-burnout, ascending (accel < 1g)
  //   SEPARATION   = 3,  // Separation event detected
  //   DESCENT      = 4,  // Descending (altitude decreasing)
  //   LANDED       = 5   // On ground (low altitude + no movement)
  // };

  // }

  // if (currentPacket.Stage == 2) { 

  // enum RocketState {
  //   PAD           = 0,  // On ground, armed
  //   BOOST_S1      = 1,  // Stage 1 boosting (passenger)
  //   COAST_S1      = 2,  // Stage 1 coasting (passenger)
  //   SEPARATION    = 3,  // Separation detected
  //   COAST_S2      = 4,  // Post-separation coast
  //   BOOST_S2      = 5,  // Sustainer firing
  //   COAST_APOGEE  = 6,  // Coasting to apogee
  //   APOGEE        = 7,  // At apogee, deploy water
  //   DESCENT       = 8,  // Descending
  //   LANDED        = 9   // On ground
  // };

  // }


//LSM6DOX config
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  if (!adxl.begin()) {
    Serial.print("ADXL Error");
    while(1);
  }  
  adxl.setDataRate(ADXL343_DATARATE_100_HZ);
  currentPacket.state = 0; // setting equal to for On the ground 
  currentPacket.Stage = Stage;
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
  currentPacket.accelZLateral = event.acceleration.z; // Perpendicular accel

  //LSM Data
  sensors_event_t lsm_accel, lsm_gyro, lsm_temp;

  lsm6ds.getEvent(&lsm_accel, &lsm_gyro, &lsm_temp);

  currentPacket.gyroRoll = lsm_gyro.gyro.x;
  currentPacket.gyroPitch = lsm_gyro.gyro.y;
  currentPacket.gyroYaw = lsm_gyro.gyro.z;

  //Finally send the packet of flight data
  rf95.send((unit8_t *)&currentPacket, sizeof(currentPacket));
  rf95.waitPacketSent();

  //Debug of flgiht data
  Serial.print("+T"); Serial.print(currentPacket.timestamp);
  Serial.print(" | Alt: "); Serial.print(currentPacket.altitude);
  Serial.print(" | Thrust(G): "); Serial.print(currentPacket.accelThrust / 9.8); //In Gs
  Serial.print(" | Spin: "); Serial.println(currentPacket.gyroRoll);

  }

}
