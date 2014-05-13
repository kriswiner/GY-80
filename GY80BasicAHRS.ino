/* GY-80 Basic Example Code
 by: Kris Winer
 date: May 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic functionality of the 10DoF sensor GY-80 including parameterizing the register addresses, 
 initializing the sensor, getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Added display functions to allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using 
 open source Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the GY-80 breakout board.
 
 Hardware setup:
 GY-80 Breakout --------- Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The GY-80 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include "Wire.h"  
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-9150 Register Map and Descriptions, Revision 4.0, RM-MPU-9150A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU 9150 are virtually identical but the latter has an on-board magnetic sensor
//
// ADXL345 Accelerometer Registers
#define WHO_AM_I_ADXL345        0x00   // Should return 0xE5
#define ADXL345_THRESH_TAP      0x1D   // Tap threshold
#define ADXL345_OFSX            0x1E   // X-axis offset
#define ADXL345_OFSY            0x1F   // Y-axis offset
#define ADXL345_OFSZ            0x20   // Z-axis offset
#define ADXL345_DUR             0x21   // Tap duration
#define ADXL345_LATENT          0x22   // Tap latency
#define ADXL345_WINDOW          0x23   // Tap window
#define ADXL345_THRESH_ACT      0x24   // Activity threshold
#define ADXL345_THRESH_INACT    0x25   // Inactivity threshold
#define ADXL345_TIME_INACT      0x26   // Inactivity time
#define ADXL345_ACT_INACT_CTL   0x27   // Axis enable control for activity/inactivity detection
#define ADXL345_THRESH_FF       0x28   // Free-fall threshold
#define ADXL345_TIME_FF         0x29   // Free-fall time
#define ADXL345_TAP_AXES        0x2A   // Axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS  0x2B   // Source of single/double tap
#define ADXL345_BW_RATE         0x2C   // Data rate and power mode control
#define ADXL345_POWER_CTL       0x2D   // Power-saving features control
#define ADXL345_INT_ENABLE      0x2E   // Interrupt enable control
#define ADXL345_INT_MAP         0x2F   // Interrupt mapping control
#define ADXL345_INT_SOURCE      0x30   // Source of interrupts
#define ADXL345_DATA_FORMAT     0x31   // Data format control
#define ADXL345_DATAX0          0x32   // X-axis data 0
#define ADXL345_DATAX1          0x33   // X-axis data 1
#define ADXL345_DATAY0          0x34   // Y-axis data 0
#define ADXL345_DATAY1          0x35   // Y-axis data 1
#define ADXL345_DATAZ0          0x36   // Z-axis data 0
#define ADXL345_DATAZ1          0x37   // Z-axis data 1
#define ADXL345_FIFO_CTL        0x38   // FIFO control
#define ADXL345_FIFO_STATUS     0x39   // FIFO status
#define ADXL345_ADDRESS         0x53   // Device address when ADO = 0 

#define WHO_AM_I_L3G4200D       0x0F  // Should return 0xD3
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_REFERENCE      0x25
#define L3G4200D_OUT_TEMP       0x26
#define L3G4200D_STATUS_REG     0x27
#define L3G4200D_OUT_X_L        0x28
#define L3G4200D_OUT_X_H        0x29
#define L3G4200D_OUT_Y_L        0x2A
#define L3G4200D_OUT_Y_H        0x2B
#define L3G4200D_OUT_Z_L        0x2C
#define L3G4200D_OUT_Z_H        0x2D
#define L3G4200D_FIFO_CTRL_REG  0x2E
#define L3G4200D_FIFO_SRC_REG   0x2F
#define L3G4200D_INT1_CFG       0x30
#define L3G4200D_INT1_SRC       0x31
#define L3G4200D_INT1_TSH_XH    0x32
#define L3G4200D_INT1_TSH_XL    0x33
#define L3G4200D_INT1_TSH_YH    0x34
#define L3G4200D_INT1_TSH_YL    0x35
#define L3G4200D_INT1_TSH_ZH    0x36
#define L3G4200D_INT1_TSH_ZL    0x37
#define L3G4200D_INT1_DURATION  0x38
#define L3G4200D_ADDRESS        0x69  // Device address when ADO = 0

#define HMC5883L_ADDRESS      0x1E
#define HMC5883L_CONFIG_A     0x00
#define HMC5883L_CONFIG_B     0x01
#define HMC5883L_MODE         0x02
#define HMC5883L_OUT_X_H      0x03
#define HMC5883L_OUT_X_L      0x04
#define HMC5883L_OUT_Z_H      0x05
#define HMC5883L_OUT_Z_L      0x06
#define HMC5883L_OUT_Y_H      0x07
#define HMC5883L_OUT_Y_L      0x08
#define HMC5883L_STATUS       0x09
#define HMC5883L_IDA          0x0A  // should return 0x48
#define HMC5883L_IDB          0x0B  // should return 0x34
#define HMC5883L_IDC          0x0C  // should return 0x33

#define BMP085_ADDRESS        0x77  // I2C address of BMP085

#define HT16K33_ADDRESS         0x70
#define HT16K33_ON              0x21  // Commands
#define HT16K33_STANDBY         0x20
#define HT16K33_DISPLAYON       0x81
#define HT16K33_DISPLAYOFF      0x80
#define HT16K33_BLINKON         0x85 // Blink is off (00), 2 Hz (01), 1 Hz (10), or 0.5 Hz (11) for bits (21) 
#define HT16K33_BLINKOFF        0x81
#define HT16K33_DIM             0xE0

// Arrangement for display 1 (4 digit bubble display)
// 
//               a = A0
//             _________
//            |         |
//   f = A2   |  g = A4 | b = A1
//            |_________|
//            |         |
//   e = A5   |         | c = A6
//            |_________|
//               d = A3        DP = A7


static const byte numberTable[] =
{
  0x6F, // 0 = 0
  0x42, // 1 = 1, etc
  0x3B, // 2
  0x5B, // 3
  0x56, // 4
  0x5D, // 5
  0x7D, // 6
  0x43, // 7
  0x7F, // 8
  0x57, // 9
  0x80, // decimal point
  0x00, // blank
  0x10, // minus sign
};

#define display1 1
#define display2 2

#define AHRS  true          // set to false for basic data read
#define SerialDebug false  // set to true to print serial output for debugging

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

// Set accelerometer ODR and Bandwidth
enum Arate {
  ARTBW_010_005 = 0, // 0.1 Hz ODR, 0.05Hz bandwidth
  ARTBW_020_010,
  ARTBW_039_020,
  ARTBW_078_039,
  ARTBW_156_078,
  ARTBW_313_156,
  ARTBW_125_625,
  ARTBW_25_125,
  ARTBW_50_25,
  ARTBW_100_50,
  ARTBW_200_100,
  ARTBW_400_200,
  ARTBW_800_400,
  ARTBW_1600_800,
  ARTBW_3200_1600  // 3200 Hz ODR, 1600 Hz bandwidth
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Grate { // set gyro ODR and Bandwidth with 4 bits
  GRTBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
  GRTBW_100_25,
  GRTBW_100_25a,
  GRTBW_100_25b,
  GRTBW_200_125,
  GRTBW_200_25,
  GRTBW_200_50,
  GRTBW_200_70,
  GRTBW_400_20,
  GRTBW_400_25,
  GRTBW_400_50,
  GRTBW_400_110,
  GRTBW_800_30,
  GRTBW_800_35,
  GRTBW_800_50,
  GRTBW_800_110  // 800 Hz ODR, 110 Hz bandwidth   
};

enum Mrate { // set magnetometer ODR
  MRT_0075 = 0, // 0.75 Hz ODR
  MRT_015,      // 1.5 Hz
  MRT_030,      // 3.0 Hz
  MRT_075,      // 7.5 Hz
  MRT_15,       // 15 Hz
  MRT_30,       // 30 Hz
  MRT_75,       // 75 Hz ODR    
};

enum OSS {  // BMP-085 sampling rate
  OSS_0 = 0,  // 4.5 ms conversion time
  OSS_1,      // 7.5
  OSS_2,      // 13.5
  OSS_3       // 25.5
};
// Specify sensor parameters
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Arate = ARTBW_200_100; // 200 Hz ODR, 100 Hz bandwidth
uint8_t Grate = GRTBW_200_50;  // 200 Hz ODR,  50 Hz bandwidth
uint8_t Mrate = MRT_75;        //  75 Hz ODR 
uint8_t OSS = OSS_3;           // maximum pressure resolution
float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

// These are constants used to calulate the temperature and pressure from the BMP-085 sensor
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md, b5;  
uint16_t ac4, ac5, ac6;
float temperature, pressure;

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int Backlight = 58;

int16_t accelCount[3];  // 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // 16-bit signed gyro sensor output
int16_t magCount[3];    // 16-bit signed magnetometer sensor output
float   magbias[3];     // User-specified magnetometer corrections values

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate
boolean toggle = false;

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void setup()
{
  Wire.begin();
  Serial.begin(38400);
 
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  
  initHT16K33();          // initialize bubble display
  writeCommand(HT16K33_ADDRESS, HT16K33_BLINKON);  // Turn on blink
  clearDsplay(display1);  // clear bubble display1
  clearDsplay(display2);  // clear bubble display2
  
  display.begin(); // Initialize the display
  display.setContrast(55); // Set the contrast
  display.setRotation(0); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,0); display.print("GY-80");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("10 DOF 10-bit");
  display.setCursor(0, 30); display.print("sensor fusion");
  display.setCursor(20,40); display.print("AHRS");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen


  // Read the WHO_AM_I register of the ADXL345, this is a good test of communication
  uint8_t c = readByte(ADXL345_ADDRESS, WHO_AM_I_ADXL345);  // Read WHO_AM_I register for ADXL345
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(20,0); display.print("ADXL345");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xE5, HEX); 
  display.display();
  delay(1000); 
  
 // Read the WHO_AM_I register of the L3G4200D, this is a good test of communication
  uint8_t d = readByte(L3G4200D_ADDRESS, WHO_AM_I_L3G4200D);  // Read WHO_AM_I register for L3G4200D
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(20,0); display.print("L3G4200D");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(d, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xD3, HEX); 
  display.display();
  delay(1000); 

 // Read the WHO_AM_I register of the HMC5883L, this is a good test of communication
  uint8_t e = readByte(HMC5883L_ADDRESS, HMC5883L_IDA);  // Read WHO_AM_I register A for HMC5883L
  uint8_t f = readByte(HMC5883L_ADDRESS, HMC5883L_IDB);  // Read WHO_AM_I register B for HMC5883L
  uint8_t g = readByte(HMC5883L_ADDRESS, HMC5883L_IDC);  // Read WHO_AM_I register C for HMC5883L
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(20,0);  display.print("HMC5883L");
  display.setCursor(0,10);  display.print("I AM");
  display.setCursor( 0,20); display.print(e, HEX);  
  display.setCursor(20,20); display.print(f, HEX); 
  display.setCursor(40,20); display.print(g, HEX);
  display.setCursor(0,30);  display.print("I Should Be");
  display.setCursor(0,40);  display.print(0x48, HEX);
  display.setCursor(20,40); display.print(0x34, HEX); 
  display.setCursor(40,40); display.print(0x33, HEX);  
  display.display();
  delay(1000); 
  
  // Read the first three calibration register of the BMP-085, this is a good test of communication
  uint8_t h = readByte(BMP085_ADDRESS, 0xAA);  // Read WHO_AM_I register A for HMC5883L
  uint8_t i = readByte(BMP085_ADDRESS, 0xAB);  // Read WHO_AM_I register B for HMC5883L
  uint8_t j = readByte(BMP085_ADDRESS, 0xAC);  // Read WHO_AM_I register C for HMC5883L
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(20,0);  display.print("BMP085");
  display.setCursor(0,10);  display.print("I READ");
  display.setCursor( 0,20); display.print(h, HEX);  
  display.setCursor(20,20); display.print(i, HEX); 
  display.setCursor(40,20); display.print(j, HEX);
  display.setCursor(0,30);  display.print("Should Not Be");
  display.setCursor(0,40);  display.print(0x00, HEX);
  display.setCursor(20,40); display.print(" or "); 
  display.setCursor(46,40); display.print(0xFF, HEX);  
  display.display();
  delay(1000); 

  if (c == 0xE5 && d == 0xD3 && e == 0x48 && f == 0x34 && g == 0x33 && h != 0xFF && h != 0x00) // WHO_AM_I must match to proceed
  {  
    Serial.println("ADXL345  is online...");
    Serial.println("L3G4200D is online...");
    Serial.println("HMC5883L is online...");
    Serial.println("BMP085   is online...");
  
 // Initialize devices for active mode read of acclerometer, gyroscope, magnetometer, pressure and temperature
  bmp085Calibration();
  Serial.println("BMP-085 calibration completed....");
  
  calADXL345(); // Calibrate ADXL345 accelerometers, load biases in bias registers  
  initADXL345(); // Initialize and configure accelerometer 
  Serial.println("ADXL345 initialized for active data mode....");

  initL3G4200D(); // Initialize and configure gyroscope
  Serial.println("L3G4200D initialized for active data mode....");  

  if(selfTestHMC5883L()) {   // perform magnetometer self test
    Serial.print(" HMC5883L passed self test!");
    display.clearDisplay();
    display.setCursor(0, 20); display.print("HMC5883L pass");
    display.display();
    delay(1000);
  }
  else {
    Serial.print(" HMC5883L failed self test!");
    display.clearDisplay();
    display.setCursor(0, 20); display.print("HMC5883L fail");
    display.display(); 
    delay(1000);
  }
     
  initHMC5883L(); // Initialize and configure magnetometer
  Serial.println("HMC5883L initialized for active data mode....");  
  }
  else
  {
  display.clearDisplay();
  display.setCursor(0, 20); display.print("No Connection"); 
  display.setCursor(20, 40); display.print("to GY-80!");
  display.display();
  while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
    // If intPin goes high or data ready status is TRUE, all data registers have new data
    if (readByte(ADXL345_ADDRESS, ADXL345_INT_SOURCE) & 0x80) {  // When data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;   
    az = (float)accelCount[2]*aRes;  
    }
    
    if(readByte(L3G4200D_ADDRESS, L3G4200D_STATUS_REG) & 0x08) {
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;   
    gz = (float)gyroCount[2]*gRes;   
    }

    if(readByte(HMC5883L_ADDRESS, HMC5883L_STATUS) & 0x01) { // If data ready bit set, then read magnetometer data
    readMagData(magCount);  // Read the x/y/z adc values
    mRes = 0.73; // Conversion to milliGauss, 0.73 mG/LSB in hihgest resolution mode
    // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
    // like the gyro and accelerometer biases
    magbias[0] =  -30.;  // User environmental x-axis correction in milliGauss
    magbias[1] =  +85.;  // User environmental y-axis correction in milliGauss
    magbias[2] =  -78.;  // User environmental z-axis correction in milliGauss
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magbias[1];  
    mz = (float)magCount[2]*mRes - magbias[2];  
    }
 
 //   if(!AHRS) {

 //   }
  

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // In the Gy-80, all three orientation sensors' x-, y-, and z-axes are aligned;
  // the magnetometer z-axis (+ up) is parallel to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the GY-80, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. We negate the z-axis magnetic field to conform to AHRS convention of magnetic z-axis down.
  // This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
   MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my,  mz);
 //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);


    if (!AHRS) {
    int delt_t = millis() - count;
    if(delt_t > 500) {

      if(SerialDebug) {
    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg "); 
    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg "); 
    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg"); 
 
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 
    
    // Print mag values in degree/sec
    Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
    Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
    Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
 
   // Print temperature in degrees Centigrade      
    Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C");  
    Serial.print("Pressure is ");  Serial.print(pressure, 1);  Serial.println(" Pa");  
    Serial.println("");
    }
      
    display.clearDisplay();
     
    display.setCursor(0, 0); display.print(" x   y   zGY80");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    
        
    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");   
   
    display.setCursor(0,  32); display.print("T "); 
    display.setCursor(10, 32); display.print(temperature, 1); display.print(" C");
    display.setCursor(0,  40); display.print("P "); 
    display.setCursor(10, 40); display.print(pressure, 1); display.print(" Pa");
    display.display();
    
    count = millis();
    }
    }
    else {
      
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    }               
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;

    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");
    }
    
    display.clearDisplay();
      
    display.setCursor(0, 0); display.print(" x   y   z  ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
  
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    // Display 0.5-second average filter rate   
    display.setCursor(0, 40); display.print("rt: "); display.print(1.0f/deltat, 2); display.print(" Hz"); 
    display.display(); 
    
    temperature = (float)bmp085GetTemperature()/10.;  // Get temperature from BMP-085 in degrees C
    pressure = (float)bmp085GetPressure();            // Get pressure from BMP-085 in Pa
    temperature = temperature*9./5. + 32.;            // convert to Fahrenheit

    writeFloat(display1, temperature, 1);             // display temperature and pressure to bubble display
    writeFloat(display2, pressure/1000, 2);
 

    count = millis();  
    }
    
}

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    case AFS_2G:
          aRes = 2.0/(512.*64.);   // 10-bit 2s-complement
          break;
    case AFS_4G:
          aRes = 4.0/(1024.*32.);  // 11-bit 2s-complement
          break;
    case AFS_8G:
          aRes = 8.0/(2048.*16.);  // 12-bit 2s-complement
          break;
    case AFS_16G:
          aRes = 16.0/(4096.*8.);  // 13-bit 2s-complement
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(ADXL345_ADDRESS, ADXL345_DATAX0, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(L3G4200D_ADDRESS, L3G4200D_OUT_X_L | 0x80, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[1];  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}
 
void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[4] << 8) | rawData[5];  
  destination[2] = ((int16_t)rawData[2] << 8) | rawData[3]; 
}

void readTempData(byte destination)
{
  destination = readByte(L3G4200D_ADDRESS, L3G4200D_OUT_TEMP);  // Read the one raw data register 
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }
        
        
// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
// These BMP-085 functions were adapted from Jim Lindblom of SparkFun Electronics
void bmp085Calibration()
{
  ac1 = readByte(BMP085_ADDRESS, 0xAA) << 8 | readByte(BMP085_ADDRESS, 0xAB);
  ac2 = readByte(BMP085_ADDRESS, 0xAC) << 8 | readByte(BMP085_ADDRESS, 0xAD);
  ac3 = readByte(BMP085_ADDRESS, 0xAE) << 8 | readByte(BMP085_ADDRESS, 0xAF);
  ac4 = readByte(BMP085_ADDRESS, 0xB0) << 8 | readByte(BMP085_ADDRESS, 0xB1);
  ac5 = readByte(BMP085_ADDRESS, 0xB2) << 8 | readByte(BMP085_ADDRESS, 0xB3);
  ac6 = readByte(BMP085_ADDRESS, 0xB4) << 8 | readByte(BMP085_ADDRESS, 0xB5);
  b1  = readByte(BMP085_ADDRESS, 0xB6) << 8 | readByte(BMP085_ADDRESS, 0xB7);
  b2  = readByte(BMP085_ADDRESS, 0xB8) << 8 | readByte(BMP085_ADDRESS, 0xB9);
  mb  = readByte(BMP085_ADDRESS, 0xBA) << 8 | readByte(BMP085_ADDRESS, 0xBB);
  mc  = readByte(BMP085_ADDRESS, 0xBC) << 8 | readByte(BMP085_ADDRESS, 0xBD);
  md  = readByte(BMP085_ADDRESS, 0xBE) << 8 | readByte(BMP085_ADDRESS, 0xBF);
}

  // Temperature returned will be in units of 0.1 deg C
  int16_t bmp085GetTemperature()
  {
  int16_t ut = 0;
  writeByte(BMP085_ADDRESS, 0xF4, 0x2E); // start temperature measurement
  delay(5);
  uint8_t rawData[2] = {0, 0};
  readBytes(BMP085_ADDRESS, 0xF6, 2, &rawData[0]); // read raw temperature measurement
  ut = (((int16_t) rawData[0] << 8) | rawData[1]);
 
 long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return  ((b5 + 8)>>4);  
}

// Calculate pressure read calibration values  
// b5 is also required so bmp085GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure()
{
  long up = 0;
  writeByte(BMP085_ADDRESS, 0xF4, 0x34 | OSS << 6); // Configure pressure measurement for highest resolution
  delay(5 + 8*OSS); // delay 5 ms at lowest resolution, 29 ms at highest
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(BMP085_ADDRESS, 0xF6, 3, &rawData[0]); // read raw pressure measurement of 19 bits
  up = (((long) rawData[0] << 16) | ((long)rawData[1] << 8) | rawData[2]) >> (8 - OSS);

  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

        
        
        
void initHMC5883L()
{  
 // Set magnetomer ODR; default is 15 Hz 
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, Mrate << 2);   
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x00);  // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
  writeByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80 );     // enable continuous data mode
}

byte selfTestHMC5883L()
{  
  int16_t selfTest[3] = {0, 0, 0};
  //  Perform self-test and calculate temperature compensation bias
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x71);   // set 8-average, 15 Hz default, positive self-test measurement
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0xA0);   // set gain (bits[7:5]) to 5
  writeByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80 );      // enable continuous data mode
  delay(150); // wait 150 ms

  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};                        // x/y/z gyro register data stored here
  readBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];          // Turn the MSB and LSB into a signed 16-bit value
  selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];  
  selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3]; 
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x00);   // exit self test
 
  if(selfTest[0] < 575 && selfTest[0] > 243 && selfTest[1] < 575 && selfTest[1] > 243 && selfTest[2] < 575 && selfTest[2] > 243)  
  { return true; } else { return false;}
}

void initL3G4200D()
{  
 // Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode
  writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1, Grate << 4 | 0x0F);   
  writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG3, 0x08);        // Push/pull, active high interrupt, enable data ready interrupt  
  writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4, Gscale << 4); // set gyro full scale
  writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG5, 0x00);        // Disable FIFO
}


void initADXL345()
{  
 // wake up device
  writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
  delay(100);  // Let device settle down
  writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08); // Put device in normal mode

 // Set accelerometer configuration; interrupt active high, left justify MSB
  writeByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x04 | Ascale); // Set full scale range for the accelerometer 

// Choose ODR and bandwidth
  writeByte(ADXL345_ADDRESS, ADXL345_BW_RATE, Arate); // Select normal power operation, and ODR and bandwidth
  
  writeByte(ADXL345_ADDRESS, ADXL345_INT_ENABLE, 0x80);  // Enable data ready interrupt
  writeByte(ADXL345_ADDRESS, ADXL345_INT_MAP, 0x00);     // Enable data ready interrupt on INT_1
  
  writeByte(ADXL345_ADDRESS, ADXL345_FIFO_CTL, 0x00);    // Bypass FIFO
}

void calADXL345()
{  
 uint8_t data[6] = {0, 0, 0, 0, 0, 0};
 int abias[3] = {0, 0, 0};
 int16_t accel_bias[3] = {0, 0, 0};
 int samples, ii;
 
 // wake up device
  writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x00); // Put device in standby mode and clear sleep bit 2
  delay(10);  // Let device settle down
  writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0x08); // Put device in normal mode
  delay(10);
  
// Set accelerometer configuration; interrupt active high, left justify MSB
  writeByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x04 | 0x00); // Set full scale range to 2g for the bias calculation 
  uint16_t  accelsensitivity = 256;  // = 256 LSB/g at 2g full scale
  
// Choose ODR and bandwidth
  writeByte(ADXL345_ADDRESS, ADXL345_BW_RATE, 0x09); // Select normal power operation, and 100 Hz ODR and 50 Hz bandwidth
  delay(10);
  
  writeByte(ADXL345_ADDRESS, ADXL345_FIFO_CTL, 0x40 | 0x2F);    // Enable FIFO stream mode | collect 32 FIFO samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = readByte(ADXL345_ADDRESS, ADXL345_FIFO_STATUS);
  for(ii = 0; ii < samples ; ii++) {
    readBytes(ADXL345_ADDRESS, ADXL345_DATAX0, 6, &data[0]);
    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]) >> 6;
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]) >> 6;
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) >> 6;
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  // Remove gravity from z-axis accelerometer bias value
  if(accel_bias[2] > 0) {
    accel_bias[2] -= accelsensitivity; 
  }
  else { 
    accel_bias[2] += accelsensitivity; 
  }
  
  abias[0] = (int)((-accel_bias[0]/4) & 0xFF); // offset register are 8 bit 2s-complement, so have sensitivity 1/4 of 2g full scale
  abias[1] = (int)((-accel_bias[1]/4) & 0xFF);
  abias[2] = (int)((-accel_bias[2]/4) & 0xFF);
  
  writeByte(ADXL345_ADDRESS, ADXL345_OFSX, abias[0]);
  writeByte(ADXL345_ADDRESS, ADXL345_OFSY, abias[1]);
  writeByte(ADXL345_ADDRESS, ADXL345_OFSZ, abias[2]);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++ Useful Display Functions++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void writeInteger(int dsply, int data)
{
  char string[10] = "";                          // define character array to hold the digits
  itoa(data, string);                            // get ascii character string representation of the integer to be displayed
  uint8_t length = strlen(string);               // get the length of the string; number of digits in integer
  uint8_t blanks = 4 - length;                   // how many blanks do we have?

  if (length > 4) return;                        // if length greater than 4 digits we can't display on a four-digit display!

  for (uint8_t digit = 0; digit < 4; digit++)    // scroll through each digit to determine what to write to the display
  {
      char ch = string[digit];                   // get the ascii character of the next string segment

      for (int j = 0; j < blanks; j++) {
        writeDigit(dsply, j + 1, 11, 0);            // clear digit wherever there are blanks
      }

      if (ch == '-') {
      writeDigit(dsply, digit + 1 + blanks, 12, 0); // check if negative sign needed
      } 
      else {                                     // character must be a digit
      ch -= '0';                                 // convert it to an integer
      writeDigit(dsply, digit + 1 + blanks, ch, 0); // write it to the display; right justify the integer
      } 
  }
}

void writeFloat(int dsply, float data, int dp)
{
  char string[10] = "";  // define character array to hold the digits
  int dpindex = 0;       // define decimal point index
  int datanew = 0;
  
  switch (dp)
  {
    case 0:
    datanew = (int )(1.*data);
    break;
 
    case 1:
    datanew = (int )(10.*data);
    break;

    case 2:
    datanew = (int )(100.*data);
    break;
 
    case 3:
    datanew = (int )(1000.*data);
    break;
   }
   
  
  itoa(datanew, string);                         // get ascii character string representation of the integer to be displayed
  uint8_t length = strlen(string);               // get the length of the string; number of digits in integer
  uint8_t blanks = 4 - length;                   // how many blanks do we have?

  if (length > 4) return;                        // if length greater than 4 digits we can't display on a four-digit display!

// scroll through each digit to determine what to write to the display
for (uint8_t digit = 0; digit < blanks; digit++)      // first the blanks
  {
          if( (digit + 1) == (4 - dp) ) {             // handle special case where blank coincides with decimal point
            writeDigit(dsply, digit + 1, 0, 0x80);    // add leading zero before decimal place
          }
          else {
            writeDigit(dsply, digit + 1, 11, 0x00);   // otherwise clear digit wherever there are blanks
          }
  }

  for (uint8_t digit = 0; digit < 4; digit++)         // now the characters to determine what to write to the display
  {
      char ch = string[digit];                        // get the ascii character of the next string segment

      if (ch == '-') {
        if((digit + 1 + blanks) == (4 - dp) ) {
          writeDigit(dsply, digit + 1 + blanks,  0, 0x80);  // check if negative sign needed, add a decimal point
          writeDigit(dsply, digit + 0 + blanks, 12, 0x00);  // add a leading zero
        }
        else {
          writeDigit(dsply, digit + 1 + blanks, 12, 0x00);  // check if negative sign needed, no decimal point
        }
        }
      else  {                                               // character must be a digit
        ch -= '0';                                          // convert it to an integer
        if((digit + 1 + blanks) == (4 - dp) ) {
          writeDigit(dsply, digit + 1 + blanks, ch, 0x80);  // write it to the display with decimal point; right justify the integer
        } 
        else {
          writeDigit(dsply, digit + 1 + blanks, ch, 0x00);  // write it to the display; right justify the integer
        } 
     }
  }
}

void writeDigit(int dsply, byte digit, byte data, byte dp) 
{
if(dsply == 1) {
digit = (digit - 1)*2 ; 
} 
if(dsply == 2) {
  digit = (digit - 1)*2 + 9 ;
}
if(dsply == 3) {
  digit = (digit - 1)*2 + 8;
}
writeByte(HT16K33_ADDRESS, digit, numberTable[data] | dp);
}


void clearDsplay(int dsply) 
{
  for(int i = 0; i < 9; i++)  {
  writeDigit(dsply, i, 11, 0);  // Clear display, 11 is blank in the numberTable above
  }
}


void initHT16K33()
{
  writeCommand(HT16K33_ADDRESS, HT16K33_ON);         // Turn on system oscillator
  writeCommand(HT16K33_ADDRESS, HT16K33_DISPLAYON);  // Display on
  writeCommand(HT16K33_ADDRESS, HT16K33_DIM + 5);    // Maximum brightness

}


void blinkHT16K33(int time) 
{
  writeCommand(HT16K33_ADDRESS, HT16K33_BLINKON);  // Turn on blink
  delay(1000*time);
  writeCommand(HT16K33_ADDRESS, HT16K33_BLINKOFF);  // Turn on blink
}


 /* itoa:  convert n to characters in s */
 void itoa(int n, char s[])
 {
     int i, sign;
 
     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 } 
 
 /* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }

  // Wire.h read and write protocols
  void writeCommand(uint8_t address, uint8_t command)
  {
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(command);              // Put command in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

  // Wire.h read and write protocols
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
