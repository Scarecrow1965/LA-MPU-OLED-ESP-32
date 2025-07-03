// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: main.cpp
//
// Description:
//
// Provides the computer workstation the ability to use the remote/handset,
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     7-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

// libraries for the OLED
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

// libraries for MPU-6050
#include <I2Cdev.h>
// #include <MPU6050.h>

// variables for the MPU-6050
#define INTERRUPT_PIN 4 // interrupt connected to MPU-6050 // GPIO 4 on the ESP32
const int MPU6050_addr = 0x68; // to initialize the address of the MPU-6050
static int16_t ax, ay, az, gx, gy, gz;
static int16_t temp;
static double tempCelsius;

// additional libraries for the MPU-6050
#include "dumpData.h"
#include "IMU_Zero.h"
#include "calibrateData.h"

#include "LinearActuator.h"
#include "filterData.h"
#include "standupTableMovement.h"
// FilterData filteredData;

// ================
// This will enable for the OLED screen to display information
// definition of OLED display SSD1306 for ESP32
#define OLED_CLOCK 22 // SCA pin on Display = pin 17 (I2C_SCL) on ESP32 DEVKIT V1 = GPIO 22
#define OLED_DATA 21  // SDL pin on display = pin 20 (I2C_SDA) on ESP32 DEVKIT V1 = GPIO 21
// U8G2 SSD1306 Driver here to run OLED Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, U8X8_PIN_NONE); // This works but according to the function, it shouldn't
uint8_t oled_LineH = 0;
bool bLED = false; // LED state

// ADAFRUIT SSD1306 Driver here to run animation
#define SCREEN_I2C_ADDR 0x3C // or 0x3C // also should be the same address as the MPU6050 which causes problems
// #define SCREEN_I2C_ADDR 0x68 // alternate addr for the OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RST_PIN -1  // Reset pin (-1 if not available)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);
// ================

#define USE_LIBRARY2
#ifdef USE_LIBRARY2
#include <MPU6050.h>
MPU6050 MPU2;
#endif

// ================
// CREATING A PID CONTROLLER

// Any variables for the PID controller go here!
// float targetValue = 0;

// Variables for Time Keeper function:
// #define LOOP_TIME 10 // Time in ms (10ms = 100Hz)
// unsigned long timerValue = 0;

// Declare variables
// float Kp = 7;       // (P)roportional Tuning Parameter
// float Ki = 6;       // (I)ntegral Tuning Parameter
// float Kd = 3;       // (D)erivative Tuning Parameter
// float iTerm = 0;    // Used to accumulate error (integral)
// float lastTime = 0; // Records the time the function was last called
// float maxPID = 255; // The maximum value that can be output
// float oldValue = 0; // The last sensor value

// /**
//  * PID Controller
//  * @param  (target)  The target position/value we are aiming for
//  * @param  (current) The current value, as recorded by the sensor
//  * @return The output of the controller
//  */

// /****** PID CONTROLLER *****/
// float pid(float target, float current)
// {
//   // Calculate the time since function was last called
//   float thisTime = millis();
//   float dT = thisTime - lastTime;
//   lastTime = thisTime;
// 
//   // Calculate error between target and current values
//   float error = target - current;
// 
//   // Calculate the integral term
//   iTerm += error * dT;
// 
//   // Calculate the derivative term (using the simplification)
//   float dTerm = (oldValue - current) / dT;
// 
//   // Set old variable to equal new ones
//   oldValue = current;
// 
//   // Multiply each term by its constant, and add it all up
//   float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);
// 
//   // Limit PID value to maximum values
//   if (result > maxPID)
//     result = maxPID;
//   else if (result < -maxPID)
//     result = -maxPID;
// 
//   return result;
// };
// ================

// // function to display information on the OLED screen without FPS
// void displayAllGyro2(void)
// {
//     bLED = !bLED; // toggle LED State
//     digitalWrite(LED_BUILTIN, bLED);
//
//     MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//     temp = MPU2.getTemperature();
//
//     display.clearDisplay();
//     u8g2.clearBuffer();
//     u8g2.home();
//     u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display
//     u8g2.setFont(u8g2_font_profont10_tf);
//     u8g2.setCursor(3, oled_LineH + 2);
//     u8g2.println("MPU Raw output:");
//     u8g2.setCursor(3, oled_LineH * 3 + 2);
//     u8g2.println("Accel(m/s^2)");
//     u8g2.setCursor(3, oled_LineH * 4 + 2);
//     u8g2.print("X:");
//     u8g2.print(ax);
//     u8g2.print(", Y:");
//     u8g2.print(ay);
//     u8g2.print(", Z:");
//     u8g2.print(az);
//     u8g2.setCursor(3, oled_LineH * 5 + 2);
//     u8g2.println("Gyro(rps)");
//     u8g2.setCursor(3, oled_LineH * 6 + 2);
//     u8g2.print("X:");
//     u8g2.print(gx);
//     u8g2.print(", Y:");
//     u8g2.print(gy);
//     u8g2.print(", Z:");
//     u8g2.print(gz);
//     u8g2.setCursor(3, oled_LineH * 7 + 2);
//     u8g2.print("Temp= ");
//     u8g2.print(tempCelsius);
//     u8g2.print(" ");
//     u8g2.print(char(176));
//     u8g2.setFont(u8g2_font_profont10_tf);
//     u8g2.print("C");
//     u8g2.sendBuffer(); // Send it out
// };                     // end displaying the MPU-6050 info without the fps

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
#ifdef USE_LIBRARY2
  // ================
  pinMode(LED_BUILTIN, OUTPUT); // this is the LED on the ESP32 board as output
  // ================
  // Set relay pins as outputs
  // const int relay1_LA1LHP_Pin PROGMEM = 27;
  // const int relay2_LA1LHM_Pin PROGMEM = 26;
  // const int relay3_LA2RHP_Pin PROGMEM = 25;
  // const int relay4_LA2RHM_Pin PROGMEM = 33;
  pinMode(relay1_LA1LHP_Pin, OUTPUT);
  pinMode(relay2_LA1LHM_Pin, OUTPUT);
  pinMode(relay3_LA2RHP_Pin, OUTPUT);
  pinMode(relay4_LA2RHM_Pin, OUTPUT);
  // ================
  // Set up the interrupt pin, and configure the interrupt to be triggered on a rising edge
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr, RISING);

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial)
    ;

  // ================
  Serial.println("Starting relay activation"); // used for testing purposes only
  // Set initial relay states (HIGH to deactivate all relays)
  digitalWrite(relay1_LA1LHP_Pin, HIGH);
  digitalWrite(relay2_LA1LHM_Pin, HIGH);
  digitalWrite(relay3_LA2RHP_Pin, HIGH);
  digitalWrite(relay4_LA2RHM_Pin, HIGH);
  // ================

  Wire.begin();
  Wire.setClock(400000); // Set I2C clock frequency to 400kHz

  // initialize device
  Serial.println("Initializing I2C devices...");
  MPU2.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.print("MPU Connection ");
  Serial.println(MPU2.testConnection() ? "successful" : "failed");

  // Functions to test out the MPU6050 or to initialize it
  // this function is used to initialize the MPU6050
  IMUInitialization(); // initializes the MPU6050 to normal values
  // if using IMUSetup, this takes 5 minutes
  // dumpData(); // this function is used to setup the dump data from the MPU6050
  // this is one of the functions used to calibrate the MPU6050
  calibrateData();
  // this calibration takes 1 minute

  // ================
  // setup for the OLED display
  u8g2.begin();
  u8g2.clear();
  u8g2.setFont(u8g2_font_profont10_tf);
  oled_LineH = u8g2.getFontAscent() - u8g2.getFontDescent();
  if (!u8g2.begin())
  {
    Serial.println(F("SSD1306 allocation failed! using the U8G2 library"));
    for (;;)
    {
      // don't proceed, loop forever
    }
  };

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
  {
    Serial.println(F("SSD1306 allocation failed! using the Adafruit library"));
    for (;;)
    {
      // don't proceed, loop forever
    }
  };
  display.clearDisplay();
  Serial.println("OLED Display Setup Complete");
  // ================
  
  // timerValue = millis(); // Set the timer to the current time
  // filteredData.kalmanFilter(float newAngle, float newRate, float dt); // this function is used to setup the kalman filter

  // ================
  // Time for the relay activation
  

  // ================
  // printout the commands list once
  // Serial.println(F("\nSend 'U' for Up, 'D' for Down, 'S' for STOP"));
  // this command is included in the tableMovement function in standupTableMovement.h
  // ================

  // ================
  // Initialize pins for RT-11 remote wires
  // pinMode(memoryButtonPin, INPUT_PULLUP);
  // pinMode(dataDisplayPin, INPUT_PULLUP);
  // pinMode(groundPin, INPUT_PULLUP);
  // pinMode(txPin, INPUT_PULLUP);
  // pinMode(vccPin, INPUT_PULLUP);
  // pinMode(memoryRecallPin, INPUT_PULLUP);
  // pinMode(upPin, INPUT_PULLUP);
  // pinMode(downPin, INPUT_PULLUP);
  // ================

  Serial.println("Setup complete"); // used for testing purposes only
#endif
};

// ================================================================
// ===                      CONSTANT LOOP                       ===
// ================================================================

void loop()
{
#ifdef USE_LIBRARY2
  // ================
  // following lines are used to relay the information from the MPU6050 to the OLED screen
  // read raw accel/gyro measurements from device
  // MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // temp = MPU2.getTemperature();
  // double tempCelsius = (temp / 340.00) + 36.53; // convert to Celsius using MPU6050.h library

  // display tab-separated accel/gyro x/y/z values
  // Serial.print("MPU Raw Output:\nAccX:\t");
  // Serial.print(ax);
  // Serial.print("\t AccY: ");
  // Serial.print(ay);
  // Serial.print("\t AccZ: ");
  // Serial.print(az);
  // Serial.print("\t GyroX: ");
  // Serial.print(gx);
  // Serial.print("\t GyroY: ");
  // Serial.print(gy);
  // Serial.print("\t GyroZ: ");
  // Serial.print(gz);
  // Serial.print("\t Temp: ");
  // Serial.println(tempCelsius);
  // ================

  // Functions to test out the MPU6050 or to initialize it
  // dumpDataLoop(); // this function is used to dump the data from the MPU6050
  // ensure the Serial.print lines above are commented out as well as the getMotion6 function if yo uare going to use this function

  // this works to display the raw gyro or filtered gyro information on the OLED screen
  // displayAllGyro2(); // display the raw gyro infomration on the OLED screen // this function is used to display actual gyro information on the OLED screen
  // or
  filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
  // filteredData.getThresholdResets(); // this function is used to reset the threshold values if the MPU-6050 is not properly calibrated 
  // filteredData.complimentaryFilter(); // this function is used to apply the complimentary filter to the MPU6050

  // blink LED to indicate activity
  bLED = !bLED; // toggle LED State
  digitalWrite(LED_BUILTIN, bLED);

  // Only run the controller once the time interval has passed
  // if (millis() - timerValue > LOOP_TIME)
  // {
  //   timerValue = millis();
  // 
  //   // Replace getAngle() with your sensor data reading
  //   float currentValue = getAngle();
  // 
  //   // Run the PID controller
  //   float motorOutput = pid(targetValue, currentValue);
  // 
  //   // Replace moveMotors() with your desired output
  //   moveMotors(motorOutput);
  // }

  tableCommands(); // this function is used to control the table movement
  // ================
  #endif
};

// =========================================================
// END OF PROGRAM
// =========================================================
