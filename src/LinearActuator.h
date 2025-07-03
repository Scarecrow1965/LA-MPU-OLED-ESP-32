// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: MPUData.h
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

#ifndef LINEARACTUATOR_H
#define LINEARACTUATOR_H

#include <Arduino.h>
#include <Wire.h>
// #include "MPU6050.h"

// from main.cpp variables
// extern int16_t ax, ay, az, gx, gy, gz;
// extern int16_t temp;
// extern double tempCelsius;

// #include "calibrateData.h"
#include "filterData.h"
FilterData filteredData;
// function to display the filtered gyro information on the OLED screen
// filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
// filteredData.filterData(); // this function is used to filter the gyro data

// ===================================================
// Setup for relay Module and linear actuator commands
// ===================================================
// Pin assignments for relays controlling linear actuators
// Legend: LA = Linear Actuator, LHP = Left Actuator Positive, LHM = Left Actuator Negative, RHP = Right Actuator Positive, RHM = Right Actuator Negative
const int relay1_LA1LHP_Pin = 27; // Relay controlling actuator #1 positive // it is actually pin 6 on the DOIT ESP32 Dev Kit V1 or GPIO27
const int relay2_LA1LHM_Pin = 26; // Relay controlling actuator #1 negative // it is actually pin 7 on the DOIT ESP32 Dev Kit V1 or GPIO26
const int relay3_LA2RHP_Pin = 25; // Relay controlling actuator #2 positive // it is actually pin 8 on the DOIT ESP32 Dev Kit V1 or GPIO25
const int relay4_LA2RHM_Pin = 33; // Relay controlling actuator #2 negative // it is actually pin 9 on the DOIT ESP32 Dev Kit V1 or GPIO33

// Variables to control linear actuator movement
bool isMovingUp = false;
bool isMovingDown = false;
bool isStationary = false;
bool isTableLevel = false;
// Variables to indicate if the table is level
double x_angle_ok = 2.0;      // should equal to 2 degrees
double x_angle_perfect = 0.0; // should equal to 0 degrees
// Table command variables
static char lastCommand = 0;
bool isCommandBeingProcessed;

// Threshold values for detecting motion
// const int threshold = 5; // Adjust this value according to your needs
// const int threshold = 3; // Adjust this value according to your needs

// to ensure the linear actuators are moving
const int BASE_SPEED = 100;          // Base speed for linear actuator movement = 100%
const int MAX_SPEED_ADJUSTMENT = 15; // Maximum speed adjustment for linear actuator movement = 15%

// Timer variables
// unsigned long lastTime = 0;
// unsigned long lastTimeTemperature = 0;
// unsigned long lastTimeAcc = 0;
// unsigned long gyroDelay = 10;
// unsigned long temperatureDelay = 1000;
// unsigned long accelerometerDelay = 200;

// unsigned long oledPreviousMillis = 0;
unsigned long previousMillis = 0;
// unsigned long tablePreviousMillis = 0;
unsigned long currentMillis = 0;

// delay equivalence
// const unsigned int timePeriod005 = 5;    // delay equal to 5 ms
// const unsigned int timePeriod010 = 10;   // Delay equal to 10 ms
const unsigned int timePeriod100 = 100; // Delay equal to 100 ms
// const unsigned int TimePeriod050 = 500;  // Delay equal to 1/2 second
// const unsigned int timePeriod1 = 1000;   // Delay equal to 1 sec pause
// const unsigned int timePeriod2 = 2000;   // Delay equal to 2 second pause
// const unsigned int timePeriod10 = 10000; // Delay equal to 10 seconds pause
// const unsigned int timePeriod15 = 15000; // Delay equal to 15 seconds duration
// const unsigned int timePeriod30 = 30000; // Delay equal to 30 second duration

// float gyroThresholdX, gyroThresholdY, gyroThresholdZ;                             // to store the gyro threshold, not set yet
// float accelThresholdX, accelThresholdY, accelThresholdZ;                          // to store the accelerometer threshold, not set yet

// these variables are used to store the position of the MPU-6050
// Variables to store current and previous position
int currentPosition = 0;
int previousPosition = 0;
int bottomOutPosition = 0;
// Determine the position based on accelerometer data
int newPosition = currentPosition;
double velX = 0, velY = 0, velZ = 0; // velocity
double posX = 0, posY = 0, posZ = 0; // position
// wouldn't posX = 0 be the bottom out position?
float elapsedTime, currentTime, previousTime;
volatile bool getAccelerationDataTaskRunning = false;

// this is necessary for the proper calculations of the Kalmanfilter to work within the getAccelerationData function.
float calculateXAngle(int16_t ax, int16_t az)
{
    return atan2(ax, az);
};

float calculateYAngle(int16_t ay, int16_t az)
{
    return atan2(ay, az);
};

float calculateZAngle(int16_t az, int16_t ay)
{
    return atan2(az, ay);
};

// The following code is for the linear actuator movement
void getAccelerationData(void)
{
    filteredData.filterData();
    double currentTime = millis() / 1000.0;        // get current time in seconds
    double deltaTime = currentTime - previousTime; // calculate time difference

    // Apply the Kalman filter to the accelerometer data
    // Apply the Kalman filters to the accelerometer data
    double axFiltered = filteredData.kalmanloop(ax, gx, calculateXAngle);
    double ayFiltered = filteredData.kalmanloop(ay, gy, calculateYAngle);
    double azFiltered = filteredData.kalmanloop(az, gz, calculateZAngle);

    // integrate acceleration to get velocity
    Serial.println("getAccelerationData");
    Serial.print("Accel X: ");    // used for testing purposes only
    Serial.print(ax);             // used for testing purposes only
    Serial.print("\t Accel Y: "); // used for testing purposes only
    Serial.print(ay);             // used for testing purposes only
    Serial.print("\t Accel Z: "); // used for testing purposes only
    Serial.print(az);             // used for testing purposes only
    Serial.print("Gyro x: ");     // used for testing purposes only
    Serial.print(gx);             // used for testing purposes only
    Serial.print("\t Gyro y: ");  // used for testing purposes only
    Serial.print(gy);             // used for testing purposes only
    Serial.print("\t Gyro z: ");  // used for testing purposes only
    Serial.println(gz);           // used for testing purposes only
    // Integrate the filtered acceleration to get velocity
    velX += axFiltered * deltaTime;
    velY += ayFiltered * deltaTime;
    velZ += azFiltered * deltaTime;
    // unfiltered acceleration
    // velX += ax * deltaTime;
    // velY += ay * deltaTime;
    // velZ += az * deltaTime;
    Serial.print("deltaTime: ");     // used for testing purposes only
    Serial.print(deltaTime);         // used for testing purposes only
    Serial.print("\t Velocity X: "); // used for testing purposes only
    Serial.print(velX);              // used for testing purposes only
    Serial.print("\t Velocity Y: "); // used for testing purposes only
    Serial.print(velY);              // used for testing purposes only
    Serial.print("\t Velocity Z: "); // used for testing purposes only
    Serial.println(velZ);            // used for testing purposes only

    // integrate velocity to get position
    posX += velX * deltaTime;
    posY += velY * deltaTime;
    posZ += velZ * deltaTime;
    Serial.print("Position X: "); // used for testing purposes only
    Serial.print(posX);              // used for testing purposes only
    Serial.print("\t Position Y: "); // used for testing purposes only
    Serial.print(posY);              // used for testing purposes only
    Serial.print("\t Position Z: "); // used for testing purposes only
    Serial.println(posZ);            // used for testing purposes only

    previousTime = currentTime; // update previous time
};

void getAccelerationDataTask(void *parameter)
{
    while (getAccelerationDataTaskRunning)
    {
        getAccelerationData();
        delay(100); // Delay to prevent overloading the processor
    }

    vTaskDelete(NULL); // Delete this task if the loop is exited
}

bool dataChanged = false;
int16_t previousGyroX = 0, previousGyroY = 0, previousAccX = 0, previousAccY = 0; // to help with providing an interrupt
// ensure these filetered values are identical to those in the filterData.h file
uint8_t x_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
uint8_t y_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
uint8_t z_accel_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
uint8_t x_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
uint8_t y_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
uint8_t z_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// //  to check to see if there is a change in the table position
void checkMPUData(bool &dataChanged)
{
    filteredData.filterData(); // reads the filtered MPU-6050 // Read gyro and accelerometer data

    // Check if gyro or accelerometer data has changed beyond the thresholds
    if (abs(gx - previousGyroX) > x_gyro_filter ||
        abs(gy - previousGyroY) > y_gyro_filter ||
        abs(ax - previousAccX) > x_accel_filter ||
        abs(ay - previousAccY) > y_accel_filter)
    {
        // Data has changed
        previousGyroX = gx;
        previousGyroY = gy;
        previousAccX = ax;
        previousAccY = ay;
        dataChanged = true;
        Serial.println("Data has changed"); // used for testing purposes only
    }
    else
    {
        // Data has not changed
        dataChanged = false;
        // Serial.println("Data has NOT Changed"); // used for testing purposes only
    }
}; // end boolean to check if gyro or accel from mpu has changed position

// ===================================================
// FUNCTIONS FOR MOVEMENT OF THE TABLE
// ===================================================

void moveActuators(bool leftUp, bool rightUp)
{
    if (leftUp)
    { // LA1LH -> if true then, move up
        digitalWrite(relay1_LA1LHP_Pin, LOW);
        digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
    }
    else
    {                                          // LA1LH -> if false then, move down
        digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
        digitalWrite(relay2_LA1LHM_Pin, LOW);
    }

    if (rightUp)
    { // LA2RH -> if true then, move up
        digitalWrite(relay3_LA2RHP_Pin, LOW);
        digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection
    }
    else
    {                                          // LA2RH -> if false then, move down
        digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
        digitalWrite(relay4_LA2RHM_Pin, LOW);
    }
};

void moveUp()
{
    moveActuators(true, true);
    isMovingDown = true;
    isMovingUp = false;
    isStationary = false;
    // Activate the relay controlling actuator #1 positive
    // digitalWrite(relay1_LA1LHP_Pin, LOW);
    // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
    // Activate the relay controlling actuator #1 negative
    // digitalWrite(relay3_LA2RHP_Pin, LOW);
    // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection

    // Start the getAccelerationDataTask
    getAccelerationDataTaskRunning = true;
    xTaskCreate(getAccelerationDataTask, "GetAccelerationData", 10000, NULL, 1, NULL);
};
void moveDown()
{
    moveActuators(false, false);
    isMovingUp = true;
    isMovingDown = false;
    isStationary = false;
    // Activate the relay controlling actuator #2 positive
    // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
    // digitalWrite(relay2_LA1LHM_Pin, LOW);
    // Activate the relay controlling actuator #2 negative
    // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
    // digitalWrite(relay4_LA2RHM_Pin, LOW);
    // Start the getAccelerationDataTask
    getAccelerationDataTaskRunning = true;
    xTaskCreate(getAccelerationDataTask, "GetAccelerationData", 10000, NULL, 1, NULL);
};

void stopMovement()
{
    digitalWrite(relay1_LA1LHP_Pin, HIGH);
    digitalWrite(relay2_LA1LHM_Pin, HIGH);
    digitalWrite(relay3_LA2RHP_Pin, HIGH);
    digitalWrite(relay4_LA2RHM_Pin, HIGH);
    isTableLevel = false; // We don't know what level the table is at after stopping movement
    isStationary = true;
    isMovingUp = false;
    isMovingDown = false;
    // Stop the getAccelerationDataTask
    getAccelerationDataTaskRunning = false;
};

void levelTable(void)
{
    filteredData.filterData();
    filteredData.complimentaryFilter();
    Serial.print("angle is calculated at: ");
    Serial.println(angle);
    // Coarse corrections
    if (abs(angle) > x_angle_ok)
    {
        Serial.println("Coarse corrections");
        if (angle > x_angle_ok)
        {
            // if LA1LH is higher then LA2RH, then we lower LA1LH
            digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
            digitalWrite(relay2_LA1LHM_Pin, LOW);
        }
        else if (angle < -x_angle_ok)
        {
            // if LA2RH is higher then LA1LH, then we lower LA2RH
            digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
            digitalWrite(relay4_LA2RHM_Pin, LOW);
        }
    }
    // Refined corrections
    else if (abs(angle) > x_angle_perfect)
    {
        Serial.println("Refined corrections");
        if (angle > x_angle_perfect)
        {
            // if LA1LH is higher then LA2RH, then we lower LA1LH
            digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
            digitalWrite(relay2_LA1LHM_Pin, LOW);
        }
        else if (angle < -x_angle_perfect)
        {
            // if LA2RH is higher then LA1LH, then we lower LA2RH
            digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
            digitalWrite(relay4_LA2RHM_Pin, LOW);
        }
    }
    else
    {
        stopMovement();
        isTableLevel = true; // Set isTableLevel to true when the table is level
        Serial.println("Table is now level");
    }
};

void checkTableAfterStop()
{
    if (isTableLevel == false)
    {
        levelTable();
    }
    else
    {
        isTableLevel = true;
        Serial.println("Table is level");
    }
};

// void processCommand(char command)
// {
//     unsigned long startTime = millis(); // Record the start time
//     unsigned long timeout = 5000;       // Set the timeout to 5000 milliseconds (5 seconds)
//     switch (command)
//     {
//     case 'Q':
//         Serial.println(F("Quitting Program"));
//         exit(0);
//         break;
//     case 'q':
//         Serial.println(F("Quitting Program"));
//         exit(0);
//         break;
//     case 'M':
//         Serial.println(F("Constant Gyro Monitoring"));
//         while (Serial.available() == 0 && millis() - startTime < timeout) // Check if there's no new command and the timeout hasn't been reached
//         {
//             filteredData.filterData();
//             // function to display the filtered gyro information on the OLED screen
//             filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
//             // displayAllGyro2();
//             delay(100);
//         }
//         break;
//     case 'm':
//         Serial.println(F("Constant Gyro Monitoring"));
//         // unsigned long startTime = millis();                               // Record the start time
//         // unsigned long timeout = 5000;                                     // Set the timeout to 5000 milliseconds (5 seconds)
//         while (Serial.available() == 0 && millis() - startTime < timeout) // Check if there's no new command and the timeout hasn't been reached
//         {
//             filteredData.filterData();
//             // function to display the filtered gyro information on the OLED screen
//             filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
//             // displayAllGyro2();
//             delay(100);
//         }
//         break;
//     case 'D':
//         Serial.println(F("Moving Down"));
//         moveDown();
//         break;
//     case 'd':
//         Serial.println(F("Moving Down"));
//         moveDown();
//         break;
//     case 'U':
//         Serial.println(F("Moving Up"));
//         moveUp();
//         break;
//     case 'u':
//         Serial.println(F("Moving Up"));
//         moveUp();
//         break;
//     case 'S':
//         Serial.println(F("Stopping Movement"));
//         stopMovement();
//         checkTableAfterStop();
//         break;
//     case 's':
//         Serial.println(F("Stopping Movement"));
//         stopMovement();
//         checkTableAfterStop();
//         break;
//     default:
//         Serial.println(F("Invalid Command"));
//         break;
//     }
// }; // end process command function

// void tableCommands(void)
// {
//     Serial.println(F("\nSend 'U'/'u' for Up, 'D'/'d' for Down, 'S'/'s' for STOP \n'Q'/'q' to Quit Program, 'M'/'m' for Constant Gyro Monitoring.\n"));
//
//     while (true)
//     {
//         currentMillis = millis(); // Get the current time
//
//         if (currentMillis - previousMillis >= timePeriod100)
//         {
//             // Save the last time the display was updated
//             previousMillis = currentMillis;
//
//             filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
//         }
//
//         // Check if a command is available from the Serial Monitor
//         if (Serial.available())
//         {
//             // bool isCommandBeingProcessed = true;
//             char command = Serial.read();
//
//             // Consume any additional characters from the Serial buffer
//             while (Serial.available())
//             {
//                 Serial.read();
//             }
//
//             // Debug output to display the received command
//             Serial.print(F("Received Command: "));
//             Serial.println(command);
//
//             // Save the command for later use
//             lastCommand = command;
//
//             // Process the command
//             isCommandBeingProcessed = true; // Set the flag to true before processing the command
//             processCommand(command);
//             isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
//         }
//         else
//         {
//             // If no new command is received, use the last command
//             if (lastCommand != 0)
//             {
//                 Serial.print(F("Repeating last command: "));
//                 Serial.println(lastCommand);
//
//                 // Process the last command
//                 isCommandBeingProcessed = true; // Set the flag to true before processing the command
//                 processCommand(lastCommand);
//                 isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
//             }
//             else
//             {
//                 Serial.println(F("No command received"));
//                 stopMovement();
//             }
//         }
//     }
// }; // end table commands function

#endif // LINEARACTUATOR_H

// =========================================================
// END OF PROGRAM
// =========================================================