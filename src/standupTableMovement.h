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
// History:     20-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef STANDUPTABLEMOVEMENT_H
#define STANDUPTABLEMOVEMENT_H

#include <Arduino.h>
#include <Wire.h>

// from main.cpp variables
// extern int16_t ax, ay, az, gx, gy, gz;
// extern int16_t temp;
// extern double tempCelsius;

// #include "calibrateData.h"
#include "filterData.h"
// FilterData filteredData;
// function to display the filtered gyro information on the OLED screen
// filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
// filteredData.filterData(); // this function is used to filter the gyro data

#include "LinearActuator.h"
extern unsigned long previousMillis;
extern unsigned long currentMillis;
extern const unsigned int timePeriod100; // Delay equal to 100 ms

void processCommand(char command)
{
    unsigned long startTime = millis(); // Record the start time
    unsigned long timeout = 5000;       // Set the timeout to 5000 milliseconds (5 seconds)
    
    switch (command)
    {
    case 'Q':
        Serial.println(F("Quitting Program"));
        exit(0);
        break;
    case 'q':
        Serial.println(F("Quitting Program"));
        exit(0);
        break;
    case 'M':
        Serial.println(F("Constant Gyro Monitoring"));
        while (Serial.available() == 0 && millis() - startTime < timeout) // Check if there's no new command and the timeout hasn't been reached
        {
            filteredData.filterData();
            // function to display the filtered gyro information on the OLED screen
            filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
            // displayAllGyro2();
            delay(100);
        }
        break;
    case 'm':
        Serial.println(F("Constant Gyro Monitoring"));
        // unsigned long startTime = millis();                               // Record the start time
        // unsigned long timeout = 5000;                                     // Set the timeout to 5000 milliseconds (5 seconds)
        while (Serial.available() == 0 && millis() - startTime < timeout) // Check if there's no new command and the timeout hasn't been reached
        {
            filteredData.filterData();
            // function to display the filtered gyro information on the OLED screen
            filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
            // displayAllGyro2();
            delay(100);
        }
        break;
    case 'D':
        Serial.println(F("Moving Down"));
        moveDown();
        break;
    case 'd':
        Serial.println(F("Moving Down"));
        moveDown();
        break;
    case 'U':
        Serial.println(F("Moving Up"));
        moveUp();
        break;
    case 'u':
        Serial.println(F("Moving Up"));
        moveUp();
        break;
    case 'S':
        Serial.println(F("Stopping Movement"));
        stopMovement();
        checkTableAfterStop();
        break;
    case 's':
        Serial.println(F("Stopping Movement"));
        stopMovement();
        checkTableAfterStop();
        break;
    default:
        Serial.println(F("Invalid Command"));
        break;
    }
}; // end process command function

void tableCommands(void)
{
    Serial.println(F("\nSend 'U'/'u' for Up, 'D'/'d' for Down, 'S'/'s' for STOP \n'Q'/'q' to Quit Program, 'M'/'m' for Constant Gyro Monitoring.\n"));

    while (true)
    {
        currentMillis = millis(); // Get the current time

        if (currentMillis - previousMillis >= timePeriod100)
        {
            // Save the last time the display was updated
            previousMillis = currentMillis;

            filteredData.displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
        }

        // Check if a command is available from the Serial Monitor
        if (Serial.available())
        {
            // bool isCommandBeingProcessed = true;
            char command = Serial.read();

            // Consume any additional characters from the Serial buffer
            while (Serial.available())
            {
                Serial.read();
            }

            // Debug output to display the received command
            Serial.print(F("Received Command: "));
            Serial.println(command);

            // Save the command for later use
            lastCommand = command;

            // Process the command
            isCommandBeingProcessed = true; // Set the flag to true before processing the command
            processCommand(command);
            isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
        }
        else
        {
            // If no new command is received, use the last command
            if (lastCommand != 0)
            {
                Serial.print(F("Repeating last command: "));
                Serial.println(lastCommand);

                // Process the last command
                isCommandBeingProcessed = true; // Set the flag to true before processing the command
                processCommand(lastCommand);
                isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
            }
            else
            {
                Serial.println(F("No command received"));
                stopMovement();
                checkTableAfterStop();
            }
        }
    }
}; // end table commands function

#endif // STANDUPTABLEMOVEMENT_H

// =========================================================
// END OF PROGRAM
// =========================================================