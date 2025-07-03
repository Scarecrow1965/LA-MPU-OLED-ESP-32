// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: calibrateData.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     6-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef CALIBRATEDATA_H
#define CALIBRATEDATA_H

#include <Arduino.h>
#include <Wire.h>

// FROM: https://wired.chillibasket.com/2015/01/calibrating-mpu6050/

// Arduino sketch that returns calibration offsets for MPU6050
//   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */

// I2Cdev and MPU6050 must be installed as libraries
#include <I2Cdev.h>

#define USE_LIBRARY2
#ifdef USE_LIBRARY2
#include <MPU6050.h>

///////////////////////////////////   CONFIGURATION   /////////////////////////////
// Change this 3 variables if you want to fine tune the sketch to your needs.
int buffersize = 1000; // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int accel_deadzone = 8; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int gyro_deadzone = 1; // Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 MPU3;
// MPU6050 MPU2;
// MPU6050 accelgyro; // original
// MPU6050 accelgyro(0x68);

// extern int16_t ax, ay, az, gx, gy, gz;
int mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;
int count = 0;
int state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
//--------------------------------------------//
// Get the mean values from the sensor
//--------------------------------------------//
void meansensors()
{
    long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: AccX: ");
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
        // Serial.print("\t buffer size :");
        // Serial.print(buffersize);
        // Serial.println();

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (count == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }
};

//--------------------------------------------//
// Calibrate sensor
//--------------------------------------------//
void calibration(void)
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    // az_offset = (16384 - mean_az) / 8; // this is based on setAccelerometerRange(MPU6050_RANGE_2_G) = 16384
    az_offset = (8192 - mean_az) / 8; // this is based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    while (1)
    {
        int ready = 0;
        MPU3.setXAccelOffset(ax_offset);
        MPU3.setYAccelOffset(ay_offset);
        MPU3.setZAccelOffset(az_offset);

        MPU3.setXGyroOffset(gx_offset);
        MPU3.setYGyroOffset(gy_offset);
        MPU3.setZGyroOffset(gz_offset);

        // used for testing purposes only
        // Serial.print("MPU Calibration output: AX_offset: ");
        // Serial.print(ax_offset);
        // Serial.print("\t AY_offset: ");
        // Serial.print(ay_offset);
        // Serial.print("\t AZ_offset: ");
        // Serial.print(az_offset);
        // Serial.print("\t GX_offset: ");
        // Serial.print(gx_offset);
        // Serial.print("\t GY_offset: ");
        // Serial.print(gy_offset);
        // Serial.print("\t GZ_offset: ");
        // Serial.print(gz_offset);
        // Serial.println();

        meansensors();

        Serial.println("...");

        if (abs(mean_ax) <= accel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / accel_deadzone;

        if (abs(mean_ay) <= accel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / accel_deadzone;
        // this is based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192
        // if setAccelerometerRange(MPU6050_RANGE_2_G), then use 16384
        if (abs(8192 - mean_az) <= accel_deadzone)
            ready++;
        else
            az_offset = az_offset + (8192 - mean_az) / accel_deadzone;

        if (abs(mean_gx) <= gyro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);

        if (abs(mean_gy) <= gyro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);

        if (abs(mean_gz) <= gyro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);

        if (ready == 6)
            break;
    }
};

void calibrateXAccel(void)
{
    MPU3.setXAccelOffset(0);
    // sensor reading
    long buff_ax = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: AccX:\t");
        // Serial.println(ax);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_ax = buff_ax + ax;
        }
        if (count == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    ax_offset = -mean_ax / 8;
    while (1)
    {
        int ready = 0;
        MPU3.setXAccelOffset(ax_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: AX_offset set at:\t");
        // Serial.println(ax_offset);

        // sensor reading
        long buff_ax = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: AccX: ");
            // Serial.println(ax);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_ax = buff_ax + ax;
            }
            if (count == (buffersize + 100))
            {
                mean_ax = buff_ax / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(mean_ax) <= accel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / accel_deadzone;

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Accel X reading with offset:\t");
    Serial.println(mean_ax);
    Serial.print("Your new Accel X offset:\t");
    Serial.println(ax_offset);
};

void calibrateYAccel(void)
{
    MPU3.setYAccelOffset(0);
    // sensor reading
    long buff_ay = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: AccY:\t");
        // Serial.println(ay);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_ay = buff_ay + ay;
        }
        if (count == (buffersize + 100))
        {
            mean_ay = buff_ay / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    ay_offset = -mean_ay / 8;
    while (1)
    {
        int ready = 0;
        MPU3.setYAccelOffset(ay_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: AY_offset set at:\t");
        // Serial.println(ay_offset);

        // sensor reading
        long buff_ay = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: AccY: ");
            // Serial.println(ay);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_ay = buff_ay + ay;
            }
            if (count == (buffersize + 100))
            {
                mean_ay = buff_ay / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(mean_ay) <= accel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / accel_deadzone;

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Accel Y reading with offset:\t");
    Serial.println(mean_ay);
    Serial.print("Your new Accel Y offset:\t");
    Serial.println(ay_offset);
};

void calibrateZAccel(void)
{
    MPU3.setZAccelOffset(0);
    // sensor reading
    long buff_az = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: AccX:\t");
        // Serial.println(ax);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_az = buff_az + az;
        }
        if (count == (buffersize + 100))
        {
            mean_az = buff_az / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    ax_offset = -mean_az / 8;
    while (1)
    {
        int ready = 0;
        MPU3.setZAccelOffset(az_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: AX_offset set at:\t");
        // Serial.println(ax_offset);

        // sensor reading
        long buff_az = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: AccX: ");
            // Serial.println(ax);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_az = buff_az + az;
            }
            if (count == (buffersize + 100))
            {
                mean_az = buff_az / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(8192 - mean_az) <= accel_deadzone)
            ready++;
        else
            az_offset = az_offset + (8192 - mean_az) / accel_deadzone;

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Accel Z reading with offset:\t");
    Serial.println(mean_az);
    Serial.print("Your new Accel Z offset:\t");
    Serial.println(az_offset);
};

void calibrateXGyro(void)
{
    MPU3.setXGyroOffset(0);
    // sensor reading
    long buff_gx = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: GyroX:\t");
        // Serial.println(gx);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_gx = buff_gx + gx;
        }
        if (count == (buffersize + 100))
        {
            mean_gx = buff_gx / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    gx_offset = -mean_gx / 4;
    while (1)
    {
        int ready = 0;
        MPU3.setXGyroOffset(gx_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: GX_offset set at:\t");
        // Serial.println(gx_offset);

        // sensor reading
        long buff_gx = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: GyroX: ");
            // Serial.println(gx);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_gx = buff_gx + gx;
            }
            if (count == (buffersize + 100))
            {
                mean_gx = buff_gx / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(mean_gx) <= accel_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Gyro X reading with offset:\t");
    Serial.println(mean_gx);
    Serial.print("Your new Gyro X offset:\t");
    Serial.println(gx_offset);
};

void calibrateYGyro(void)
{
    MPU3.setYGyroOffset(0);
    // sensor reading
    long buff_gy = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: GyroY:\t");
        // Serial.println(gy);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_gy = buff_gy + gy;
        }
        if (count == (buffersize + 100))
        {
            mean_gy = buff_gy / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    gy_offset = -mean_gy / 4;
    while (1)
    {
        int ready = 0;
        MPU3.setYGyroOffset(gy_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: GY_offset set at:\t");
        // Serial.println(gy_offset);

        // sensor reading
        long buff_gy = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: GyroY: ");
            // Serial.println(gy);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_gy = buff_gy + gy;
            }
            if (count == (buffersize + 100))
            {
                mean_gy = buff_gy / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(mean_gy) <= gyro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Gyro Y reading with offset:\t");
    Serial.println(mean_gy);
    Serial.print("Your new Gyro Y offset:\t");
    Serial.println(gy_offset);
};

void calibrateZGyro(void)
{
    MPU3.setZGyroOffset(0);
    // sensor reading
    long buff_gz = 0;
    int count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: GyroZ:\t");
        // Serial.println(gz);

        if (count > 100 && count <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_gz = buff_gz + gz;
        }
        if (count == (buffersize + 100))
        {
            mean_gz = buff_gz / buffersize;
        }
        count++;
        delay(2); // Needed so we don't get repeated measures
    }

    // calibrating time
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        MPU3.setZGyroOffset(gz_offset);
        // used for testing purposes only
        // Serial.print("MPU Calibration output: GZ_offset set at:\t");
        // Serial.println(gz_offset);

        // sensor reading
        long buff_gz = 0;
        int count = 0;

        while (count < (buffersize + 101))
        {
            // read raw accel/gyro measurements from device
            MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU Output: GyroZ: ");
            // Serial.println(gz);

            if (count > 100 && count <= (buffersize + 100))
            { // First 100 measures are discarded
                buff_gz = buff_gz + gz;
            }
            if (count == (buffersize + 100))
            {
                mean_gz = buff_gz / buffersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        if (abs(mean_gz) <= gyro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);

        if (ready == 1)
            break;
    }
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor Gyro Z reading with offset:\t");
    Serial.println(mean_gz);
    Serial.print("Your new Gyro Z offset:\t");
    Serial.println(gz_offset);
};

///////////////////////////////////   SETUP   ////////////////////////////////////
void calibrateDatasetup()
{
    // start message
    Serial.println("\nMPU6050 Calibration Sketch");
    delay(2000);
    Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
    delay(3000);

    // reset offsets
    // DO NOT COMMENT THESE LINES IF YOU HAVE ALREADY SETUP THE OFFSETS
    MPU3.setXAccelOffset(0); 
    MPU3.setYAccelOffset(0); 
    MPU3.setZAccelOffset(0); 
    MPU3.setXGyroOffset(0); 
    MPU3.setYGyroOffset(0); 
    MPU3.setZGyroOffset(0); 
};

///////////////////////////////////   LOOP   ////////////////////////////////////
//----------------------------------------------------//
// Calibrate bias of the accelerometer and gyroscope
// Sensor needs to be calibrated at each power cycle.
//----------------------------------------------------//
void calibrateDataloop()
{
    if (state == 0)
    {
        Serial.println("\nReading sensors for first time...");
        meansensors();
        state++;
        delay(1000);
    }

    if (state == 1)
    {
        Serial.println("\nCalculating offsets...");
        calibration();
        state++;
        delay(1000);
    }

    if (state == 2)
    {
        meansensors();
        Serial.println("\nFINISHED!");
        Serial.print("\nSensor readings with offsets:\t");
        Serial.print(mean_ax);
        Serial.print("\t");
        Serial.print(mean_ay);
        Serial.print("\t");
        Serial.print(mean_az);
        Serial.print("\t");
        Serial.print(mean_gx);
        Serial.print("\t");
        Serial.print(mean_gy);
        Serial.print("\t");
        Serial.println(mean_gz);
        Serial.print("Your offsets:\t");
        Serial.print(ax_offset);
        Serial.print("\t");
        Serial.print(ay_offset);
        Serial.print("\t");
        Serial.print(az_offset);
        Serial.print("\t");
        Serial.print(gx_offset);
        Serial.print("\t");
        Serial.print(gy_offset);
        Serial.print("\t");
        Serial.println(gz_offset);
        Serial.println("\nData is printed as: accelX accelY accelZ gyroX gyroY gyroZ");
        // Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0"); // based on setAccelerometerRange(MPU6050_RANGE_2_G) = 16384
        Serial.println("Check that your sensor readings are close to 0 0 8192 0 0 0"); // based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192
        Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
        // while (1)
        //     ;

        // HERE IS THE RESPONSES:
        // Sensor readings with offsets:   2       -1      16388   0       -1      1
        // Your offsets : -527 - 4177 5472 - 191 - 41 - 12
        //
        // 7-mar-2024
        // Sensor readings with offsets:   7       -6      16383   0       -1      1
        // Your offsets : -524 - 4177 5446 - 187 - 40 - 11
        // Sensor readings with offsets:   3       -3      16379   0       0       0
        // Your offsets : -525 - 4177 5453 - 189 - 40 - 12
        // 6-mar-2024
        // Sensor readings with offsets:   0       -5      16378   -2      1       0
        // Your offsets : -527 - 4177 5469 - 191 - 40 - 13

        // Your offsets:	-529	-4179	5469	-191	-41	-14
        // 4-mar-2024
        // Sensor readings with offsets : -3 - 5 16389 - 1 0 - 1
        // Your offsets : -528 - 4178 5469 - 191 - 41 - 14
        // Data is printed as : acelX acelY acelZ giroX giroY giroZ Check that your sensor readings are close to 0 0 16384 0 0 0
    }
};

void calibrateData()
{
    calibrateDatasetup();
    Serial.println("Calibrating MPU6050");
    calibrateDataloop();
    Serial.println("Calibration Complete!");
};

// ============================================
// THIS PART IS NOT NEEDED BUT IT IS A GOOD EXAMPLE OF HOW TO GET THE ANGLE FROM THE MPU6050
// FOR STABILIZING THE TABLE

// unsigned long lastTime = 0;
//--------------------------------------------//
// Get angle from the gyroscope. Uint: degree
//--------------------------------------------//
// float getGyroRoll(int gyroX, int gyroBiasX, uint32_t lastTime)
// {
//     float gyroRoll;

//     // integrate gyroscope value in order to get angle value
//     gyroRoll = ((gyroX - gyroBiasX) / 131) * ((float)(micros() - lastTime) / 1000000);
//     return gyroRoll;
// };

//-----------------------------------------------//
// Get angle from the accelerometer. Uint: degree
//-----------------------------------------------//
// float getAccRoll(int accY, int accBiasY, int accZ, int accBiasZ)
// {
//     float accRoll;

//     // calculate angle value
//     accRoll = (atan2((accY - accBiasY), (accZ - accBiasZ))) * RAD_TO_DEG;

//     if (accRoll <= 360 && accRoll >= 180)
//     {
//         accRoll = 360 - accRoll;
//     }
//     return accRoll;
// };

//--------------------------------------------//
// Get current angle of the robot
//--------------------------------------------//
// float currentAngle()
// {

//     // Get raw IMU data
//     // readIMU();
//     MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     // Complementary filter for angle calculation
//     float gRoll = getGyroRoll(gx, gx_offset, lastTime);
//     float aRoll = getAccRoll(ay, ay_offset, az, az_offset);

//     float angleGet = 0.98 * (angleGet + gRoll) + 0.02 * (aRoll);

//     lastTime = micros(); // Reset the timer

//     return angleGet;
// };
// ============================================

#endif // USE_LIBRARY2

#endif // CALIBRATEDATA_H

// =========================================================
// END OF PROGRAM
// =========================================================