// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: imu_zero.h
//
// Description:
//
// Provides the inital zero function for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     2-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef IMU_ZERO_H
#define IMU_ZERO_H

// MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
// 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-11 - added PID offset generation at begninning Generates first offsets
//                 - in @ 6 seconds and completes with 4 more sets @ 10 seconds
//                 - then continues with origional 2016 calibration code.
//      2016-11-25 - added delays to reduce sampling rate to ~200 Hz
//                   added temporizing printing during long computations
//      2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
//                   dynamic speed change when closing in
//      2016-10-22 - cosmetic changes
//      2016-10-19 - initial release of IMU_Zero
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release of MPU6050_RAW

/* ============================================
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

  If an MPU6050
      * is an ideal member of its tribe,
      * is properly warmed up,
      * is at rest in a neutral position,
      * is in a location where the pull of gravity is exactly 1g, and
      * has been loaded with the best possible offsets,
then it will report 0 for all accelerations and displacements, except for
Z acceleration, for which it will report 16384 (that is, 2^14).  Your device
probably won't do quite this well, but good offsets will all get the baseline
outputs close to these target values.

  Put the MPU6050 on a flat and horizontal surface, and leave it operating for
5-10 minutes so its temperature gets stabilized.

  Run this program.  A "----- done -----" line will indicate that it has done its best.
With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take
a few minutes to get there.

  Along the way, it will generate a dozen or so lines of output, showing that for each
of the 6 desired offsets, it is
      * first, trying to find two estimates, one too low and one too high, and
      * then, closing in until the bracket can't be made smaller.

  The line just above the "done" line will look something like
    [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]
As will have been shown in interspersed header lines, the six groups making up this
line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration,
and so on.

  The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.
===============================================
*/

#include <Arduino.h>
#include <Wire.h>

// #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <I2Cdev.h>

#define USE_LIBRARY3
#ifdef USE_LIBRARY3
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 MPUGyro;
Adafruit_MPU6050 ADAmpu;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA = ',';
const char BLANK = ' ';
const char PERIOD = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150; // empirical, to hold sampling to 200 Hz
const int NFast = 1000;   // the bigger, the better (but slower)
const int NSlow = 10000;  // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
static int Offsets[6]; // this will be used to store the final offsets
int LinesOut;
int N;

void ForceHeader()
{
    LinesOut = 99;
};

void GetSmoothed()
{
    int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
    {
        Sums[i] = 0;
    }
    //    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
    { // get sums
        MPUGyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                           &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
            Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
            Sums[j] = Sums[j] + RawValue[j];
    } // get sums
      //    unsigned long usForN = micros() - Start;
      //    Serial.print(" reading at ");
      //    Serial.print(1000000/((usForN+N/2)/N));
      //    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
    {
        Smoothed[i] = (Sums[i] + N / 2) / N;
    }
}; // GetSmoothed

void Initialize()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //     Wire.begin();
    // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //     Fastwire::setup(400, true);
    // #endif

    // initialize device
    // Serial.println("Initializing I2C devices...");
    // MPUGyro.initialize();

    // verify connection
    // Serial.println("Testing device connections...");
    // Serial.println(MPUGyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
    /*A tidbit on how PID (PI actually) tuning works.
      When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and
      integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral
      uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it
      to the integral value. Each reading narrows the error down to the desired offset. The greater the error from
      set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the
      integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the
      noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100
      readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to
      the fact it reacts to any noise.
    */
    MPUGyro.CalibrateAccel(6);
    MPUGyro.CalibrateGyro(6);
    Serial.println("\nat 600 Readings");
    MPUGyro.PrintActiveOffsets();
    Serial.println();
    MPUGyro.CalibrateAccel(1);
    MPUGyro.CalibrateGyro(1);
    Serial.println("\nat 700 Total Readings");
    MPUGyro.PrintActiveOffsets();
    Serial.println();
    MPUGyro.CalibrateAccel(1);
    MPUGyro.CalibrateGyro(1);
    Serial.println("\nat 800 Total Readings");
    MPUGyro.PrintActiveOffsets();
    Serial.println();
    MPUGyro.CalibrateAccel(1);
    MPUGyro.CalibrateGyro(1);
    Serial.println("\nat 900 Total Readings");
    MPUGyro.PrintActiveOffsets();
    Serial.println();
    MPUGyro.CalibrateAccel(1);
    MPUGyro.CalibrateGyro(1);
    Serial.println("\nat 1000 Total Readings");
    MPUGyro.PrintActiveOffsets();
    Serial.println("\n\nAny of the above offsets will work nice \n\nLets proof the PID tuning using another method:\n");
}; // Initialize

void SetOffsets(int TheOffsets[6])
{
    MPUGyro.setXAccelOffset(TheOffsets[iAx]);
    MPUGyro.setYAccelOffset(TheOffsets[iAy]);
    MPUGyro.setZAccelOffset(TheOffsets[iAz]);
    MPUGyro.setXGyroOffset(TheOffsets[iGx]);
    MPUGyro.setYGyroOffset(TheOffsets[iGy]);
    MPUGyro.setZGyroOffset(TheOffsets[iGz]);
}; // SetOffsets

void SetAveraging(int NewN)
{
    N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
}; // SetAveraging

void ShowProgress()
{
    if (LinesOut >= LinesBetweenHeaders)
    { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
    } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
    {
        Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
            Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
        {
            Serial.println(RBRACKET);
        }
        else
        {
            Serial.print("]\t");
        }
    }
    LinesOut++;
}; // ShowProgress

void PullBracketsIn()
{
    boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];

    Serial.println("\nclosing in:\n");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
    {
        StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
        {
            SetAveraging(NSlow);
        }
        else
        {
            AllBracketsNarrow = true;
        } // tentative
        for (int i = iAx; i <= iGz; i++)
        {
            if (HighOffset[i] <= (LowOffset[i] + 1))
            {
                NewOffset[i] = LowOffset[i];
            }
            else
            { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                {
                    AllBracketsNarrow = false;
                }
            } // binary search
        }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // closing in
            if (Smoothed[i] > Target[i])
            { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
            } // use lower half
            else
            { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
            } // use upper half
        }     // closing in
        ShowProgress();
    } // still working

    // attempting to get the last few readings
    Serial.print("final offsets: ");
    for (int j = 0; j < 6; j++)
    {
        Serial.print(NewOffset[j]);
        Offsets[j] = NewOffset[j];
        if (j < 5)
        {
            Serial.print(",\t");
        }
        else
        {
            Serial.println(".");
        }
    }
}; // PullBracketsIn

void PullBracketsOut()
{
    boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:\n");
    ForceHeader();

    while (!Done)
    {
        Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
            {
                Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
            }
            else
            {
                NextLowOffset[i] = LowOffset[i];
            }
        } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
            {
                Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
            }
            else
            {
                NextHighOffset[i] = HighOffset[i];
            }
        } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
        {
            LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
        }
    } // keep going
};    // PullBracketsOut

void IMUsetup(void)
{
    Initialize();
    for (int i = iAx; i <= iGz; i++)
    {                  // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel
        HighOffset[i] = 0;
        LowOffset[i] = 0;
    } // set targets and initial guesses
    // Target[iAz] = 16384; // based on 2G range
    Target[iAz] = 8192; // based on 4G range
    // Target[iAz] = 4096;  // based on 8G range
    // Target[iAz] = 2048;  // based on 16G range

    SetAveraging(NFast);

    PullBracketsOut(); // displays initial offsets
    PullBracketsIn();  // displays final offsets

    Serial.println("-------------- done --------------");
}; // setup

void IMUInitialization(void)
{
    // initialize the MPU6050 device
    // ADAmpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    // ADAmpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    ADAmpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    // ADAmpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    // ADAmpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    // ADAmpu.setGyroRange(MPU6050_RANGE_500_DEG);
    ADAmpu.setGyroRange(MPU6050_RANGE_250_DEG);
    ADAmpu.setCycleRate(MPU6050_CYCLE_40_HZ);
    // ADAmpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    ADAmpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    // ADAmpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // ADAmpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    // ADAmpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.println("Presets entered"); // used for testing purposes only

    // IMUsetup(); // function to see what possible calibration requirements are needed. // this initialization takes 5 minutes

    // Add the set____ below to supply your the gyro offsets here, scaled for min sensitivity
    // MPUGyro.setXAccelOffset(Offsets[0]);
    // Serial.print("XAccOffset = "); // used for testing purposes only
    // Serial.println(Offsets[0]);    // used for testing purposes only
    // MPUGyro.setYAccelOffset(Offsets[1]);
    // Serial.print("YAccOffset = "); // used for testing purposes only
    // Serial.println(Offsets[1]);   // used for testing purposes only
    // MPUGyro.setZAccelOffset(Offsets[2]);
    // Serial.print("ZAccOffset = "); // used for testing purposes only
    // Serial.println(Offsets[2]);  // used for testing purposes only
    // MPUGyro.setXGyroOffset(Offsets[3]);
    // Serial.print("XGyroOffset = "); // used for testing purposes only
    // Serial.println(Offsets[3]);   // used for testing purposes only
    // MPUGyro.setYGyroOffset(Offsets[4]);
    // Serial.print("YGyroOffset = "); // used for testing purposes only
    // Serial.println(Offsets[4]);   // used for testing purposes only
    // MPUGyro.setZGyroOffset(Offsets[5]);
    // Serial.print("ZGyroOffset = "); // used for testing purposes only
    // Serial.println(Offsets[5]);  // used for testing purposes only
    // Serial.println("MPU6050 new offsets are set"); // used for testing purposes only
};

#endif // USE_LIBRARY3

#endif // IMU_ZERO_H

// =========================================================
// END OF PROGRAM
// =========================================================