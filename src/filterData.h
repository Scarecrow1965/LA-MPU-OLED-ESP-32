// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: filterData.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     12-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef FILTERDATA_H
#define FILTERDATA_H

#include <Arduino.h>
#include <Wire.h>

// libraries for the OLED
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

// libraries for MPU-6050
#include "calibrateData.h"
void calibrateXAccel(void);
void calibrateYAccel(void);
void calibrateZAccel(void);
void calibrateXGyro(void);
void calibrateYGyro(void);
void calibrateZGyro(void);

#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu6050;

// extern int16_t ax, ay, az, gx, gy, gz;
// extern int16_t temp;
// extern double tempCelsius;
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2; // This works but according to the function, it shouldn't
extern uint8_t oled_LineH;
// extern bool bLED; // LED state
extern Adafruit_SSD1306 display;
extern int Offsets[6];

// Initialize the angle and the previous gyro measurement
static float angle;




class FilterData
{

private:
    // variables
    int count = 0;
    long x_accel_buffer, y_accel_buffer, z_accel_buffer, x_gyro_buffer, y_gyro_buffer, z_gyro_buffer;
    uint16_t x_accel_mean = 0, y_accel_mean = 0, z_accel_mean = 0, x_gyro_mean = 0, y_gyro_mean = 0, z_gyro_mean = 0;
    uint16_t x_accel_offset, y_accel_offset, z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset;
    // Change this 3 variables if you want to fine tune the sketch to your needs.
    // int buffersize = 1000;     // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    int filtersize = 300; // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:300)
    // int acel_deadzone = 8;     // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    uint8_t x_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    uint8_t y_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    uint8_t z_accel_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    // int giro_deadzone = 1;     // Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    uint8_t x_gyro_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    uint8_t y_gyro_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    uint8_t z_gyro_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

    // based on IMU_Zero.h, I am currently using the following:
    // setAccelerometerRange(MPU6050_RANGE_4_G);, setGyroRange(MPU6050_RANGE_250_DEG);,
    // setFilterBandwidth(MPU6050_BAND_44_HZ);, setCycleRate(MPU6050_CYCLE_40_HZ);,
    double gyro_sensitivity = 131.0; // 131 LSB/(degrees/sec) // 250/Degrees/second
    // double gyro_sensitivity = 65.5;  // 65.5 LSB/(degrees/sec) // 500/Degrees/second
    // double gyro_sensitivity = 32.8;  // 32.8 LSB/(degrees/sec) // 1000/Degrees/second
    // double gyro_sensitivity = 16.4;  // 16.4 LSB/(degrees/sec) // 2000/Degrees/second
    // double accel_sensitivity = 16384.0; // 16384 LSB/g // 2g full scale range
    double accel_sensitivity = 8192.0; // 8192 LSB/g // 4g full scale range
    // double accel_sensitivity = 4096.0;  // 4096 LSB/g // 8g full scale range
    // double accel_sensitivity = 2048.0;  // 2048 LSB/g // 16g full scale range

    // Define the time step and the filter constant for the complimentary filter function
    float dt = 0.01;    // Time step (100Hz update rate)
    float alpha = 0.98; // Filter constant

    // Use with Kalman Filter
    const float Q_angle = 0.001;  // Process noise variance for the accelerometer
    const float Q_bias = 0.003;   // Process noise variance for the gyro bias
    const float R_measure = 0.03; // Measurement noise variance
    float angle = 0; // The angle calculated by the Kalman filter
    float bias = 0;  // The gyro bias calculated by the Kalman filter
    float rate;      // Unbiased rate calculated from the rate and the calculated bias
    unsigned long timer;
    float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

    // Function to see if the MPU-6050 is actually setup correctly
    bool isXAccelAboveThreshold = false;
    bool isYAccelAboveThreshold = false;
    bool isZAccelAboveThreshold = false;
    bool isXGyroAboveThreshold = false;
    bool isYGyroAboveThreshold = false;
    bool isZGyroAboveThreshold = false;

    uint8_t isXaccelaboveCount = 0;
    uint8_t isYaccelaboveCount = 0;
    uint8_t isZaccelaboveCount = 0;
    uint8_t isXgyroaboveCount = 0;
    uint8_t isYgyroaboveCount = 0;
    uint8_t isZgyroaboveCount = 0;

    int XaccelConsecutiveCount = 0;
    int YaccelConsecutiveCount = 0;
    int ZaccelConsecutiveCount = 0;
    int XgyroConsecutiveCount = 0;
    int YgyroConsecutiveCount = 0;
    int ZgyroConsecutiveCount = 0;

public:
    // functions

    // constructor
    void filterData()
    {
        count = 0;
        x_accel_buffer = 0, y_accel_buffer = 0, z_accel_buffer = 0, x_gyro_buffer = 0, y_gyro_buffer = 0, z_gyro_buffer = 0;

        while (count < (filtersize + 101))
        {
            // read raw accel/gyro measurements from device
            mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // used for testing purposes only
            // Serial.print("MPU before Filter: AccX: ");
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
            // Serial.println();

            if (count > 100 && count <= (filtersize + 100))
            { // First 100 measures are discarded
                x_accel_buffer = x_accel_buffer + ax;
                y_accel_buffer = y_accel_buffer + ay;
                z_accel_buffer = z_accel_buffer + az;
                x_gyro_buffer = x_gyro_buffer + gx;
                y_gyro_buffer = y_gyro_buffer + gy;
                z_gyro_buffer = z_gyro_buffer + gz;
            }
            if (count == (filtersize + 100))
            {
                x_accel_mean = x_accel_buffer / filtersize;
                y_accel_mean = y_accel_buffer / filtersize;
                z_accel_mean = z_accel_buffer / filtersize;
                x_gyro_mean = x_gyro_buffer / filtersize;
                y_gyro_mean = y_gyro_buffer / filtersize;
                z_gyro_mean = z_gyro_buffer / filtersize;
            }
            count++;
            delay(2); // Needed so we don't get repeated measures
        }

        // ================
        // x_accel_offset = x_accel_mean / accel_filter_X;
        // y_accel_offset = y_accel_mean / acel_filter_Y;
        z_accel_offset = (accel_sensitivity - z_accel_mean) / tempCelsius;
        // x_gyro_offset = x_gyro_mean / gyro_filter_X;
        // y_gyro_offset = y_gyro_mean / gyro_filter_Y;
        // z_gyro_offset = z_gyro_mean / gyro_filter_Z;
        //
        // this is what the offset would be like?
        // ax = x_accel_offset;
        // ay = y_accel_offset;
        az = z_accel_offset;
        // gx = x_gyro_offset;
        // gy = y_gyro_offset;
        // gz = z_gyro_offset;
        // ================

        // ================
        // this gives me a filtered look at what the MPU is actually feeling
        ax = x_accel_mean;
        ay = y_accel_mean;
        // az = z_accel_mean;
        gx = x_gyro_mean;
        gy = y_gyro_mean;
        gz = z_gyro_mean;
        // if ax and ay are within =/- 10, then the MPU is at rest
        // if gx, gy, and gz are within =/- 20, then the MPU is at rest
        // if az is close to 8192, then I could subtract mean_az from it
        // ================

        // used for testing purposes only
        Serial.print("MPU after Filter:\nAccX: ");
        Serial.print(ax);
        Serial.print("\tAccY: ");
        Serial.print(ay);
        Serial.print("\tAccZ: ");
        Serial.print(az);
        Serial.print("\tGyroX: ");
        Serial.print(gx);
        Serial.print("\tGyroY: ");
        Serial.print(gy);
        Serial.print("\tGyroZ: ");
        Serial.print(gz);
        temp = mpu6050.getTemperature();
        double tempCelsius = (temp / 340.00) + 36.53;
        Serial.print("\tTemps: ");
        Serial.println(tempCelsius);
        complimentaryFilter(); // adding one last line for Serial monitor
    };

    // destructor
    ~FilterData(){
        // nothing to destruct??
    };

    void displayFilteredGyro(void)
    {
        filterData(); // averaging raw outputs for 300 samples
        temp = mpu6050.getTemperature();
        double tempCelsius = (temp / 340.00) + 36.53; // convert to Celsius using MPU6050.h library

        display.clearDisplay(); // clears the display for adafruit library
        u8g2.clearBuffer(); // clears the display buffer for u8g2 library
        u8g2.home();
        u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.setCursor(3, oled_LineH * 1 + 2);
        u8g2.println("Filtered MPU Output");
        u8g2.setCursor(3, oled_LineH * 3 + 2);
        u8g2.println("Accel(m/s^2)");
        u8g2.setCursor(3, oled_LineH * 4 + 2);
        u8g2.print("X:");
        u8g2.print(ax);
        u8g2.print(", Y:");
        u8g2.print(ay);
        u8g2.print(", Z:");
        u8g2.print(az);
        u8g2.setCursor(3, oled_LineH * 5 + 2);
        u8g2.println("Gyro(rps)");
        u8g2.setCursor(3, oled_LineH * 6 + 2);
        u8g2.print("X:");
        u8g2.print(gx);
        u8g2.print(", Y:");
        u8g2.print(gy);
        u8g2.print(", Z:");
        u8g2.print(gz);
        u8g2.setCursor(3, oled_LineH * 7 + 2);
        u8g2.print("Temp= ");
        u8g2.print(tempCelsius);
        u8g2.print(" ");
        u8g2.print(char(176));
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.print("C");
        u8g2.sendBuffer(); // Send it out
    };                     // end displaying the MPU-6050 info without the fps

    void complimentaryFilter(void)
    {
        // Initialize the angle and the previous gyro measurement
        float angle = 0;
        float prevGyro = 0;

        // Get accelerometer and gyro readings
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Convert gyro readings to degrees/sec
        // float gyroRate = gx / 131.0; // based on 250 degrees/second
        float gyroRate = gx / gyro_sensitivity; // since I have already gotten the information from the datasheet

        // Calculate the angle from the accelerometer
        float accelAngle = atan2(ay, az) * 180 / PI;

        // Apply the complementary filter
        angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle;

        // Print the angle
        Serial.print("Complimentary Filter for X-axis: ");
        Serial.println(angle);

        // Update the previous gyro measurement
        prevGyro = gyroRate;

        // Delay to match the time step
        delay(10);
    }

    float kalmanloop(int16_t accelRaw, int16_t gyroRaw, float (*angleCalculation)(int16_t, int16_t))
    {
        // Get accelerometer and gyro readings
        // int16_t ax, ay, az, gx, gy, gz;
        // mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
        // Convert gyro readings to degrees/sec
        float gyroRate = gx / gyro_sensitivity; // sensitivity is 131 LSB/(degrees/sec) for 250 degrees/second
    
        // Calculate the angle from the accelerometer
        // float accelAngle = atan2(ay, az) * 180 / PI; // working on the Y-axis
        // float accelAngle = atan2(ax, az) * 180 / PI; // working on the X-axis
        float accelAngle = angleCalculation(accelRaw, az) * 180 / PI;

        // Calculate dt
        float dt = (micros() - timer) / 1000000.0;
        timer = micros();
    
        // Apply the Kalman filter
        float newAngle = kalmanFilter(accelAngle, gyroRate, dt);
    
        // Print the angle
        Serial.print("Kalman Loop newAngle:\t");
        Serial.println(newAngle);
        return newAngle;
    };
    
    float kalmanFilter(float newAngle, float newRate, float dt)
    {
        // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        rate = newRate - bias;
        angle += dt * rate;
    
        // Update estimation error covariance - Project the error covariance ahead
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
    
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        float S = P[0][0] + R_measure; // Estimate error
        float K[2];                    // Kalman gain - This is a 2x1 vector
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
    
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        float y = newAngle - angle; // Angle difference
        angle += K[0] * y;
        bias += K[1] * y;
    
        // Calculate estimation error covariance - Update the error covariance
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
    
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
    
        return angle;
    };
    

    // need to retrieve the offsets from the MPU6050
    void getMPUOffsets()
    {
        x_accel_offset = (int16_t)mpu6050.getXAccelOffset();
        y_accel_offset = (int16_t)mpu6050.getYAccelOffset();
        z_accel_offset = (int16_t)mpu6050.getZAccelOffset();
        x_gyro_offset = (int16_t)mpu6050.getXGyroOffset();
        y_gyro_offset = (int16_t)mpu6050.getYGyroOffset();
        z_gyro_offset = (int16_t)mpu6050.getZGyroOffset();

        // used for testing purposes only
        Serial.print("X AccOffset: ");
        Serial.print(x_accel_offset);
        Serial.print("\tY AccOffset: ");
        Serial.print(y_accel_offset);
        Serial.print("\tZ AccOffset: ");
        Serial.print(z_accel_offset);
        Serial.print("\tX GyroOffset: ");
        Serial.print(x_gyro_offset);
        Serial.print("\tY GyroOffset: ");
        Serial.print(y_gyro_offset);
        Serial.print("\tZ GyroOffset: ");
        Serial.println(z_gyro_offset);
    };

    // Function to control the speed of the linear actuator movement
    void handleAxis(int16_t axisValue, int16_t axisFilter, bool &isAxisAboveThreshold, uint8_t &axisAboveCount, int &axisConsecutiveCount, String axis, void (*calibrateFunc)())
    {
        if (abs(axisValue) > axisFilter)
        {
            Serial.print(axis);
            Serial.println(" Axis is not at rest");
            isAxisAboveThreshold = true;
            axisAboveCount = 1;
            axisConsecutiveCount++;
        }
        else if (abs(axisValue) < axisFilter)
        {
            Serial.print(axis);
            Serial.println(" Axis is at rest");
            isAxisAboveThreshold = false;
            axisAboveCount = 0;
            axisConsecutiveCount = 0;
        }

        if (isAxisAboveThreshold && axisAboveCount && axisConsecutiveCount > axisFilter)
        {
            Serial.print("Calibrating ");
            Serial.println(axis);
            calibrateFunc();
        }
    };

    void getThresholdResets()
    {
        handleAxis(ax, x_accel_filter, isXAccelAboveThreshold, isXaccelaboveCount, XaccelConsecutiveCount, "X Accel" ,calibrateXAccel);
        handleAxis(ay, y_accel_filter, isYAccelAboveThreshold, isYaccelaboveCount, YaccelConsecutiveCount, "Y Accel", calibrateYAccel);
        handleAxis(az, z_accel_filter, isZAccelAboveThreshold, isZaccelaboveCount, ZaccelConsecutiveCount, "Z Accel", calibrateZAccel);
        handleAxis(gx, x_gyro_filter, isXGyroAboveThreshold, isXgyroaboveCount, XgyroConsecutiveCount, "Gyro X", calibrateXGyro);
        handleAxis(gy, y_gyro_filter, isYGyroAboveThreshold, isYgyroaboveCount, YgyroConsecutiveCount, "Gyro Y", calibrateYGyro);
        handleAxis(gz, z_gyro_filter, isZGyroAboveThreshold, isZgyroaboveCount, ZgyroConsecutiveCount, "Gyro Z", calibrateZGyro);
    };

    // void getThresholdResets(void)
    // {
    //     getMPUOffsets();
    //     filterData(); // averaging raw outputs for 300 samples
    //     if (abs(ax) > x_accel_filter)
    //     {
    //         Serial.println("X-Axis is not at rest");
    //         isXAccelAboveThreshold = true;
    //         isXaccelaboveCount = 1;
    //         XaccelConsecutiveCount++;
    //     }
    //     else if (abs(ax) < x_accel_filter)
    //     {
    //         Serial.println("X-Axis is at rest");
    //         isXAccelAboveThreshold = false;
    //         isXaccelaboveCount = 0;
    //         XaccelConsecutiveCount = 0;
    //     }
    // 
    //    if (abs(ay) > y_accel_filter)
    //     {
    //         Serial.println("Y-Axis is not at rest");
    //         isYAccelAboveThreshold = true;
    //         isYaccelaboveCount = 1;
    //         YaccelConsecutiveCount++;
    //     }
    //     else if (abs(ay) < y_accel_filter)
    //     {
    //         Serial.println("Y-Axis is at rest");
    //         isYAccelAboveThreshold = false;
    //         isYaccelaboveCount = 0;
    //         YaccelConsecutiveCount = 0;
    //     }
    // 
    //     if (abs(az) > z_accel_filter)
    //     {
    //         Serial.println("Z-Axis is not at rest");
    //         isZAccelAboveThreshold = true;
    //         isZaccelaboveCount = 1;
    //         ZaccelConsecutiveCount++;
    //     }
    //     else if (abs(az) < z_accel_filter)
    //     {
    //         Serial.println("Z-Axis is at rest");
    //         isZAccelAboveThreshold = false;
    //         isZaccelaboveCount = 0;
    //         ZaccelConsecutiveCount = 0;
    //     }
    // 
    //     if (isXAccelAboveThreshold && isXaccelaboveCount && XaccelConsecutiveCount > x_accel_filter)
    //     {
    //         calibrateXAccel(); // function located in calibrateData.h
    //     }
    //     if (isYAccelAboveThreshold && isYaccelaboveCount && YaccelConsecutiveCount > y_accel_filter)
    //     {
    //         calibrateYAccel(); // function located in calibrateData.h
    //     }
    //     if (isZAccelAboveThreshold && isZaccelaboveCount && ZaccelConsecutiveCount > y_accel_filter)
    //     {
    //         calibrateZAccel(); // function located in calibrateData.h
    //     }
    //     if (isXGyroAboveThreshold && isXgyroaboveCount && XgyroConsecutiveCount > x_gyro_filter)
    //     {
    //         calibrateXGyro(); // function located in calibrateData.h
    //     }
    //     if (isYGyroAboveThreshold && isYgyroaboveCount && YgyroConsecutiveCount > y_gyro_filter)
    //     {
    //         calibrateYGyro(); // function located in calibrateData.h
    //     }
    //     if (isZGyroAboveThreshold && isZgyroaboveCount && ZgyroConsecutiveCount > z_gyro_filter)
    //     {
    //         calibrateZGyro(); // function located in calibrateData.h
    //     }
    // };

}; // end of class FilterData

#endif // FILTERDATA_H

// =========================================================
// END OF PROGRAM
// =========================================================