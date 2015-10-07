// MPU-6050 LED AHRS 
// By Patrick Lloyd
// Uses modified demo code from Jeff Rowberg's i2cdevlib library
// to visualize the 3D pose of the MPU-6050 with a single WS2812
// RGB LED light strip.

// License Goodness
/* ============================================
Adafruit NeoPixel library.
Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
contributions by PJRC, Michael Miller and other members of the open
source community.

NeoPixel is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

NeoPixel is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with NeoPixel.  If not, see
<http://www.gnu.org/licenses/>.
===============================================
*/

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Updates available at: https://github.com/jrowberg/i2cdevlib

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
===============================================
*/

// Watchdog timer to reset the device if it freezes. 
#include <avr/wdt.h>

#include <Adafruit_NeoPixel.h>  // NeoPixel library from Adafruit
#define PIXELPIN       4        // Arduino pin connected to strip
#define NUMPIXELS      60       // Total number of RGB LEDs on strip

#ifdef __AVR__
  #include <avr/power.h>        // AVR Specific power library
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// MotionApps utilizes the "Digital Motion Processor" (DMP) on the MPU-6050
// to filter and fuse raw sensor data into useful quantities like quaternions,
// Euler angles, or Yaw/Pitch/Roll inertial angles
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5.0v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// yaw/pitch/roll angles (in degrees) calculated from the quaternions
// coming from the FIFO. Note this also requires gravity vector
// calculations. Also note that yaw/pitch/roll angles suffer from gimbal
// lock (for more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                NEOPIXEL AHRS ROUTINE                     ===
// ================================================================

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example in the lbrary folder for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);


void update_led_ahrs(float yaw, float pitch, float roll) {
  // Note: the YPR values are in DEGREES! not radians
  
  // Clean slate. 
  pixels.clear();

  // Determine the "nose" of the AHRS from yaw angle indication
  int yaw_index = int(NUMPIXELS * (180.0 + yaw) / 360.0);

  // Light intensity for pitch and roll quadrants 
  float roll_brightness = 255 * roll / 180.0;
  float pitch_brightness = 255 * pitch / 180.0;

  // Bread and butter: Counts through a quadrant's worth of NeoPixel indecies
  // and determines the appropriate color for all four quadrants. The pitch and
  // roll brightness values then scale how red, blue, or green each pixel. This
  // emulates in admittedly simplistic fashion a Great Circle around an RGB
  // sphere with red at the "South Pole", green at the "Equator", and blue at 
  // the "North Pole". Green values are inversely proportional to pitch and roll
  int i;
  for (i = 0; i < (NUMPIXELS / 4); i++){
    if (pitch >= 0) {
      pixels.setPixelColor((yaw_index - (NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(pitch_brightness, 255-2*pitch_brightness, 0));
      pixels.setPixelColor((yaw_index + (3*NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(0, 255-2*pitch_brightness, pitch_brightness));
    } else {
      pixels.setPixelColor((yaw_index - (NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(0, 255 + 2*pitch_brightness, -1 * pitch_brightness));
      pixels.setPixelColor((yaw_index + (3*NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(-1 * pitch_brightness, 255 + 2*pitch_brightness, 0));
    }

    if (roll >= 0) {
      pixels.setPixelColor((yaw_index - (3*NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(roll_brightness, 255-roll_brightness, 0));
      pixels.setPixelColor((yaw_index + (NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(0, 255-roll_brightness, roll_brightness));
    } else {
      pixels.setPixelColor((yaw_index - (3*NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(0, 255 + 2*roll_brightness, -1 * roll_brightness));
      pixels.setPixelColor((yaw_index + (NUMPIXELS/8) + i) % NUMPIXELS, pixels.Color(-1 * roll_brightness, 255 + 2*roll_brightness, 0));
    }      
  }

  // Set the "nose indicator" and turn the device on
  pixels.setPixelColor(yaw_index, pixels.Color(255,255,255)); // White as can be
  pixels.show();
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // One watchdog timer will reset the device if it is unresponsive
  // for a second or more
  wdt_enable(WDTO_1S);
    
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Start the NeoPixel device and turn all the LEDs off
  pixels.begin();
  pixels.show(); // Initialize all pixels to 'off'

  // initialize serial communication
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // the program is alive...for now.
  wdt_reset();
    
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float yaw_deg = ypr[0] * 180/M_PI;
    float pitch_deg = ypr[1] * 180/M_PI;
    float roll_deg = ypr[2] * 180/M_PI;

    // make pretty colors happen
    update_led_ahrs(yaw_deg, pitch_deg, roll_deg);

    // print some descriptive YRP data
    Serial.print("ypr:\t");
    Serial.print(yaw_deg);
    Serial.print("\t");
    Serial.print(pitch_deg);
    Serial.print("\t");
    Serial.println(roll_deg);
  }
}
