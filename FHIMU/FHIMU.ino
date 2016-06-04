// Far Horizons IMU
// Ryan Pierce, rdpierce@pobox.com
//
// History:
//
// This software is a modification of the software v053_MPU6000_DMP6_SPI.ino written by
// Martin Crown, and available at
// https://www.dropbox.com/s/a3cjxyue7lbiho5/v053_MPU6000_DMP6_SPI.ino
// As he acknowledges below, he based his software heavily on another sketch by Jeff Rowberg.
//
// An additional vector-based compass tilt correction was borrowed from Pololu at
// https://github.com/pololu/lsm303-arduino/blob/master/LSM303/
// This doesn't seem to work, and later Arduino versions won't compile the vector templates.
//
// This will run on an ArduIMU V3 board. It can be programmed via a 5V FTDI cable on the attached
// header. In the Arduino IDE, use Arduino Duemilanove w/ ATmega328. (Some 5V FTDI cables use 3.3V
// logic, which is fine.) Unfortunately, this microcontroller has only one UART. It can output IMU
// data, but to read GPS data, it must use the same baud rate for output as the GPS. Right now
// we are using 57600 baud for the data logging card.
//
// The ArduIMU board takes 5V in and can provide 3.3V to power a Copernicus GPS and/or
// a level shifter. Logic on the ArduIMU board is 5V.
//
// I have integrated a Parallax MS5607 barometric pressure sensor using I2C.

// ============================================================================================== //
// Version v053 - August 9, 2013 
//
// This Arduino sketch is made for the ArduIMU+ V3 board from 3DRobotics Inc.
// ( http://store.3drobotics.com/products/arduimu-v3 )
// and proven to work with Arduino IDE 1.0.5. Everything is explained as much as possible to help
// you get started or to adapt or expand it easily. Just read and learn!
//
// Stamp lasered on the MPU-6000 chip which is on my ArduIMU+ V3:
//     INVENSENSE
//     MPU-6000
//     D2P-339K1
//     EI 1306 E
// So from the last line you see that the MPU-6000 is produced in week 06 of 2013 as an E revision.
// (I don't think this will matter though, and probably any C or D revision will work as well)
//
// This sketch is largely based on the excellent (but complicated) sketch from Jeff Rowberg, version
// 6/21/2012, Copyright (c) 2012 J. Rowberg
// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//
// Jeff's sketch was made for an MPU-6050, which can only communicate using the I2C databus
// protocol. The ArduIMU+ V3 however has an MPU-6000 on it, which is the same as the MPU-6050, but
// is also able to communicate using the SPI databus protocol (in fact much simpler than I2C).
//
// And as it is, the MPU-6000 on the ArduIMU+ V3 is connected to the ATmega328P microcontroller
// (the "Arduino" processor) ONLY by the SPI databus. From 3DRobotics: "MPU-6000 uses SPI for max
// performance."
//
// So I have rewritten Jeff's sketch entirely, to work with the SPI protocol. The framework of the
// original sketch is maintained as far as possible, with the main difference that I put everything
// in a single sketchfile (this one) without including any other than two standard Arduino
// libraries (SPI.h and math.h). This made it easier -at least for me- because whatever happens when
// executing this sketch, I know the problem and the answer must be inside it and nowhere else.
//
// Disadvantage is that I left a lot of Jeff's code out of my "SPI translated" sketch just to obtain
// stable roll, pitch and yaw angles from the "quaternion", for which I needed the DMP functionality
// in the MPU-6000 to be enabled (this is what Jeff's code does). The Teapot data output was then a
// simple bonus added to this sketch [DMP = Digital Motion Processor which is inside the MPU-6000].
// -> Without the quaternion w, x, y and z data from the DMP, only the raw accelerometer x, y and z
//    data can be read from the MPU-6000 registers. From x, y and z only stable roll and pitch
//    values can be calculated, but calculating the yaw (z-axis) is impossible because yaw rotates
//    around the gravity vector which changes nothing in any of the x, y or z values.
// So I highly recommend Jeff's code for anyone wanting to expand this sketch to his/her own needs.
// (if your sketch freezes the Arduino software, kill process javaw.exe in the Windows Task Manager)
//
// Some lines of code for running the SPI protocol properly, and some info about undocumented DMP
// register settings were taken from AP_InertialSensor_MPU6000.cpp (unknown author):
// http://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_InertialSensor/AP_InertialSensor_MPU6000.cpp?name=ArduPlane-2.68
// Also some lines of code were taken from a sketch on forum.arduino.cc by user yan5619:
// http://forum.arduino.cc/index.php?topic=125623.0
//
// The so-called "Teapot" demo (also from Jeff and included seperately) takes data from this sketch
// through the software USB Serial Port (COM port) on your computer when connecting the ArduIMU+ V3
// with an FTDI FT232R or similar interface to the hardware USB port of your computer. The Teapot
// demo interface looks like it's Arduino, but in fact it is software called Processing
// ( http://www.processing.org ). If you have an older computer with a very basic graphics processor
// in it like mine, their newest version 2 might not run, so I used the older version 1.5.1 which
// still circulates on the internet.
// And yes, of course, I proved that the Teapot demo works with ArduIMU+ V3 and this sketch!
// - Close the Arduino Serial Monitor (but keep the ArduIMU+ V3 with running sketch connected via USB)
// - Open the Teapot .pde file in Processing
// - Somewhere at the beginning of the code make a small adaption to use the correct COM port
// - Run - Stop - Run again to get the demo going
//   (use the reset button on the ArduIMU+ V3 to get the ArduIMU+ V3 aligned with whatever you want)
//
// By the way, it took me some time to figure out why I could not upload any sketch to the
// ArduIMU+ V3 (the infamous "avrdude: stk500_getsync(): not in sync: resp=0x00" problem). I found
// out that the newest VCP driver from FTDI http://www.ftdichip.com (2.08.28) actually did not work
// on my (I admit quite old) Windows XP system. So I reverted to release 2.06.00: problem solved.
//
// Note: no calibrations are done on the raw values, so there may be room for improvement. To get
//       stable output values when the ArduIMU+ V3 is at rest, it helps a lot when you tape it to a
//       solid underground, for instance a block of wood.
//
// If you just want to get on with it without understanding exactly how it all works, run it, and
// put your own code in void loop() at the designated place.
//
// After uploading this sketch into the ArduIMU+ V3, you should see the blue LED flash briefly
// twice (DMP memory & configuration check), then the red LED starts flashing continuously (main
// loop is running - one flash every 20 loops). Brief flashes of the blue LED during the main loop
// denotes FIFO overflow (one flash = one overflow).
//
// I hope you enjoy this sketch, and have a lot of fun using it with the ArduIMU+ V3! 
//
// - best regards, Martin Crown
// ============================================================================================== //

// ############################################################################################## //
// ################################ HEADER ###################################################### //
// ############################################################################################## //

// in general, everything between an #ifdef and #endif statement is not compiled if the #ifdef
// statement is false: this is almost all debugging code, and if inactivated, it makes the final
// uploaded code a lot smaller and faster!

// standard Arduino includes
#include <SPI.h>  // now we can use the SPI data transfer protocol          ( http://arduino.cc/en/Reference/SPI )
#include <math.h> // now we can do some more than the standard calculations ( http://www.arduino.cc/en/Math/H )

#include <Wire.h>
#include <HMC5883LFH_T.h>
HMC5883L compass;
#define mag 0x1E //0011110b, I2C 7bit address of HMC5883
#define PRESSURE_ADDR 0x76 //0x77

// Magnetometer Corrections
// Should be in EEPROM but we only have one board.
#define MAG_X_OFFSET -8
#define MAG_Y_OFFSET 80
#define MAG_Z_OFFSET 40

//#define DEBUG // comment this line for no DEBUG output

// ============================================================================================== //
//
//                          MAKE YOUR CHOICE FOR OUTPUTS HERE AND NOW !!!
//
// uncomment "#define OUTPUT_RAW_ACCEL" if you want to see the actual raw X, Y and Z accelerometer
// components from the MPU-6000 FIFO (as also available from registers 3B & 3C, 3D & 3E and 3F & 40)
// ! expected 1 g value around 16384, but from FIFO it is around 8192 (half of the expected value)
// - no big deal since raw values will not be used for stable roll, pitch and yaw angles
//#define OUTPUT_RAW_ACCEL
//
// uncomment "#define OUTPUT_RAW_ACCEL_G" if you want to see the same as with OUTPUT_RAW_ACCEL but
// recalculated to g-force values (just divived by 8192)
#define OUTPUT_RAW_ACCEL_G
//
// uncomment "#define OUTPUT_RAW_ANGLES" if you want to see the calculated angles for roll and
// pitch from the raw acceleration components (yaw is undetermined then, this needs the use of the
// quaternion - see further on)
// around x-axis and y-axis: 0 to 360 degrees
// ! roll and pitch values calculated are somehow not independent from each other...hmmm...
#define OUTPUT_RAW_ANGLES
//
// uncomment "#define OUTPUT_RAW_GYRO" if you want to see the actual raw X, Y and Z gyroscope
// components from the MPU-6000 FIFO (as also available from registers 43 & 44, 45 & 46 and 47 & 48)
#define OUTPUT_RAW_GYRO
//
// uncomment "#define OUTPUT_TEMPERATURE" if you want to see the temperature of the MPU-6000 chip
// (which is not necessarily the same as the ambient temperature because of the power dissipation
// inside the MPU-6000 heating itself up a bit)
#define OUTPUT_TEMPERATURE
//
// uncomment "#define OUTPUT_READABLE_QUATERNION" if you want to see the actual quaternion
// components from the MPU-6000 FIFO [w, x, y, z] (not best for parsing on a remote host such as
// Processing or something though)
#define OUTPUT_READABLE_QUATERNION
//
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles (in degrees) calculated from
// the quaternions coming from the FIFO. Note that Euler angles suffer from gimbal lock (for more
// info, see http://en.wikipedia.org/wiki/Gimbal_lock )
// around x-axis and z-axis: -180 to +180 degrees, around y-axis 0 -> -90 -> 0 -> +90 -> 0
#define OUTPUT_READABLE_EULER
//
// uncomment "OUTPUT_READABLE_ROLLPITCHYAW" if you want to see the yaw/pitch/roll angles (in
// degrees) calculated from the quaternions coming from the FIFO. Note this also requires gravity
// vector calculations. Also note that yaw/pitch/roll angles suffer from gimbal lock (for more
// info, see: http://en.wikipedia.org/wiki/Gimbal_lock )
// roll, pitch, yaw: 0, 0, 0 when in horizontal flight
// roll/pitch:  0 -> -90 -> 0 -> 90 -> 0
//        yaw: -180 to +180
#define OUTPUT_READABLE_ROLLPITCHYAW
//
// uncomment "OUTPUT_MAGNETOMETER" if you want to see the magnetometer data from the HMC3883L
#define OUTPUT_MAGNETOMETER
//
// uncomment "OUTPUT_GPS" if you want to see GPS input data interleaved in the output
#define OUTPUT_GPS
//
// uncomment "OUTPUT_BARO" if you want to see barometric pressure and temperature
#define OUTPUT_BARO
//
// ============================================================================================== //

// easy name definitions to specific hardware pins so we can easily blink the LED's on and off
#define RED_LED_PIN     5 // connected to red    LED on ArduIMU+ V3 (Z on the board)
#define BLUE_LED_PIN    6 // connected to blue   LED on ArduIMU+ V3 (Y on the board)
                          //          the yellow LED on ArduIMU+ V3 (X on the board) can not be set
                          //          by the user because it is hardwired to the SPI clock (SCK)
byte blinkState = 0x00, blink_divider = 0; // controls (red) LED on or off and at what blink speed

// On the ArduIMU+ V3, output pin D7 on the ATmega328P controls a MUX which selects Serial Receive (RX)
// from a connected GPS module or from the FTDI (or similar) USB interface (more info further on).
#define SERIAL_MUX_PIN  7

// Serial.print definitions for debug output
#ifdef DEBUG
    #define DEBUG_PRINT(x)       Serial.print(x)
    #define DEBUG_PRINTF(x, y)   Serial.print(x, y)
    #define DEBUG_PRINTLN(x)     Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_DMP_CODE_SIZE         1929    // the number of values for writing the dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE        192    // the number of values for writing the dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE        47    // the number of values for writing the dmpUpdates[]

/* ================================================================================================ *\
 | Default MotionApps v2.0 42-byte FIFO packet structure (each value consists of 2 bytes):          |
 | -> this is array fifoBuffer[0-41]                                                                |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
\ * =============================================================================================== */

// This array contains the default DMP memory bank binary that gets loaded during dmpInitialize().
// It was reconstructed from observed I2C traffic generated by the UC3-A3 demo code, and not extracted
// directly from that code. That is true of all transmissions in this sketch, and any documentation has
// been added after the fact by referencing the Invensense code.
// It gets written to volatile memory, so it has to be done at each start (it only takes ~1 second though).
const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 137 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

// This array contains the DMP configurations that gets loaded during dmpInitialize().
// thanks to Noah Zerkin for piecing this stuff together!
const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,                   // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,                   // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,             // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,             // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,             // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,                   // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,                         // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,             // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,             // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,                   // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,             // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                               // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                               // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38,       // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                               // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                               // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,             // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,             // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x27                          // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100 Hz. Going faster than 100 Hz (0x00 = 200 Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200 Hz / (1 + value)).
    
    // Update: Setting to 0x18 which should set this to 8 Hz
    // Update: Setting to 0x27 which should set this to 5 Hz

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

// This array contains the DMP updates that get loaded during dmpInitialize().
const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM = {
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};

const int ChipSelPin1 = 4; // On the ArduIMU+ V3, output pin D4 on the ATmega328P connects to
                           // input pin /CS (/Chip Select) of the MPU-6000. Without a correct definition
                           // here, there is no SPI data transfer possible and this sketch will not work.

// MPU control & status variables
boolean dmpReady = false;     // set true if DMP initialization was successful
unsigned int packetSize = 42; // number of unique bytes of data written by the DMP each time (FIFO can hold multiples of 42-bytes)
unsigned int fifoCount;       // count of all bytes currently in FIFO
byte fifoBuffer[64];          // FIFO storage buffer (in fact only 42 used...)

// INTERRUPT FROM MPU-6000 DETECTION ROUTINE
volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
  {
  mpuInterrupt = true;
  }

// Far Horizons variables
uint32_t rowCount;
char sentenceBuf[100];
int sentenceBufIndex = 0;

uint16_t C[7];
uint32_t D1 = 0;
uint32_t D2 = 0;
float dT = 0;
int32_t TEMP = 0;
float OFF = 0; 
float SENS = 0; 
float P = 0;
float T2  = 0;
float OFF2  = 0;
float SENS2 = 0;
float Temperature;
float Pressure;

// ############################################################################################## //
// ########################################## Vector Math ####################################### //
// ############################################################################################## //

/*
template <typename T> struct vector
{
  T x, y, z;
};

template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector<float> *a)
{
  float magnitude = sqrt(vector_dot(a, a));
  a->x /= magnitude;
  a->y /= magnitude;
  a->z /= magnitude;
}
*/

// ############################################################################################## //
// ################################ SETUP ####################################################### //
// ############################################################################################## //
void setup()
  {
  
  pinMode(SERIAL_MUX_PIN, OUTPUT);   // Serial Mux Pin - SWITCHES SERIAL INPUT FROM GPS OR FTDI CHIP
  pinMode(RED_LED_PIN, OUTPUT);      // Red LED
  pinMode(BLUE_LED_PIN, OUTPUT);     // Blue LED
  
  rowCount = 0;
  sentenceBuf[0] = 0;
  sentenceBufIndex = 0;

  Serial.begin(57600); // start the serial communication at baud rate of 115200
  // (115200 chosen because it is required for Teapot Demo output, but it's
  //  really up to you depending on your project)
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Arduino
  // Pro Mini running at 3.3v, cannot handle the 115200 baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer. ArduIMU+ V3 runs at 16 MHz, so 115200 is no problem

  Serial.println("############# HMC5883L Setup #############");
  Wire.begin();
  compass = HMC5883L(); // Construct a new HMC5883 compass.

  Serial.println("Setting scale to +/- 1.3 Ga"); 
  int error = compass.SetScale(1.3F); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous

  digitalWrite(SERIAL_MUX_PIN, HIGH); // When LOW , set MUX to Receive Serial data from FTDI (upload sketch)
                                     // When HIGH, set MUX to Receive Serial data from GPS
  // This MUX is a really specific ArduIMU+ V3 module. This is what 3DRobotics calls
  // the "GPS port with FTDI autoswitch". This statement controls that "auto"switch. When connecting
  // a GPS module to the ArduIMU+ V3, the SERIAL_MUX_PIN must be set HIGH so that GPS data can be received.
  // It can't be both, so choose wisely or switch it actively at the right moments in this sketch.

  while(Serial.available() > 0) Serial.read(); // Flush serial buffer to clean up remnants from previous run
  Serial.println();
  Serial.println("############# MPU-6000 Data Acquisition #############");
    
  //--- SPI settings ---//
  Serial.println("Initializing SPI Protocol...");
  SPI.begin();  // start the SPI library
  SPI.setClockDivider(SPI_CLOCK_DIV16); // ArduIMU+ V3 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
                                        // 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
  SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
  SPI.setDataMode(SPI_MODE0); // latched on rising edge, transitioned on falling edge, active low
  Serial.println("...SPI Protocol initializing done.");
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  delay(100);
  
  //--- Configure the chip select pin as output ---//
  pinMode(ChipSelPin1, OUTPUT);
    
  // write & verify dmpMemory, dmpConfig and dmpUpdates into the DMP, and make all kinds of other settings
  // !!! this is the main routine to make the DMP work, and get the quaternion out of the FIFO !!!
  Serial.println("Initializing Digital Motion Processor (DMP)...");
  byte devStatus; // return status after each device operation (0 = success, !0 = error)
  devStatus = dmpInitialize();

  // make sure it worked: dmpInitialize() returns a 0 in devStatus if so
  if (devStatus == 0)
    {
    // now that it's ready, turn on the DMP 
    Serial.print("Enabling DMP... ");
    SPIwriteBit(0x6A, 7, true, ChipSelPin1); // USER_CTRL_DMP_EN
    Serial.println("done.");

    // enable Arduino interrupt detection, this will execute dmpDataReady whenever there is an interrupt,
    // independing on what this sketch is doing at that moment
    // http://arduino.cc/en/Reference/AttachInterrupt
    Serial.print("Enabling interrupt detection... ");
    // attachInterrupt(interrupt, function, mode) specifies a function to call when an external interrupt occurs
    // ArduIMU+ V3 has ATMEGA328 INT0 / D2 pin 32 (input) connected to MPU-6000 INT pin 12 (output)
    attachInterrupt(0, dmpDataReady, RISING); // the 0 points correctly to INT0 / D2
    // -> if there is an interrupt from MPU-6000 to ATMEGA328, boolean mpuInterrupt will be made true
    byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared
    Serial.println("done.");

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    Serial.println("DMP ready! Waiting for first data from MPU-6000...");

#ifdef OUTPUT_BARO    
    Serial.println("Configuring MS5607");
    Serial.println("PRESSURE SENSOR PROM COEFFICIENTS");
    Wire.beginTransmission(PRESSURE_ADDR);
    Wire.write(0x1E); // reset
    Wire.endTransmission();
    delay(10);
    for (int i=0; i<6  ; i++) {
      Wire.beginTransmission(PRESSURE_ADDR);
      Wire.write(0xA2 + (i * 2));
      Wire.endTransmission();
      Wire.beginTransmission(PRESSURE_ADDR);
      Wire.requestFrom(PRESSURE_ADDR, (uint8_t) 6);
      delay(1);
      if(Wire.available())
      {
        C[i+1] = Wire.read() << 8 | Wire.read();
      }
      else {
        Serial.println("Error reading PROM 1"); // error reading the PROM or communicating with the device
      }
      Serial.println(C[i+1]);
    }
    Serial.println();   
    Serial.println("Done configuring MS5607");
#endif
    
    Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // show chosen outputs
    /*
    Serial.println("You have chosen the following output(s):");
    #ifdef OUTPUT_RAW_ACCEL
    Serial.println("- OUTPUT_RAW_ACCEL");
    #endif
    #ifdef OUTPUT_RAW_ACCEL_G
    Serial.println("- OUTPUT_RAW_ACCEL_G");
    #endif
    #ifdef OUTPUT_RAW_ANGLES
    Serial.println("- OUTPUT_RAW_ANGLES");
    #endif
    #ifdef OUTPUT_RAW_GYRO
    Serial.println("- OUTPUT_RAW_GYRO");
    #endif
    #ifdef OUTPUT_TEMPERATURE
    Serial.println("- OUTPUT_TEMPERATURE");
    #endif
    #ifdef OUTPUT_READABLE_QUATERNION
    Serial.println("- OUTPUT_READABLE_QUATERNION");
    #endif
    #ifdef OUTPUT_READABLE_EULER
    Serial.println("- OUTPUT_READABLE_EULER");
    #endif
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
    Serial.println("- OUTPUT_READABLE_YAWPITCHROLL");
    #endif
    Serial.println();
    */
    
    }
   else // have to check if this is still functional
    {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    }

  } // end void setup()

// ############################################################################################## //
// ################################ MAIN LOOP ################################################### //
// ############################################################################################## //
// Sensor orientation ArduIMU+ V3:
// (all rotations "through" the MPU-6000 on the board)
// +X is ROLL   rotation along longer dimension of the board (towards GPS connector - "front")
// +Y is PITCH  rotation along shorter dimension of the board (towards GPS connector - "left")
// +Z is YAW    rotation around line upwards (through the MPU-6000 - "top")

void loop()
  {

  #ifdef DEBUG
    Serial.println(">>> Main loop start (must be seen over and over again)...");
  #endif

  // if DMP initialization during setup failed, don't try to do anything
  if (!dmpReady)
    {
    return;
    }


  
  // wait for MPU interrupt or extra packet(s) available
  // INFO: if there is an interrupt send from the MPU-6000 to the ATmega328P (the "Arduino" processor),
  //       boolean variable "mpuInterrupt" will be made "true" (see explanation in void setup() )
  while ((mpuInterrupt == false) && (fifoCount < packetSize)) {
    // do nothing until mpuInterrupt = true or fifoCount >= 42
    
    // Actually, no, check the serial port while we are waiting for IMU data
    #ifdef OUTPUT_GPS
    checkSerial();
    #endif
  }
  // Check again
  #ifdef OUTPUT_GPS
  checkSerial();
  #endif
  
  // there has just been an interrupt, so reset the interrupt flag, then get INT_STATUS byte
  mpuInterrupt = false;
  byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared

  // get current FIFO count
  fifoCount = getFIFOCount(ChipSelPin1);
  
  // check for FIFO overflow (this should never happen unless our code is too inefficient or DEBUG output delays code too much)
  if ((mpuIntStatus & 0x10) || fifoCount == 1008)
    // mpuIntStatus & 0x10 checks register 0x3A for FIFO_OFLOW_INT
    // the size of the FIFO buffer is 1024 bytes, but max. set to 1008 so after 24 packets of 42 bytes
    // the FIFO is reset, otherwise the last FIFO reading before reaching 1024 contains only 16 bytes
    // and can/will produces output value miscalculations 
    {
    // reset so we can continue cleanly
    //SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
    SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
    //SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

    digitalWrite(BLUE_LED_PIN, HIGH); // shows FIFO overflow without disturbing output with message
    DEBUG_PRINTLN("FIFO overflow! FIFO resetted to continue cleanly.");
    }

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
    // mpuIntStatus & 0x02 checks register 0x3A for (undocumented) DMP_INT
    {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = getFIFOCount(ChipSelPin1);
    
    digitalWrite(BLUE_LED_PIN, LOW); // LED off again now that FIFO overflow is resolved

    // read a packet from FIFO
    SPIreadBytes(0x74, packetSize, fifoBuffer, ChipSelPin1);

    // verify contents of fifoBuffer before use:
    # ifdef DEBUG
      for (byte n = 0 ; n < packetSize; n ++)
        {
        Serial.print("\tfifoBuffer["); Serial.print(n); Serial.print("]\t: "); Serial.println(fifoBuffer[n], HEX);
        }
    # endif

    // track FIFO count here in case there is more than one packet (each of 42 bytes) available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount = fifoCount - packetSize;

// ============================================================================================== //
// >>>>>>>>> - from here the 42 FIFO bytes from the MPU-6000 can be used to generate output >>>>>>>>
// >>>>>>>>> - this would be the place to add your own code into                            >>>>>>>>
// >>>>>>>>> - of course all the normal MPU-6000 registers can be used here too             >>>>>>>>
// ============================================================================================== //
    
    // get the quaternion values from the FIFO - needed for Euler and roll/pitch/yaw angle calculations
    int raw_q_w = ((fifoBuffer[0] << 8)  + fifoBuffer[1]);  // W
    int raw_q_x = ((fifoBuffer[4] << 8)  + fifoBuffer[5]);  // X
    int raw_q_y = ((fifoBuffer[8] << 8)  + fifoBuffer[9]);  // Y
    int raw_q_z = ((fifoBuffer[12] << 8) + fifoBuffer[13]); // Z
    float q_w = raw_q_w / 16384.0f;
    float q_x = raw_q_x / 16384.0f;
    float q_y = raw_q_y / 16384.0f;
    float q_z = raw_q_z / 16384.0f;

    ++rowCount;

    int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
    int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
    int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);

#ifdef OUTPUT_BARO
        D1 = getVal(PRESSURE_ADDR, 0x48); // Pressure raw
        D2 = getVal(PRESSURE_ADDR, 0x58);// Temperature raw

        dT   = (float)D2 - ((uint32_t)C[5] * 256);
        OFF  = ((float)C[2] * 131072) + ((dT * C[4]) / 64);
        SENS = ((float)C[1] * 65536) + (dT * C[3] / 128);

        TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;
  
        if(TEMP < 2000) // if temperature lower than 20 Celsius 
        {
    
          T2=pow(dT,2)/2147483648;
          OFF2=61*pow((TEMP-2000),2)/16;
          SENS2=2*pow((TEMP-2000),2);
    
          if(TEMP < -1500) // if temperature lower than -15 Celsius 
          {
            OFF2=OFF2+15*pow((TEMP+1500),2);
            SENS2=SENS2+8*pow((TEMP+1500),2);
          }
 
          TEMP = TEMP - T2;
          OFF = OFF - OFF2; 
          SENS = SENS - SENS2;
        }
 
        Temperature = (float)TEMP / 100; 
  
        P  = (D1 * SENS / 2097152 - OFF) / 32768;
        Pressure = (float)P / 100;
#endif



    
    /*
    #ifdef OUTPUT_RAW_ACCEL
      // print accelerometer values from fifoBuffer
      Serial.print("Raw acceleration ax, ay, az [8192 = 1 g]: "); Serial.print("\t\t");
      Serial.print  (AcceX); Serial.print("\t");
      Serial.print  (AcceY); Serial.print("\t");
      Serial.println(AcceZ);
    #endif
    */

    float Ax = AcceX / 8192.0f; // calculate g-value
    float Ay = AcceY / 8192.0f; // calculate g-value
    float Az = AcceZ / 8192.0f; // calculate g-value    
    #ifdef OUTPUT_RAW_ACCEL_G
      // same as OUTPUT_RAW_ACCEL but recalculated to g-force values
      /*
      Serial.print("Raw acceleration ax, ay, az [g]: "); Serial.print("\t\t\t");
      Serial.print  (Ax, 3); Serial.print("\t");
      Serial.print  (Ay, 3); Serial.print("\t");
      Serial.println(Az, 3);
      */
      Serial.print("$FHIAC,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(Ax);
      Serial.print(",");
      Serial.print(Ay);
      Serial.print(",");
      Serial.println(Az);
      
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif
        
    #ifdef OUTPUT_RAW_ANGLES
      // print calculated angles for roll and pitch from the raw acceleration components
      // (yaw is undetermined here, this needs the use of the quaternion - see further on)
      //int AcceX = ((fifoBuffer[28] << 8) + fifoBuffer[29]);
      //int AcceY = ((fifoBuffer[32] << 8) + fifoBuffer[33]);
      //int AcceZ = ((fifoBuffer[36] << 8) + fifoBuffer[37]);
      // atan2 outputs the value of -pi to pi (radians) - see http://en.wikipedia.org/wiki/Atan2
      // We then convert it to 0 to 2 pi and then from radians to degrees - in the end it's 0 - 360 degrees
      float ADegX = (atan2(AcceY, AcceZ) + PI) * RAD_TO_DEG;
      float ADegY = (atan2(AcceX, AcceZ) + PI) * RAD_TO_DEG;
      /*
      Serial.print("Calculated angle from raw acceleration - roll, pitch and yaw [degrees]: ");
      Serial.print(ADegX); Serial.print("\t");
      Serial.print(ADegY); Serial.print("\t");
      Serial.println("undetermined");
      */      
      Serial.print("$FHIAA,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(ADegX);
      Serial.print(",");
      Serial.println(ADegY);
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif    

    #ifdef OUTPUT_RAW_GYRO
      // print gyroscope values from fifoBuffer
      int GyroX = ((fifoBuffer[16] << 8) + fifoBuffer[17]);
      int GyroY = ((fifoBuffer[20] << 8) + fifoBuffer[21]);
      int GyroZ = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
      float GrateX = GyroX / 16.4f;
      float GrateY = GyroY / 16.4f;
      float GrateZ = GyroZ / 16.4f;
      /*
      Serial.print("Raw gyro rotation ax, ay, az [value/deg/s]: "); Serial.print("\t\t");
      Serial.print(GyroX); Serial.print("\t");
      Serial.print(GyroY); Serial.print("\t");
      Serial.println(GyroZ);
      */
      Serial.print("$FHIGR,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(GrateX);
      Serial.print(",");
      Serial.print(GrateY);
      Serial.print(",");
      Serial.println(GrateZ);      
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif

    #ifdef OUTPUT_TEMPERATURE
      // print calculated temperature from standard registers (not available in fifoBuffer)
      // the chip temperature may be used for correction of the temperature sensitivity of the
      // accelerometer and the gyroscope - not done in this sketch 
      byte Temp_Out_H = SPIread(0x41,ChipSelPin1);
      byte Temp_Out_L = SPIread(0x42,ChipSelPin1);
      int TemperatureRaw = Temp_Out_H << 8 | Temp_Out_L;
      float Temperature = (TemperatureRaw / 340.00) + 36.53; // formula from datasheet chapter 4.19
      //Serial.print("Chip temperature for corrections [deg. Celsius]: ");
      //Serial.println(Temperature, 2);
      Serial.print("$FHITC,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.println(Temperature);
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif

    #ifdef OUTPUT_READABLE_QUATERNION
      /*
      Serial.print("Quaternion qw, qx, qy, qz [-1 to +1]: "); Serial.print("\t");
      Serial.print  (q_w); Serial.print("\t");
      Serial.print  (q_x); Serial.print("\t");
      Serial.print  (q_y); Serial.print("\t");
      Serial.println(q_z);
      */
      Serial.print("$FHIQU,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(q_w);
      Serial.print(",");
      Serial.print(q_x);
      Serial.print(",");
      Serial.print(q_y);
      Serial.print(",");
      Serial.println(q_z);
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif     
    #endif

    #ifdef OUTPUT_READABLE_EULER
    // calculate Euler angles
    // http://en.wikipedia.org/wiki/Atan2
    // http://en.wikipedia.org/wiki/Sine (-> Inverse) 
    float euler_x = atan2((2 * q_y * q_z) - (2 * q_w * q_x), (2 * q_w * q_w) + (2 * q_z * q_z) - 1); // phi
    float euler_y = -asin((2 * q_x * q_z) + (2 * q_w * q_y));                                        // theta
    float euler_z = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1); // psi
    euler_x = euler_x * 180.0/M_PI; // angle in degrees -180 to +180
    euler_y = euler_y * 180.0/M_PI; // angle in degrees
    euler_z = euler_z * 180.0/M_PI; // angle in degrees -180 to +180
    /*
    Serial.print("Euler angles x, y, z [degrees]: ");
    Serial.print(euler_x); Serial.print("\t");
    Serial.print(euler_y); Serial.print("\t");
    Serial.print(euler_z); Serial.println();
    */
      Serial.print("$FHIEU,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(euler_x);
      Serial.print(",");
      Serial.print(euler_y);
      Serial.print(",");
      Serial.println(euler_z);
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif
      

      // display Euler angles in degrees

      // dmpGetGravity
      float grav_x = 2 * ((q_x * q_z) - (q_w * q_y));
      float grav_y = 2 * ((q_w * q_x) + (q_y * q_z));
      float grav_z = (q_w * q_w) - (q_x * q_x) - (q_y * q_y) + (q_z * q_z);
    
      // roll: (tilt left/right, about X axis)
      float rpy_rol = atan(grav_y / (sqrt((grav_x * grav_x) + (grav_z * grav_z))));
      // pitch: (nose up/down, about Y axis)
      float rpy_pit = atan(grav_x / (sqrt((grav_y * grav_y) + (grav_z * grav_z))));
      // yaw: (rotate around Z axis)
      float rpy_yaw = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1);
    #ifdef OUTPUT_READABLE_ROLLPITCHYAW
      /*
      Serial.print("Roll, pitch and yaw angles [degrees]: ");
      Serial.print(rpy_rol * 180/M_PI); Serial.print("\t");
      Serial.print(rpy_pit * 180/M_PI); Serial.print("\t");
      Serial.print(rpy_yaw * 180/M_PI); Serial.println();
      */
      Serial.print("$FHIPR,");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(rpy_pit*180.0/M_PI);
      Serial.print(",");
      Serial.print(rpy_rol*180.0/M_PI);
      Serial.print(",");
      Serial.println(rpy_yaw*180.0/M_PI);
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif

    #ifdef OUTPUT_MAGNETOMETER
      // Retrive the raw values from the compass (not scaled).
      MagnetometerRaw raw = compass.ReadRawAxis();
      Serial.print("$FHIMR,");   
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(raw.XAxis);
      Serial.print(",");
      Serial.print(raw.YAxis);
      Serial.print(","); 
      Serial.println(raw.ZAxis);    
      
      // Retrived the scaled values from the compass (scaled to the configured scale).  
      MagnetometerScaled scaled = compass.RawToScaled(&raw);
      // Correct the data
      raw.XAxis += MAG_X_OFFSET;
      raw.YAxis += MAG_Y_OFFSET;
      raw.ZAxis += MAG_Z_OFFSET;
      MagnetometerScaled scaledCorrected = compass.RawToScaled(&raw);
      
      Serial.print("$FHIMS,");   
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(scaled.XAxis);
      Serial.print(",");
      Serial.print(scaled.YAxis);
      Serial.print(","); 
      Serial.print(scaled.ZAxis);
      Serial.print(",");
      Serial.print(scaledCorrected.XAxis);
      Serial.print(",");
      Serial.print(scaledCorrected.YAxis);
      Serial.print(","); 
      Serial.println(scaledCorrected.ZAxis);      
      
      // Print heading info
      Serial.print("$FHIMH,");   
      Serial.print(rowCount);
      Serial.print(",");    
      // Compute raw heading
      float Heading, HeadingO;
      Heading = atan2(scaled.YAxis,scaled.XAxis);
      Heading += M_PI;
      Serial.print(Heading*180.0/M_PI);
      Serial.print(",");
      Heading = atan2(scaledCorrected.YAxis,scaledCorrected.XAxis);
      Heading += M_PI;
      Serial.print(Heading*180.0/M_PI);
      Serial.print(",");      
      // Tilt correction
      float cos_roll;
      float sin_roll;
      float cos_pitch;
      float sin_pitch;    
      cos_roll = cos(rpy_rol);
      sin_roll = sin(rpy_rol);
      cos_pitch = cos(rpy_pit);
      sin_pitch = sin(rpy_pit); 
      float TCMagX = scaled.XAxis*cos_pitch + scaled.YAxis*sin_roll*sin_pitch + 
        scaled.ZAxis*cos(rpy_rol)*sin_pitch;
      float TCMagY = scaled.YAxis*cos_roll + scaled.ZAxis*sin_roll;
      float TCMagOX = scaledCorrected.XAxis*cos_pitch + scaledCorrected.YAxis*sin_roll*sin_pitch + 
        scaledCorrected.ZAxis*cos(rpy_rol)*sin_pitch;
      float TCMagOY = scaledCorrected.YAxis*cos_roll + scaledCorrected.ZAxis*sin_roll;
      // Compute tilt corrected heading
      Heading = atan2(TCMagY,TCMagX);
      Heading += M_PI;
      // Compute tilt and offset corrected heading
      HeadingO = atan2(TCMagOY,TCMagOX);
      HeadingO += M_PI;      
      Serial.print(Heading*180.0/M_PI);
      Serial.print(",");
      Serial.println(HeadingO*180.0/M_PI);
      // This is another method
      /*
      vector<float> temp_m = { scaled.XAxis, scaled.YAxis, scaled.ZAxis };
      vector<float> temp_g = { Ax, Ay, Az };
      vector<float> E;
      vector<float> N;
      vector_cross(&temp_m, &temp_g, &E);
      vector_normalize(&E);
      vector_cross(&temp_g, &E, &N);
      vector_normalize(&N);
      vector<float> from = { -1.0, 0.0, 0.0 };
      Heading = atan2( vector_dot(&E, &from), vector_dot(&N, &from)) * 180.0 / M_PI;
      if ( Heading < 0 )
        Heading += 360;
      Serial.println(Heading);
      */
      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
    #endif   


#ifdef OUTPUT_BARO
      Serial.print("$FHPRS");
      Serial.print(",");
      Serial.print(rowCount);
      Serial.print(",");
      Serial.print(Temperature);
      Serial.print(",");
      Serial.print(Pressure);
      Serial.print(",");
      Serial.print(D2);
      Serial.print(",");
      Serial.print(D1);
      Serial.print(",");
      Serial.print(dT);
      Serial.print(",");
      Serial.print(SENS);
      Serial.print(",");
      Serial.print(OFF/100);
      Serial.print(",");
      Serial.print(T2);
      Serial.print(",");
      Serial.print(SENS2);
      Serial.print(",");
      Serial.println(OFF2/100);

      #ifdef OUTPUT_GPS
      checkSerial();
      #endif
#endif
      

      
// ============================================================================================== //
// >>>>>>>>> - this would normally be the end of adding your own code into this sketch          >>>>
// >>>>>>>>> - end of using the 42 FIFO bytes from the MPU-6000 to generate output              >>>>
// >>>>>>>>> - after blinking the red LED, the main loop starts again (and again, and again...) >>>>
// ============================================================================================== //
    }

  // blink LED to indicate activity
  blink_divider ++;
  if (blink_divider == 10) // toggle LED on/off every 10 main loops, otherwise it blinks so fast it looks like always on
    {
    blinkState = ~blinkState; // byte toggling between FF and 00
    if (blinkState == 0x00) digitalWrite(RED_LED_PIN, LOW);
    if (blinkState == 0xFF) digitalWrite(RED_LED_PIN, HIGH);
    blink_divider = 0;
    }

  } // end void loop()

// ############################################################################################## //
// ################################ SPI read/write functions #################################### //
// ############################################################################################## //

// --- Function for SPI reading one byte from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > register contents
byte SPIread(byte reg, int ChipSelPin)
  {
  DEBUG_PRINT("SPI (/CS"); DEBUG_PRINT(ChipSelPin); DEBUG_PRINT(") ");
  DEBUG_PRINT("reading 1 byte from register 0x");
  if (reg < 0x10) DEBUG_PRINT("0"); // add leading zero - this is an Arduino bug
  DEBUG_PRINTF(reg, HEX); DEBUG_PRINT("... ");
  digitalWrite(ChipSelPin, LOW);     // select MPU-6000 for SPI transfer (low active)
  SPI.transfer(reg | 0x80);          // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it
  byte read_value = SPI.transfer(0x00); // write 8-bits zero to MPU-6000, read the 8-bits coming back from reg at the same time
  digitalWrite(ChipSelPin, HIGH);    // deselect MPU-6000 for SPI transfer
  DEBUG_PRINT("0x");
  if (read_value < 0x10) DEBUG_PRINT("0"); // add leading zero - this is an Arduino bug
  DEBUG_PRINTF(read_value, HEX); DEBUG_PRINTLN(" (done)");
  return read_value;
  }

// --- Function for SPI writing one byte to sensor
// reg        : MPU-6000 register number to write to
// data       : data to be written into reg
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > nothing
void SPIwrite(byte reg, byte data, int ChipSelPin)
  {
  DEBUG_PRINT("SPI (/CS"); DEBUG_PRINT(ChipSelPin); DEBUG_PRINT(") ");
  DEBUG_PRINT("writing 1 byte to   register 0x"); DEBUG_PRINTF(reg, HEX); DEBUG_PRINT("... ");
  digitalWrite(ChipSelPin, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(ChipSelPin, HIGH);
  DEBUG_PRINT("0x");
  if (data < 0x10) DEBUG_PRINT("0"); // add leading zero - this is an Arduino bug
  DEBUG_PRINTF(data, HEX); DEBUG_PRINTLN(" (done)");
  }

// --- Function for SPI reading one bit from sensor
// reg        : MPU-6000 register number to read from
// bitNum     : bit number in the register to read - 7 (MSB) to 0 (LSB)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > byte 0x00 if bit is 0, otherwise byte with a 1 at bitNum (rest 0's)
byte SPIreadBit(byte reg, byte bitNum, int ChipSelPin)
  {
  byte byte_value = SPIread(reg, ChipSelPin);
  byte bit_value  = byte_value & (1 << bitNum); // AND result from register byte value and byte with only one "1" at place of bit to return (rest "0"'s)

  #ifdef DEBUG
  Serial.print(" bit_"); Serial.print(bitNum); Serial.print(" = "); 
  if (bit_value == 0x00)
    {
    Serial.println("0");
    }
  else
    {
    Serial.println("1");
    }
  #endif

  return bit_value;
  }

//--- Function for SPI writing one bit to sensor
// reg        : MPU-6000 register number to write to
// bitNum     : bit number in the register to write to - 7 (MSB) to 0 (LSB)
// databit    : bit value to be written into reg - false or 0 | true or non-zero (1 will be logical)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > nothing
//
// first read byte, then insert bit value, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void SPIwriteBit(byte reg, byte bitNum, byte databit, int ChipSelPin)
  {
  byte byte_value = SPIread(reg, ChipSelPin);
  if (databit == 0)
    {
    byte_value = byte_value & ~(1 << bitNum); // AND result from register byte value and byte with only one "0" at place of bit to write (rest "1"'s)
    }
  else // databit is intended to be a "1"
    {
    byte_value = byte_value |  (1 << bitNum); // OR  result from register byte value and byte with only one "1" at place of bit to write (rest "0"'s)
    }
  SPIwrite(reg, byte_value, ChipSelPin);
  
  #ifdef DEBUG
  Serial.print(" bit_"); Serial.print(bitNum); Serial.print(" set to "); 
  if (databit == 0)
    {
    Serial.println("0");
    }
  else
    {
    Serial.println("1");
    }
  #endif

  }

//--- Function for SPI reading multiple bytes to sensor
// read multiple bytes from the same device register, most of the times this
// is the FIFO transfer register (which after each read, is automatically
// loaded with new data for the next read)
// reg        : MPU-6000 register number to write to
// length     : number of bytes to be read
// data       : buffer array (starting with [0]) to store the read data in
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > array of data[0 - length]
void SPIreadBytes(byte reg, unsigned int length, byte *data, int ChipSelPin) 
  {
  #ifdef DEBUG
    Serial.print("SPI (/CS");
    Serial.print(ChipSelPin);
    Serial.print(") reading ");
    Serial.print(length, DEC);
    Serial.print(" byte(s) from 0x");
    if (reg < 0x10) {Serial.print("0");} // add leading zero - this is an Arduino bug
    Serial.print(reg, HEX);
    Serial.println("... ");
  #endif

  digitalWrite(ChipSelPin, LOW);
  delay(10); // wait 10 ms for MPU-6000 to react on chipselect (if this is 4 ms or less, SPI.transfer fails)
  SPI.transfer(reg | 0x80); // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it

  unsigned int count = 0;
  byte data_bytes_printed = 0;

  for (count = 0; count < length; count ++)
    {
    data[count] = SPI.transfer(0x00);
    #ifdef DEBUG
      if (data[count] < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
      Serial.print(data[count], HEX); Serial.print(" ");
      data_bytes_printed ++;
      if (data_bytes_printed == 16) // print lines of 16 bytes
        {
        Serial.println();
        data_bytes_printed = 0;
        }
    #endif
    }

  digitalWrite(ChipSelPin, HIGH);
  DEBUG_PRINTLN(" (done)");
  }

//--- Function for SPI reading multiple bits from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > databits
//
// 01101001 read byte
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
//    010   masked
//   -> 010 shifted
byte SPIreadBits(byte reg, byte bitStart, byte length, int ChipSelPin)
  {
    byte b = SPIread(reg, ChipSelPin);
    byte mask = ((1 << length) - 1) << (bitStart - length + 1);
    b = b & mask;
    b = b >> (bitStart - length + 1);
    return b;
  }

//--- Function for SPI writing multiple bits to sensor
// reg        : MPU-6000 register number to write to
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
//
// bbbbb010 -> data (bits to write - leading 0's)
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
// 00011100 mask byte
// 10101111 original reg value (read)
// 10100011 original reg value & ~mask
// 10101011 masked | original reg value
//
// first read byte, then insert bit values, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void SPIwriteBits(byte reg, byte bitStart, byte length, byte data, int ChipSelPin)
  {
  byte byte_value = SPIread(reg, ChipSelPin1);
  byte mask = ((1 << length) - 1) << (bitStart - length + 1); // create mask
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask;                     // zero all non-important bits in data (just to make sure)
  byte_value &= ~(mask);            // zero all important bits in existing byte, maintain the rest
  byte_value |= data;               // combine data with existing byte
  SPIwrite(reg, byte_value, ChipSelPin);

  #ifdef DEBUG
    Serial.print(" bits set: "); 
    for (byte i = 0; i < (7 - bitStart); i ++) Serial.print("x");
    for (byte j = 0; j < length; j ++) Serial.print(bitRead(data, bitStart - j));
    for (byte k = 0; k < (bitStart - length + 1); k ++) Serial.print("x");
    Serial.println();
  #endif

  }

// ############################################################################################## //
// ################################ DMP functions used in dmpInitialize() ####################### //
// ############################################################################################## //
// If you like to know how it works, please read on. Otherwise, just FIRE AND FORGET ;-)

void setMemoryBank(byte bank, boolean prefetchEnabled, boolean userBank, int ChipSelPin)
  {
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)
  bank = bank & 0x1F; // 0x1F = 00011111
  // bank 0: 0 & 0x1F = 00000000 $ 00011111 = 00000000
  // bank 1: 1 & 0x1F = 00000001 $ 00011111 = 00000001
  // bank 2: 2 & 0x1F = 00000010 $ 00011111 = 00000010
  // bank 3: 3 & 0x1F = 00000011 $ 00011111 = 00000011
  // bank 4: 4 & 0x1F = 00000100 $ 00011111 = 00000100
  // bank 5: 5 & 0x1F = 00000101 $ 00011111 = 00000101
  // bank 6: 6 & 0x1F = 00000110 $ 00011111 = 00000110
  // bank 7: 7 & 0x1F = 00000111 $ 00011111 = 00000111
  // is this to maximize the number of banks to 00011111 is 0x1F = 31 ?
  if (userBank) bank |= 0x20;
  if (prefetchEnabled) bank |= 0x40;
  SPIwrite(0x6D, bank, ChipSelPin);
  }

//***********************************************************//
void setMemoryStartAddress(byte startaddress, int ChipSelPin)
  {
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)
  SPIwrite(0x6E, startaddress, ChipSelPin);
  }


//***********************************************************//
boolean writeDMPMemory()
  {
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)

  Serial.print("\tWriting   DMP memory.......... ");
  
  unsigned int i, j;
  byte dmp_byte;

  // ### there are 8 DMP banks (numbers 0 to 7)
  
  // DMP banks 0 - 6 are completely filled with 256 bytes:
  for (i = 0; i < 7; i ++)
    {
    DEBUG_PRINT("@@@ write bank "); DEBUG_PRINTLN(i);
    setMemoryBank(i, false, false, ChipSelPin1); // bank number  = i
    setMemoryStartAddress(0, ChipSelPin1);       // startaddress = 0 so start writing every DMP bank from the beginning
    digitalWrite(ChipSelPin1,LOW);
    SPI.transfer(0x6F);

    for (j = 0; j < 256; j ++) // max. 256 bytes of data fit into one DMP bank
      {
      dmp_byte = pgm_read_byte(dmpMemory + (i * 256) + j);
      SPI.transfer(dmp_byte);
      #ifdef DEBUG
        if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
        Serial.println(dmp_byte, HEX);
      #endif
      }
    digitalWrite(ChipSelPin1,HIGH);
    DEBUG_PRINTLN();
    }

  // DMP bank 7 gets only 137 bytes:
  DEBUG_PRINTLN("@@@ write bank 7");
  setMemoryBank(7, false, false, ChipSelPin1); // bank number  = 7
  setMemoryStartAddress(0, ChipSelPin1);       // startaddress = 0 so start writing also this DMP bank from the beginning
  digitalWrite(ChipSelPin1,LOW);
  SPI.transfer(0x6F);

  for (j = 0; j < 137; j ++) // only 137 bytes of data into DMP bank 7
    {
    dmp_byte = pgm_read_byte(dmpMemory + (7 * 256) + j);
    SPI.transfer(dmp_byte);
    #ifdef DEBUG
      if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
      Serial.println(dmp_byte, HEX);
    #endif
    }
  digitalWrite(ChipSelPin1,HIGH);
  DEBUG_PRINTLN();

  Serial.println("done.");
  
  return true; // end of writeDMPMemory reached
  }

//***********************************************************//
boolean verifyDMPMemory()
  {
  // - the value in 0x6D activates a specific bank in the DMP
  // - the value in 0x6E sets the read/write pointer to a specific startaddress within the specified DMP bank
  // - register 0x6F is the register from which to read or to which to write the data
  //   (after each r/w autoincrement address within the specified DMP bank starting from startaddress)

  Serial.print("\tVerifying DMP memory.......... ");

  unsigned int i, j;
  byte dmp_byte, check_byte;
  boolean verification = true;
  
  // ### there are 8 DMP banks (numbers 0 to 7)
  
  // DMP banks 0 - 6 are completely read, all 256 bytes:
  for (i = 0; i < 7; i ++)
    {
    DEBUG_PRINT(">>> read bank "); DEBUG_PRINTLN(i);
    setMemoryBank(i, false, false, ChipSelPin1); // bank number  = i
    setMemoryStartAddress(0, ChipSelPin1);       // startaddress = 0 so start reading every DMP bank from the beginning
    digitalWrite(ChipSelPin1,LOW);
    SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

    for (j = 0; j < 256; j ++) // max. 256 bytes of data fit into one DMP bank
      {
      check_byte = pgm_read_byte(dmpMemory + (i * 256) + j);
      dmp_byte = SPI.transfer(0x00);
      if (dmp_byte != check_byte)
        {
        Serial.println("$$$ dmpMemory: byte verification error");
        verification = false;
        }
      #ifdef DEBUG
        if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
        Serial.println(dmp_byte, HEX);
      #endif
      }
    digitalWrite(ChipSelPin1,HIGH);
    DEBUG_PRINTLN();
    }

  // DMP bank 7 only read first 137 bytes:
  DEBUG_PRINTLN(">>> read bank 7");
  setMemoryBank(7, false, false, ChipSelPin1); // bank number  = 7
  setMemoryStartAddress(0, ChipSelPin1);       // startaddress = 0 so start reading also this DMP bank from the beginning
  digitalWrite(ChipSelPin1,LOW);
  SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

  for (j = 0; j < 137; j ++) // only 137 bytes of data into DMP bank 7
    {
    check_byte = pgm_read_byte(dmpMemory + (7 * 256) + j);
    dmp_byte = SPI.transfer(0x00);
    if (dmp_byte != check_byte)
      {
      Serial.println("$$$ dmpMemory: byte verification error");
      verification = false;
      }
    #ifdef DEBUG
      if (dmp_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
      Serial.println(dmp_byte, HEX);
    #endif
    }
  digitalWrite(ChipSelPin1,HIGH);
  DEBUG_PRINTLN();
  
  if (verification == true)  Serial.println("success!");
  if (verification == false) Serial.println("FAILED!");

  return verification; // true if DMP correctly written, false if not
  }

//***********************************************************//
boolean writeDMPConfig()
  {
  byte progBuffer, success, special;
  unsigned int i, j;
  // config set dmpConfig is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  byte bank, offset, length;

  Serial.print("\tWriting   DMP configuration... ");

  for (i = 0; i < MPU6050_DMP_CONFIG_SIZE;)
    {
    bank   = pgm_read_byte(dmpConfig + i++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
    offset = pgm_read_byte(dmpConfig + i++);
    length = pgm_read_byte(dmpConfig + i++);
    
    if (length > 0) // regular block of data to write
      {
      DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
      setMemoryBank(bank, false, false, ChipSelPin1); // bank number  = bank
      DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
      setMemoryStartAddress(offset, ChipSelPin1);     // startaddress = offset from the beginning (0) of the bank
      DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

      digitalWrite(ChipSelPin1,LOW);
      SPI.transfer(0x6F);

      for (j = 0; j < length; j++)
        {
        progBuffer = pgm_read_byte(dmpConfig + i + j);
        SPI.transfer(progBuffer);
        DEBUG_PRINTLNF(progBuffer, HEX);
        }
       
      digitalWrite(ChipSelPin1,HIGH);
      i = i + length;
      }
        
    else // length = 0; special instruction to write
      {
      // NOTE: this kind of behavior (what and when to do certain things)
      // is totally undocumented. This code is in here based on observed
      // behavior only, and exactly why (or even whether) it has to be here
      // is anybody's guess for now.
      special = pgm_read_byte(dmpConfig + i++);
      DEBUG_PRINTLN("!! Special command code ");
      DEBUG_PRINTF(special, HEX);
      DEBUG_PRINTLN(" found...");
      if (special == 0x01)
        {
        // enable DMP-related interrupts (ZeroMotion, FIFOBufferOverflow, DMP)
        SPIwrite(0x38, 0x32, ChipSelPin1);  // write 00110010: ZMOT_EN, FIFO_OFLOW_EN, DMP_INT_EN true
                                            // by the way: this sets all other interrupt enables to false
        success = true;
        }
      else
        {
        // unknown other special command if this may be needed in the future, but for now this should not happen
        success = false;
        }
      }
    }

  Serial.println("done.");

  return true;
  }

//***********************************************************//
boolean verifyDMPConfig()
  {
  byte check_byte, progBuffer, success, special;
  unsigned int i, j;
  // config set dmpConfig is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  byte bank, offset, length;
  boolean verification = true;

  Serial.print("\tVerifying DMP configuration... ");

  for (i = 0; i < MPU6050_DMP_CONFIG_SIZE;)
    {
    bank   = pgm_read_byte(dmpConfig + i++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
    offset = pgm_read_byte(dmpConfig + i++);
    length = pgm_read_byte(dmpConfig + i++);
      
    if (length > 0) // regular block of data to read
      {
      DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
      setMemoryBank(bank, false, false, ChipSelPin1); // bank number  = bank
      DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
      setMemoryStartAddress(offset, ChipSelPin1);     // startaddress = offset from the beginning (0) of the bank
      DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

      digitalWrite(ChipSelPin1,LOW);
      SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

      for (j = 0; j < length; j++)
        {
        progBuffer = pgm_read_byte(dmpConfig + i + j);
        check_byte = SPI.transfer(0x00);
        if (progBuffer != check_byte)
          {
          DEBUG_PRINTLN("$$$ dmpConfig: byte verification error");
          verification = false;
          }
        #ifdef DEBUG
          if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
          Serial.println(check_byte, HEX);
        #endif
        }
       
      digitalWrite(ChipSelPin1,HIGH);
      i = i + length;
      }
        
    else // length = 0; special instruction to write
      {
      // NOTE: this kind of behavior (what and when to do certain things)
      // is totally undocumented. This code is in here based on observed
      // behavior only, and exactly why (or even whether) it has to be here
      // is anybody's guess for now.
      special = pgm_read_byte(dmpConfig + i++);
      DEBUG_PRINT("!! Special command code ");
      DEBUG_PRINTF(special, HEX);
      DEBUG_PRINTLN(" found...");
      if (special == 0x01)
        {
        // enable DMP-related interrupts (ZeroMotion, FIFOBufferOverflow, DMP)
        check_byte = SPIread(0x38, ChipSelPin1);  // shoudl read 00110010: ZMOT_EN, FIFO_OFLOW_EN, DMP_INT_EN true
                                                  
        if (check_byte != 0x32)
          {
          DEBUG_PRINTLN("$$$ dmpConfig: byte verification error");
          verification = false;
          }
        #ifdef DEBUG
          if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
          Serial.println(check_byte, HEX);
        #endif

        success = true;
        }
      else
        {
        // unknown special command
        success = false;
        }
      }
    }

  if (verification == true)  Serial.println("success!");
  if (verification == false) Serial.println("FAILED!");

  return verification; // true if DMP correctly written, false if not
  }

//***********************************************************//
unsigned int writeDMPUpdates(unsigned int pos, byte update_number)
// process only one line from dmpUpdates each time writeDMPUpdates() is called
  {
  // pos is the current reading position within dmpUpdates
  byte progBuffer, success;
  unsigned int j;
  // config set dmpUpdates is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  byte bank, offset, length;

  Serial.print("\tWriting   DMP update "); Serial.print(update_number); Serial.print("/7 ..... ");


  bank   = pgm_read_byte(dmpUpdates + pos++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
  offset = pgm_read_byte(dmpUpdates + pos++);
  length = pgm_read_byte(dmpUpdates + pos++);
      
  DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
  setMemoryBank(bank, false, false, ChipSelPin1); // bank number  = bank
  DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
  setMemoryStartAddress(offset, ChipSelPin1);     // startaddress = offset from the beginning (0) of the bank
  DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

  digitalWrite(ChipSelPin1,LOW);
  SPI.transfer(0x6F);

  for (j = 0; j < length; j++)
    {
    progBuffer = pgm_read_byte(dmpUpdates + pos + j);
    SPI.transfer(progBuffer);
    DEBUG_PRINTLNF(progBuffer, HEX);
    }
       
  digitalWrite(ChipSelPin1,HIGH);
  pos = pos + length;
  DEBUG_PRINT("!! last position written: "); DEBUG_PRINTLN(pos);

  Serial.println("done.");

  return pos; // return last used position in dmpUpdates: will be starting point for next call!
  }

//***********************************************************//
unsigned int verifyDMPUpdates(unsigned int pos_verify, byte update_number)
// process only one line from dmpUpdates each time writeDMPUpdates() is called
  {
  // pos_verify is the current verifying position within dmpUpdates
  byte check_byte, progBuffer, success;
  unsigned int j;
  // config set dmpUpdates is a long string of blocks with the following structure:
  // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
  byte bank, offset, length;
  boolean verification = true;

  Serial.print("\tVerifying DMP update "); Serial.print(update_number); Serial.print("/7 ..... ");

  bank   = pgm_read_byte(dmpUpdates + pos_verify++); // pgm_read_byte() is a macro that reads a byte of data stored in a specified address(PROGMEM area)
  offset = pgm_read_byte(dmpUpdates + pos_verify++);
  length = pgm_read_byte(dmpUpdates + pos_verify++);
      
  DEBUG_PRINT("!! bank  : "); DEBUG_PRINTLNF(bank, HEX);
  setMemoryBank(bank, false, false, ChipSelPin1); // bank number  = bank
  DEBUG_PRINT("!! offset: "); DEBUG_PRINTLNF(offset, HEX);
  setMemoryStartAddress(offset, ChipSelPin1);     // startaddress = offset from the beginning (0) of the bank
  DEBUG_PRINT("!! length: "); DEBUG_PRINTLNF(length, HEX);

  digitalWrite(ChipSelPin1,LOW);
  SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it

  for (j = 0; j < length; j++)
    {
    progBuffer = pgm_read_byte(dmpUpdates + pos_verify + j);
    check_byte = SPI.transfer(0x00);
    if (progBuffer != check_byte)
      {
      DEBUG_PRINTLN("$$$ dmpUpdates: byte verification error");
      verification = false;
      }
    #ifdef DEBUG
      if (check_byte < 0x10) Serial.print("0"); // add leading zero - this is an Arduino bug
      Serial.println(check_byte, HEX);
    #endif
    }
       
  digitalWrite(ChipSelPin1,HIGH);
  pos_verify = pos_verify + length;
  DEBUG_PRINT("!! last position verified: "); DEBUG_PRINTLN(pos_verify);

  if (verification == true)  Serial.println("success!");
  if (verification == false) Serial.println("FAILED!");
  //return verification; // true if DMP correctly written, false if not

  return pos_verify; // return last used position in dmpUpdates: will be starting point for next call!
  }

//***********************************************************//
/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (defined by register 35 and 36).
 * - return: Current FIFO buffer size
 */
unsigned int getFIFOCount(int ChipSelPin)
  {
  // FIFO_COUNT should always be read in high-low order (0x72-0x73) in order to
  // guarantee that the most current FIFO Count value is read
  byte fifo_H = SPIread(0x72, ChipSelPin1);
  byte fifo_L = SPIread(0x73, ChipSelPin1);
  unsigned int two_bytes = (fifo_H << 8) | fifo_L;
  return two_bytes;
  }

// ############################################################################################## //
// ################################ Main DMP initialize function ################################ //
// ############################################################################################## //
// If you like to know how it works, please read on. Otherwise, just FIRE AND FORGET ;-)

byte dmpInitialize()
  {
  // Trigger a full device reset.
  // A small delay of ~50ms may be desirable after triggering a reset.
  DEBUG_PRINTLN(F("Resetting MPU6000..."));
  SPIwriteBit(0x6B, 7, true, ChipSelPin1); // DEVICE_RESET
  digitalWrite(BLUE_LED_PIN, HIGH); // shows start of DMP inititialize
  delay(30 + 170); // wait after reset + time to see the LED blink

  // Setting the SLEEP bit in the register puts the device into very low power
  // sleep mode. In this mode, only the serial interface and internal registers
  // remain active, allowing for a very low standby current. Clearing this bit
  // puts the device back into normal mode. To save power, the individual standby
  // selections for each of the gyros should be used if any gyro axis is not used
  // by the application.
  // disable sleep mode
  DEBUG_PRINTLN(F("Disabling sleep mode..."));
  SPIwriteBit(0x6B, 6, false, ChipSelPin1); // SLEEP

  // get MPU hardware revision
  DEBUG_PRINTLN(F("Selecting user bank 16..."));
  setMemoryBank(0x10, true, true, ChipSelPin1);
  DEBUG_PRINTLN(F("Selecting memory byte 6..."));
  setMemoryStartAddress(0x06, ChipSelPin1);
  DEBUG_PRINTLN(F("Checking hardware revision..."));
  digitalWrite(ChipSelPin1,LOW);
  SPI.transfer(0x6F | 0x80); // 0x6F | 0x80 causes a "1" added as MSB to 0x6F to denote reading from reg i.s.o. writing to it
  byte hwRevision = SPI.transfer(0x00);
  digitalWrite(ChipSelPin1,HIGH);
  DEBUG_PRINT(F("Revision @ user[16][6] = "));
  DEBUG_PRINTLNF(hwRevision, HEX);
  DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
  setMemoryBank(0, false, false, ChipSelPin1);

  // check OTP bank valid
  DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
  byte otpValid = SPIreadBit(0x00, 0, ChipSelPin1);
  DEBUG_PRINT(F("OTP bank is "));
  DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

  // get X/Y/Z gyro offsets
  DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
  byte xgOffsetTC = SPIreadBits(0x01, 6, 6, ChipSelPin1); // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
  byte ygOffsetTC = SPIreadBits(0x02, 6, 6, ChipSelPin1); // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
  byte zgOffsetTC = SPIreadBits(0x03, 6, 6, ChipSelPin1); // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
  DEBUG_PRINT("X gyro offset = ");
  DEBUG_PRINTLN(xgOffsetTC);
  DEBUG_PRINT("Y gyro offset = ");
  DEBUG_PRINTLN(ygOffsetTC);
  DEBUG_PRINT("Z gyro offset = ");
  DEBUG_PRINTLN(zgOffsetTC);

  // load DMP code into memory banks
  DEBUG_PRINT(F("########################### Writing DMP code to MPU memory banks ("));
  DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
  DEBUG_PRINTLN(F(" bytes)"));
  if (writeDMPMemory())
    {
    DEBUG_PRINTLN(F("########################### Success! DMP code written but not verified."));
    
    DEBUG_PRINTLN(F("########################### Verify DMP code..."));
    verifyDMPMemory();
        
    digitalWrite(BLUE_LED_PIN, LOW); // shows end of write DMP memory
    delay(200); // time to see the LED blink

    // write DMP configuration
    DEBUG_PRINT(F("########################### Writing DMP configuration to MPU memory banks ("));
    DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
    DEBUG_PRINTLN(F(" bytes in config def)"));
    if (writeDMPConfig())
      {
      DEBUG_PRINTLN(F("########################### Success! DMP configuration written but not verified."));

      DEBUG_PRINTLN(F("########################### Verify DMP configuration..."));
      verifyDMPConfig();
            
      digitalWrite(BLUE_LED_PIN, HIGH); // shows start of write DMP configuration
      delay(200); // time to see the LED blink

      DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
      SPIwriteBits(0x6B, 2, 3, 0x03, ChipSelPin1); // CLKSEL[2:0] = 011 = PLL with Z axis gyroscope reference

      DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
      SPIwrite(0x38, 0x12, ChipSelPin1); // INT_ENABLE = 00010010 = FIFO_OFLOW_EN & DMP_INT_EN

      // register INT_ENABLE 0x38:
      // bit 0 DATA_RDY_EN      0x01
      // bit 1 DMP_INT_EN       0x02 (undocumented) - enabling this bit also enables DATA_RDY_EN it seems
      // bit 2 UNKNOWN_INT_EN   0x04 (undocumented)
      // bit 3 I2C_MST_INT_EN   0x08
      // bit 4 FIFO_OFLOW_EN    0x10
      // bit 5 ZMOT_EN          0x20 (undocumented)
      // bit 6 MOT_EN           0x40
      // bit 7 FF_EN            0x80 (undocumented)

      // register INT_STATUS 0x3A:
      // bit 0 DATA_RDY_INT     0x01
      // bit 1 DMP_INT          0x02 (undocumented)
      // bit 2 UNKNOWN_INT      0x04 (undocumented)
      // bit 3 I2C_MST_INT      0x08
      // bit 4 FIFO_OFLOW_INT   0x10
      // bit 5 ZMOT_INT         0x20 (undocumented)
      // bit 6 MOT_INT          0x40
      // bit 7 FF_INT           0x80 (undocumented)
  
      DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
      //setRate(4); // 1kHz / (1 + 4) = 200 Hz (when DLPF_CFG enabled [1 to 6] - true, see below)
      SPIwrite(0x19, 4, ChipSelPin1); // SMPLRT_DIV[7:0] = 4 (ok)

      // FSYNC input not connnected on ArduIMU+ V3
      DEBUG_PRINTLN(F("Disable external frame synchronization..."));
      SPIwriteBits(0x1A, 5, 3, 0x00, ChipSelPin1); // EXT_SYNC_SET[2:0] = 000 = input disabled

      DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
      SPIwriteBits(0x1A, 2, 3, 0x03, ChipSelPin1); // DLPF_CFG[2:0] = 011 = accel 44 Hz gyro 42 Hz
      
      DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
      SPIwriteBits(0x1B, 4, 2, 0x03, ChipSelPin1); // FS_SEL[1:0] = 11 = +/- 2000 deg/s
      
      DEBUG_PRINTLN(F("Setting accelerometer full scale range to +/- 2 g..."));
      SPIwriteBits(0x1C, 4, 2, 0x00, ChipSelPin1);

      DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
      SPIwrite(0x70, 0x03, ChipSelPin1); // DMP related register
      SPIwrite(0x71, 0x00, ChipSelPin1); // DMP related register

      DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
      SPIwriteBit(0x00, 0, false, ChipSelPin1); // [0] OTP_BNK_VLD

      // enabling this part causes misalignment and drift of x, y and z axis
      // relative to ArduIMU+ V3/MPU-6000 x, y and z axis
      /*
      DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
      SPIwriteBits(0x00, 6, 6, xgOffsetTC, ChipSelPin1);
      SPIwriteBits(0x01, 6, 6, ygOffsetTC, ChipSelPin1);
      SPIwriteBits(0x02, 6, 6, zgOffsetTC, ChipSelPin1);
      */

      /*
      DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
      setXGyroOffset(0);
      setYGyroOffset(0);
      setZGyroOffset(0);
      */

      DEBUG_PRINTLN(F("###################### Writing final memory update 1/7 (function unknown)..."));
      byte update_number = 1;      // holds update number for user information
      unsigned int pos = 0;        // pos        is the current reading position within dmpUpdates; this is the first call; set pos        = 0 only once!
      pos = writeDMPUpdates(pos, update_number); 
      unsigned int pos_verify = 0; // pos_verify is the current reading position within dmpUpdates; this is the first call; set pos_verify = 0 only once!
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("Resetting FIFO..."));
      //SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
      SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
      //SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

      // Get current FIFO buffer size.
      // This value indicates the number of bytes stored in the FIFO buffer. This
      // number is in turn the number of bytes that can be read from the FIFO buffer
      // and it is directly proportional to the number of samples available given the
      // set of sensor data bound to be stored in the FIFO (register 35 and 36).
      DEBUG_PRINTLN(F("Reading FIFO count..."));
      unsigned int fifoCount = getFIFOCount(ChipSelPin1);
 
      // just after FIFO reset so count probably 0
      DEBUG_PRINT(F("Current FIFO count = "));
      DEBUG_PRINTLN(fifoCount);
      SPIreadBytes(0x74, fifoCount, fifoBuffer, ChipSelPin1);

      DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
      SPIwrite(0x1F, 2, ChipSelPin1); // MOT_THR[7:0] = 2

      DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
      SPIwrite(0x21, 156, ChipSelPin1); // detection threshold for Zero Motion interrupt generation

      DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
      SPIwrite(0x20, 80, ChipSelPin1); // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms

      DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
      SPIwrite(0x22, 0, ChipSelPin1); // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms

      DEBUG_PRINTLN(F("Resetting FIFO..."));
      //SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
      SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
      //SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

      DEBUG_PRINTLN(F("Enabling FIFO..."));
      SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

      DEBUG_PRINTLN(F("Enabling DMP..."));
      SPIwriteBit(0x6A, 7, true, ChipSelPin1); // USER_CTRL_DMP_EN

      DEBUG_PRINTLN(F("Resetting DMP..."));
      SPIwriteBit(0x6A, 3, true, ChipSelPin1); // Reset DMP

      DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      //delay(50); // may be used as test just to see if number of FIFO bytes changes

      DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
      while ((fifoCount = getFIFOCount(ChipSelPin1)) < 3);

      /* switched off, may crash the sketch (FIFO contents not used here anyway)
      // 1st read FIFO
      DEBUG_PRINT(F("Current FIFO count = "));
      DEBUG_PRINTLN(fifoCount);
      DEBUG_PRINTLN(F("Reading FIFO data..."));
      Serial.println("Reading FIFO data 1st time...");
      SPIreadBytes(0x74, fifoCount, fifoBuffer, ChipSelPin1);
      */

      DEBUG_PRINTLN(F("Reading interrupt status..."));
      byte mpuIntStatus = SPIread(0x3A, ChipSelPin1);

      DEBUG_PRINT(F("Current interrupt status = "));
      DEBUG_PRINTLNF(mpuIntStatus, HEX);

      // Jeff Rowberg's code had a read statement here... I suppose that must be a write statement!
      //DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
      //for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
      //readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
      DEBUG_PRINTLN(F("Writing final memory update 6/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
      while ((fifoCount = getFIFOCount(ChipSelPin1)) < 3);

      DEBUG_PRINT(F("Current FIFO count="));
      DEBUG_PRINTLN(fifoCount);

      /* switched off, may crash the sketch (FIFO contents not used here anyway)
      // 2nd read FIFO
      //DEBUG_PRINTLN(F("Reading FIFO data..."));
      Serial.println("Reading FIFO data 2nd time...");
      SPIreadBytes(0x74, fifoCount, fifoBuffer, ChipSelPin1);
      */

      DEBUG_PRINTLN(F("Reading interrupt status..."));
      mpuIntStatus = SPIread(0x3A, ChipSelPin1);

      DEBUG_PRINT(F("Current interrupt status="));
      DEBUG_PRINTLNF(mpuIntStatus, HEX);

      DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
      update_number ++;
      pos = writeDMPUpdates(pos, update_number); 
      pos_verify = verifyDMPUpdates(pos_verify, update_number);

      DEBUG_PRINTLN(F("DMP is good to go! Finally."));

      DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
      SPIwriteBit(0x6A, 7, false, ChipSelPin1); // USER_CTRL_DMP_EN

      DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
      //SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
      SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
      //SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable
      SPIread(0x3A, ChipSelPin1); // reading the register will clear all INT bits

      }
     else
      {
      DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
      return 2; // configuration block loading failed
      }

    }
  else
    {
    DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
    return 1; // main binary block loading failed
    }

  digitalWrite(BLUE_LED_PIN, LOW); // shows end of write DMP configuration
  delay(200); // time to see the LED blink

  Serial.println("... Digital Motion Processor (DMP) initializing done.");
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  return 0; // success
  }

// ############################################################################################## //
// ################################ Check Serial function ####################################### //
// ############################################################################################## //

void checkSerial() {
  while ( Serial.available() ) {
    char c;
    c = Serial.read();
    sentenceBuf[sentenceBufIndex] = c;
    sentenceBufIndex++;
    sentenceBuf[sentenceBufIndex] = 0;
    if ( sentenceBufIndex == 99 ) {
      // Overflow, start over
      sentenceBuf[0] = 0;
      sentenceBufIndex = 0;
    } else if ( c == 10 ) {
      // New sentence, output
      Serial.print( sentenceBuf );
      sentenceBuf[0] = 0;
      sentenceBufIndex = 0;
    }
  }     
}


// ############################################################################################## //
// ################################ I2C Baro Function     ####################################### //
// ############################################################################################## //

long getVal(int address, byte code)
{
  unsigned long ret = 0;
  Wire.beginTransmission(address);
  Wire.write(code);
  Wire.endTransmission();
  delay(10);
  // start read sequence
  Wire.beginTransmission(address);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)3);
  if (Wire.available() >= 3)
  {
    ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  }
  else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
}

// ############################################################################################## //
// ################################ ENDOF SKETCH ################################################ //
// ############################################################################################## //

