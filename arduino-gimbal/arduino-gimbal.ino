//////////////////////////////////////////////////////////////////////////////////////////////
//                         ____               _             _                               //
//                        |___ \             / \    __  __ (_)  ___                         //
//                          __) |  _____    / _ \   \ \/ / | | / __|                        //
//                         / __/  |_____|  / ___ \   >  <  | | \__ \                        //
//                        |_____|         /_/   \_\ /_/\_\ |_| |___/                        //
//    ____                                      ____   _               _               _    //
//   / ___|    ___   _ __  __   __   ___       / ___| (_)  _ __ ___   | |__     __ _  | |   //
//   \___ \   / _ \ | '__| \ \ / /  / _ \     | |  _  | | | '_ ` _ \  | '_ \   / _` | | |   //
//    ___) | |  __/ | |     \ V /  | (_) |    | |_| | | | | | | | | | | |_) | | (_| | | |   //
//   |____/   \___| |_|      \_/    \___/      \____| |_| |_| |_| |_| |_.__/   \__,_| |_|   //
//                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////
//   By Eric Roch and Ankur Mehta                                       November 26, 2016   //
//   ECE 110 Final Project                     University of Illinois at Urbana-Champaign   //
//////////////////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>
#include <SparkFun_ADXL345.h>

#define DEBUG   // COMMENT to disable debug printing
                // UNCOMMENT to enable debug printing

///////////////////////////////////////////////////////////////
//    ____           _     _     _                           //
//   / ___|    ___  | |_  | |_  (_)  _ __     __ _    ___    //
//   \___ \   / _ \ | __| | __| | | | '_ \   / _` |  / __|   //
//    ___) | |  __/ | |_  | |_  | | | | | | | (_| |  \__ \   //
//   |____/   \___|  \__|  \__| |_| |_| |_|  \__, |  |___/   //
//                                           |___/           //
///////////////////////////////////////////////////////////////

// Sensitivity mode
// 0 - 2g, 1 - 4g, 2 - 8g, 3 - 16g
int mode = 2;

int range = pow( 2, mode+1 );


////////////////////////////////////////////////////////////////////////
//   __     __                 _           _       _                  //
//   \ \   / /   __ _   _ __  (_)   __ _  | |__   | |   ___    ___    //
//    \ \ / /   / _` | | '__| | |  / _` | | '_ \  | |  / _ \  / __|   //
//     \ V /   | (_| | | |    | | | (_| | | |_) | | | |  __/  \__ \   //
//      \_/     \__,_| |_|    |_|  \__,_| |_.__/  |_|  \___|  |___/   //
//                                                                    //                                            
////////////////////////////////////////////////////////////////////////

// objects for the accelerometer and servos
ADXL345 adxl = ADXL345();
Servo servoX;
Servo servoY;

// arrays to store calibration data
// Offsets = 0.5 * (Accel_+1 + Accel_-1)    [units: g]
// Gains = 0.5 * (Accel_+1 - Accel_-1)/1g   [units: none]
const float offsets[4][3] = {
    // x    y    z
    { 7.0, 7.0, -2.5},  //  2g
    { 4.0, 3.5, -2.5},  //  4g
    { 1.5, 1.5, -1.5},  //  8g
    { 0.5, 0.5, -1.0}   // 16g
};
const float gains[4][3] = {
    // x      y      z
    {257.0, 257.0, 246.5},  //  2g
    {129.0, 128.5, 125.5},  //  4g
    { 64.5,  64.5,  62.5},  //  8g
    { 32.5,  32.5,  31.0}   // 16g
};

// variables to store the servo positions
int xPos = 0;
int yPos = 0;

// variables to store accelerometer readings
int x, y, z;
int prevX, prevY, prevZ;
float avgX, avgY, avgZ;
float normX, normY, normZ;


//////////////////////////////////////////////////////////////////////
//    ____                                                          //
//   |  _ \    _ __    ___     __ _    _ __    __ _    _ __ ___     //
//   | |_) |  | '__|  / _ \   / _` |  | '__|  / _` |  | '_ ` _ \    //
//   |  __/   | |    | (_) | | (_| |  | |    | (_| |  | | | | | |   //
//   |_|      |_|     \___/   \__, |  |_|     \__,_|  |_| |_| |_|   //
//                            |___/                                 //
//////////////////////////////////////////////////////////////////////

void setup() {
    // start the serial data communication
    Serial.begin(9600);
    
    // setup the servo on pins 20/21
    servoX.attach(21);
    servoY.attach(20);

    // move each servo through it's range of motion
    test_range(servoX, 25, 175, 100);
    test_range(servoY, 13, 165, 89);
    
    // turn on the accelerometer
    adxl.powerOn();
    adxl.setRangeSetting(range);
    
    delay(500);
}

void loop() {
    adxl.readAccel(&x, &y, &z);
    
    // wrap values back to negative negative numbers
    // --> 32767 is the upper limit of signed integers
    // --> 65536 is the range of unsigned integers
    if (x > 32767) x -= 65536;
    if (y > 32767) y -= 65536;
    if (z > 32767) z -= 65536;

    // take the average of the current and previous readings to reduce jitter
    avgX = (x+prevX)/2;
    avgY = (y+prevY)/2;
    avgZ = (z+prevZ)/2;

    // normalize readings based on calibration data
    // returns values in the range -100 to 100 (at 1g)
    normX = (avgX - offsets[mode][0]) / gains[mode][0] * 100;
    normY = (avgY - offsets[mode][1]) / gains[mode][1] * 100;
    normZ = (avgZ - offsets[mode][2]) / gains[mode][2] * 100;

    // map the normalized readings to servo values
    xPos = cmap(normX, -100, 100, 25, 175);
    yPos = cmap(normY, -100, 100, 13, 165);
    
    servoX.write(xPos);
    servoY.write(yPos);

    // if debugging is enabled, print values to the serial monitor
    print_debug();

    // store the current readings as the previous readings
    prevX = x; prevY = y; prevZ = z;

    // Short delay to allow servos to adjust
    // Minimum delay = 20 msec
    delay(20);
}

// Maps a value from one range to another, constraining the output value
// to the output range.  If the input range is zero (fromLow == fromHigh),
// the function returns zero.
int cmap(float val, int fromLow, int fromHigh, int toLow, int toHigh) {
    int fromRange = fromHigh - fromLow;
    int toRange = toHigh - toLow;

    if (fromRange != 0) {
        int mapped = int( (val - fromLow) * toRange / fromRange + toLow );
        return constrain(mapped, toLow, toHigh);
    }
    else {
        return 0;
    }
}

// Sweeps the given servo from low to high and
// back to low before ending at last.
void test_range(Servo servo, int low, int high, int last) {
    for (int i = low; i < high; i += 5) {
        servo.write(i);
        delay(20);
    }
    for (int i = high; i > last; i -= 5) {
        servo.write(i);
        delay(20);
    }
    servo.write(last);  // make sure the servo ends at last
    delay(200);
}

// Print raw and normalized sensor values to
// the serial monitor for debugging
void print_debug() {
    #ifdef DEBUG
        Serial.print(x); Serial.print(", ");
        Serial.print(y); Serial.print(", ");
        Serial.print(z); Serial.println();

        Serial.print(normX); Serial.print(", ");
        Serial.print(normY); Serial.print(", ");
        Serial.print(normZ); Serial.println();
    #endif
}
