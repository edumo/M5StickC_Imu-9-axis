// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-20 - initial release

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
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

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

int MPUS = 1;

float[][] q = new float[MPUS][4];
Quaternion quat[] = {new Quaternion(1, 0, 0, 0), new Quaternion(1, 0, 0, 0), new Quaternion(1, 0, 0, 0)};

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

int lf = 10;    // Linefeed in ASCII
String myString = null;

void setup() {
  // 300px square viewport using OpenGL rendering
  size(900, 300, OPENGL);
  gfx = new ToxiclibsSupport(this);

  // setup lights and antialiasing
  lights();
  smooth();

  // display serial port list for debugging/clarity
  println(Serial.list());

  // get the first available port (use EITHER this OR the specific port code below)
  String portName = Serial.list()[3];

  // get a specific serial port (use EITHER this OR the first-available code above)
  //String portName = "COM4";

  // open the serial port
  port = new Serial(this, portName, 115200);

  // send single character to trigger DMP init/start
  // (expected by MPU6050_DMP6 example Arduino sketch)
  // port.write('r');
   myString = port.readStringUntil(lf);
  myString = null;
}

void draw() {
 
  // black background
  background(0);
  // translate everything to the middle of the viewport
  pushMatrix();
  translate(width / 2, height / 2);
  drawPlane(0);
  popMatrix();

}

void drawPlane(int planeId) {

  // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
  // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
  // different coordinate system orientation assumptions between Processing
  // and InvenSense DMP)
  float[] axis = quat[planeId].toAxisAngle();
  rotate(axis[0], -axis[1], axis[3], axis[2]);

  // draw main body in red
  fill(255, 0, 0, 200);
  box(10, 10, 200);

  // draw front-facing tip in blue
  fill(0, 0, 255, 200);
  pushMatrix();
  translate(0, 0, -120);
  rotateX(PI/2);
  drawCylinder(0, 20, 20, 8);
  popMatrix();
  drawWings();
}

void serialEvent(Serial port) {
  interval = millis();
  while (port.available() > 0) {
    int ch = port.read();

    if (synced == 0 && ch != '$') {
      print((char)ch);
      return;   // initial synchronization - also used to resync/realign if needed
    }
    
    synced = 1;
    print ((char)ch);

    if (
      (serialCount == 10 && ch != '\n')) {
      serialCount = 0;
      synced = 0;
      println("fuera "+serialCount+" "+(char)ch);
      return;
    }

    if (serialCount > 0 || ch == '$') {
      teapotPacket[serialCount++] = (char)ch;
      if (serialCount == 10) {
        serialCount = 0; // restart packet byte position

        int index = teapotPacket[10];
        index = 0;
        // get quaternion from data packet
        q[index][0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 32768.0f;
        q[index][1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 32768.0f;
        q[index][2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 32768.0f;
        q[index][3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 32768.0f;
        for (int i = 0; i < 4; i++) q[index][i] -= 1;
        for (int i = 0; i < 4; i++) if (q[index][i] >= 2) q[index][i] = -4 + q[index][i];
        
        // set our toxilibs quaternion to new data
        quat[index].set(q[index][0], q[index][1], q[index][2], q[index][3]);

      }
    }
  }
}
