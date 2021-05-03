// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-20 - initial release
//     2021-4-01 - @edumo demo stickC

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

int MPUS = 3;

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
  translate(width / 4, height / 2);
  drawPlane(0);
  popMatrix();
}

void drawPlane(int planeId) {

  // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
  // ...and other weirdness I haven't figured out yet
  rotateY(radians(-(ypr[0]/100f - 240)));
  rotateZ(radians(-(ypr[1]/100f - 240)));
  rotateX(radians(-(ypr[2]/100f - 240)));

  println((ypr[0] / 100f) -240f);
  println((ypr[1] / 100f) -240f);
  println((ypr[2] / 100f) -240f);
  println();

  // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
  // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
  // different coordinate system orientation assumptions between Processing
  // and InvenSense DMP)
  //float[] axis = quat[planeId].toAxisAngle();
  //rotate(axis[0], -axis[1], axis[3], axis[2]);

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

    if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
    synced = 1;
    //print ((char)ch);

    if ((serialCount == 1 && ch != 2)
      || (serialCount == 12 && ch != '\r')
      || (serialCount == 13 && ch != '\n')) {
      serialCount = 0;
      synced = 0;
      return;
    }

    if (serialCount > 0 || ch == '$') {
      teapotPacket[serialCount++] = (char)ch;
      if (serialCount == 10) {
        serialCount = 0; // restart packet byte position

        // int index = teapotPacket[10];
        // get quaternion from data packet
        //q[index][0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        //q[index][1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        //q[index][2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
        //q[index][3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
        // for (int i = 0; i < 4; i++) if (q[index][i] >= 2) q[index][i] = -4 + q[index][i];

        // set our toxilibs quaternion to new data
        //quat[index].set(q[index][0], q[index][1], q[index][2], q[index][3]);

        /*
                // below calculations unnecessary for orientation only using toxilibs
         
         // calculate gravity vector
         gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
         gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
         gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
         
         // calculate Euler angles
         euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
         euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
         euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
         
         // calculate yaw/pitch/roll angles
         ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
         */
        //ypr[1] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        //ypr[2] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        ypr[0]  = ((teapotPacket[2] & 0xff) << 8) | (teapotPacket[3] & 0xff);
        //((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        ypr[1]  = ((teapotPacket[4] & 0xff) << 8) | (teapotPacket[5] & 0xff);//((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        ypr[2]  = ((teapotPacket[6] & 0xff) << 8) | (teapotPacket[7] & 0xff);//((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;

        /*
         // output various components for debugging
         //println("q:\t" + round(q[0]*100.0f)/100.0f + "\t" + round(q[1]*100.0f)/100.0f + "\t" + round(q[2]*100.0f)/100.0f + "\t" + round(q[3]*100.0f)/100.0f);
         //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
         //println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
         */
      }
    }
  }
}

void drawWings() {


  // draw wings and tail fin in green
  fill(0, 255, 0, 200);
  beginShape(TRIANGLES);
  vertex(-100, 2, 30); 
  vertex(0, 2, -80); 
  vertex(100, 2, 30);  // wing top layer
  vertex(-100, -2, 30); 
  vertex(0, -2, -80); 
  vertex(100, -2, 30);  // wing bottom layer
  vertex(-2, 0, 98); 
  vertex(-2, -30, 98); 
  vertex(-2, 0, 70);  // tail left layer
  vertex( 2, 0, 98); 
  vertex( 2, -30, 98); 
  vertex( 2, 0, 70);  // tail right layer
  endShape();
  beginShape(QUADS);
  vertex(-100, 2, 30); 
  vertex(-100, -2, 30); 
  vertex(  0, -2, -80); 
  vertex(  0, 2, -80);
  vertex( 100, 2, 30); 
  vertex( 100, -2, 30); 
  vertex(  0, -2, -80); 
  vertex(  0, 2, -80);
  vertex(-100, 2, 30); 
  vertex(-100, -2, 30); 
  vertex(100, -2, 30); 
  vertex(100, 2, 30);
  vertex(-2, 0, 98); 
  vertex(2, 0, 98); 
  vertex(2, -30, 98); 
  vertex(-2, -30, 98);
  vertex(-2, 0, 98); 
  vertex(2, 0, 98); 
  vertex(2, 0, 70); 
  vertex(-2, 0, 70);
  vertex(-2, -30, 98); 
  vertex(2, -30, 98); 
  vertex(2, 0, 70); 
  vertex(-2, 0, 70);
  endShape();
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();

  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}
