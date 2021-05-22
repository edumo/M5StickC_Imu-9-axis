
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain


//float twoKp;    // 2 * proportional gain (Kp)
//  float twoKi;    // 2 * integral gain (Ki)
float myq0, myq1, myq2, myq3; // quaternion of sensor frame relative to auxiliary frame
float myintegralFBx, myintegralFBy, myintegralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
//  float roll, pitch, yaw;
char anglesComputed;

void initData() {
  twoKp = twoKpDef;  // 2 * proportional gain (Kp)
  twoKi = twoKiDef; // 2 * integral gain (Ki)
  myq0 = 1.0f;
  myq1 = 0.0f;
  myq2 = 0.0f;
  myq3 = 0.0f;
  myintegralFBx = 0.0f;
  myintegralFBy = 0.0f;
  myintegralFBz = 0.0f;
  anglesComputed = 0;
}

void myMahonyAHRSupdateIMU9Axis(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *q0, float *q1, float *q2, float *q3, float samplefrequency)
{
  static float myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;

  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    //updateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  invSampleFreq = (1.0f / samplefrequency);

  // Convert gyroscope degrees/sec to radians/sec
  //gx *= 0.0174533f;
  //gy *= 0.0174533f;
  //gz *= 0.0174533f;

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    //mx = mx+ 360.0 / 720.0;
    //my = my+ 360.0 / 720.0;
    //mz = mz+ 360.0 / 720.0;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = myq0 * myq0;
    q0q1 = myq0 * myq1;
    q0q2 = myq0 * myq2;
    q0q3 = myq0 * myq3;
    q1q1 = myq1 * myq1;
    q1q2 = myq1 * myq2;
    q1q3 = myq1 * myq3;
    q2q2 = myq2 * myq2;
    q2q3 = myq2 * myq3;
    q3q3 = myq3 * myq3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      // integral error scaled by Ki
      myintegralFBx += twoKi * halfex * invSampleFreq;
      myintegralFBy += twoKi * halfey * invSampleFreq;
      myintegralFBz += twoKi * halfez * invSampleFreq;
      gx += myintegralFBx;  // apply integral feedback
      gy += myintegralFBy;
      gz += myintegralFBz;
    } else {
      myintegralFBx = 0.0f; // prevent integral windup
      myintegralFBy = 0.0f;
      myintegralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += 2.0 * halfex;
    gy += 2.0 * halfey;
    gz += 2.0 * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * invSampleFreq);   // pre-multiply common factors
  gy *= (0.5f * invSampleFreq);
  gz *= (0.5f * invSampleFreq);
  qa = myq0;
  qb = myq1;
  qc = myq2;
  myq0 += (-qb * gx - qc * gy - myq3 * gz);
  myq1 += (qa * gx + qc * gz - myq3 * gy);
  myq2 += (qa * gy - qb * gz + myq3 * gx);
  myq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(myq0 * myq0 + myq1 * myq1 + myq2 * myq2 + myq3 * myq3);
  myq0 *= recipNorm;
  myq1 *= recipNorm;
  myq2 *= recipNorm;
  myq3 *= recipNorm;
  anglesComputed = 0;
  
  *q0 = myq0;
  *q1 = myq1;
  *q2 = myq2;
  *q3 = myq3;
}


/* This is a modified version of the original MahonyAHRSupdateIMU algorithm (which can be found in M5StickC/src/utility/MahonyAHRS.cpp) */
void myMahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *q0, float *q1, float *q2, float *q3, float samplefrequency) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  //static float myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
  //myq0 = 1.0, myq1 = 0.0, myq2 = 0.0, myq3 = 0.0;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = myq1 * myq3 - myq0 * myq2;
    halfvy = myq0 * myq1 + myq2 * myq3;
    halfvz = myq0 * myq0 - 0.5f + myq3 * myq3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply proportional feedback
    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / samplefrequency));    // pre-multiply common factors
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = myq0;
  qb = myq1;
  qc = myq2;
  myq0 += (-qb * gx - qc * gy - myq3 * gz);
  myq1 += (qa * gx + qc * gz - myq3 * gy);
  myq2 += (qa * gy - qb * gz + myq3 * gx);
  myq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(myq0 * myq0 + myq1 * myq1 + myq2 * myq2 + myq3 * myq3);
  myq0 *= recipNorm;
  myq1 *= recipNorm;
  myq2 *= recipNorm;
  myq3 *= recipNorm;

  *q0 = myq0;
  *q1 = myq1;
  *q2 = myq2;
  *q3 = myq3;

}
