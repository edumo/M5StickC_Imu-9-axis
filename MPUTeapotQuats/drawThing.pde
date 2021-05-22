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
