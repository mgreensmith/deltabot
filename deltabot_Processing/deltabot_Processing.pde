/*
 * DeltaBot
 * v0.1
 * Author: Matt Greensmith
 * Date: 25 May 2011
 *
 * All the kinematics calculations are from the superb write-up at:
 * http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
 */
import controlP5.*;
import processing.serial.*;

boolean debug = false;

// robot geometry
static float e = 14;     // end effector triangle
static float f = 29;     // base triangle
static float re = 50;    //length of long arm
static float rf = 14;     //length of short arm
 
// trigonometric constants
static float sqrt3 = sqrt(3.0);
static float pi = 3.141592653;    // PI
static float sin120 = sqrt3/2.0;   
static float cos120 = -0.5;        
static float tan60 = sqrt3;
static float sin30 = 0.5;
static float tan30 = 1/sqrt3;

//servo alignment
static int s1offset = -19;
static int s2offset = -14;
static int s3offset = -22;

//envelope boundaries
static float min_x = -20;
static float max_x = 20;
static float min_y = -20;
static float max_y = 20;
static float min_z = -64;
static float max_z = -36;
static float max_t = 80;
static float min_t = -45;

static int precision = 100; //round to 2 decimal places

float xp = 0;
float yp = 0;
float zp =-45;
float t1 = 0;  //servo angle t for 'theta', 1 for servo 1
float t2 = 0;
float t3 = 0;

//backups of previous positions in case we try to make an illegal move
float last_x;
float last_y;
float last_z;
float last_t1;
float last_t2;
float last_t3;

boolean validPosition;
boolean servosEnabled = false;

Serial myPort;
ControlP5 controlP5;
PFont font;
Textfield xtext;
Textfield ytext;
Textfield ztext;
Slider2D xy;
Slider z;
Slider servo1;
Slider servo2;
Slider servo3;

void setup() {
  //open serial connection
  myPort = new Serial(this, Serial.list()[2], 9600);
  
  size(300,600);
  frameRate(25);
  controlP5 = new ControlP5(this);  
  //controlP5.setColorLabel(0xff000000);
  controlP5.setAutoInitialization(false);
 
  // x axis text field
  xtext = controlP5.addTextfield("x",170,60,50,20);
  xtext.setId(1);
  xtext.setAutoClear(false);
  
  // y axis text field
  ytext = controlP5.addTextfield("y",170,100,50,20);
  ytext.setId(2);
  ytext.setAutoClear(false);
  
  //z axis text field
  ztext = controlP5.addTextfield("z",170,140,50,20);
  ztext.setId(5);
  ztext.setAutoClear(false);
  
  //x-y axis 2d slider
  xy = controlP5.addSlider2D("x-y-axis",20,200,200,200);
  xy.setBroadcast(false); 
  xy.setId(3);
  xy.setMinX(0);
  xy.setMaxX(40);
  xy.setMinY(0);  //minY is the top of the chart!
  xy.setMaxY(40);
  xy.setArrayValue(new float[] {20, 20});
  xy.setColorValueLabel(200);  //hide the incorrect labels
 
  //z azis slider
  z = controlP5.addSlider("z-axis",-64,-36,-45,230,200,15,200);
  z.setBroadcast(false); 
  z.setId(4);
  z.setSliderMode(Slider.FLEXIBLE);
  z.setColorValueLabel(200); 
  
  //servo sliders
  servo1 = controlP5.addSlider("servo1", -45, 80, 0, 20, 460, 200, 15);
  servo1.setBroadcast(false); 
  servo1.setId(6);
  servo1.setSliderMode(Slider.FLEXIBLE);
  
  servo2 = controlP5.addSlider("servo2", -45, 80, 0, 20, 480, 200, 15);
  servo2.setBroadcast(false); 
  servo2.setId(7);
  servo2.setSliderMode(Slider.FLEXIBLE);
  
  servo3 = controlP5.addSlider("servo3", -45, 80, 0, 20, 500, 200, 15);
  servo3
  .setBroadcast(false); 
  servo3.setId(8);
  servo3.setSliderMode(Slider.FLEXIBLE);
  
  //enable switch
  controlP5.addToggle("enable", false, 20, 20, 50, 20).setMode(ControlP5.SWITCH);
  
  updateGuiElements();
}

void draw() {
  background(200);
  if (servosEnabled) {
    fill(200);
    text("SERVO POWER OFF", 100, 35); //hide
    fill(#00bb00);
    text("SERVO POWER ON", 100, 35); //show this
  } else {
    fill(200);
    text("SERVO POWER ON", 100, 35); //hide
    fill(#ff0000);
    text("SERVO POWER OFF", 100, 35); //show this
  }
}

public void controlEvent(ControlEvent theEvent) {
  if ( debug ) {
    println("controlEvent: id: "+
          theEvent.controller().getId()+"\tname: "+
          theEvent.controller().getName()+"\tlabel: "+
          theEvent.controller().getLabel()+"\tvalue: "+
          theEvent.controller().getValue());
    println("xp: " + xp + "\typ: " + yp + "\tzp: " + zp + "\tt1: " + t1 + "\tt2: " + t2 + "\tt3: " + t3);
  }
  
  copyLastData(); //back up our last positions

  switch(theEvent.controller().getId()) {
    case (1): //x position text field has been changed
      xp = Float.valueOf(xtext.getStringValue()).floatValue();
      setThetasfromXYZ();
    break;

    case (2): //y position text field has been changed
      yp = Float.valueOf(ytext.getStringValue()).floatValue();
      setThetasfromXYZ();
    break;
    
    case (5): //z position text field has been changed
      zp = Float.valueOf(ztext.getStringValue()).floatValue();
      setThetasfromXYZ();
    break;
   
    case (3): //xy slider has changed
      xp = (xy.getArrayValue()[0] -20) * -1;       //slider is 0,100, convert to -50,50
      yp = (xy.getArrayValue()[1] -20) * -1;  //slider is 0,100, convert to -50,50, invert
      setThetasfromXYZ();
    break;
    
    case (4): //z slider
      zp = z.getValue();
      setThetasfromXYZ();   
    break;
    
    case (6)://servo1 slider
      t1 = servo1.getValue();
      setXYZfromThetas();
    break;
    
    case (7)://servo2 slider
      t2 = servo2.getValue();
      setXYZfromThetas();
    break;
    
    case (8)://servo3 slider
      t3 = servo3.getValue();
      setXYZfromThetas();
    break;   
 
  }
}

//a new xyz has been proposed, validate the position and calculate new servo angles.
void setThetasfromXYZ() {
  if ( debug ) { println("Entering: setThetasfromXYZ()"); }
  //first bounds-check the input
  if (xp < min_x) { xp = min_x; }
  if (xp > max_x) { xp = max_x; }
  if (yp < min_y) { yp = min_y; }
  if (yp > max_y) { yp = max_y; }
  if (zp < min_z) { zp = min_z; }
  if (zp > max_z) { zp = max_z; }
  
  validPosition = true;
  //set the first angle
  float theta1 = delta_calcAngleYZ(xp, yp, zp);
  if (theta1 != 999) {
    float theta2 = delta_calcAngleYZ(xp*cos120 + yp*sin120, yp*cos120-xp*sin120, zp);  // rotate coords to +120 deg
    if (theta2 != 999) {
      float theta3 = delta_calcAngleYZ(xp*cos120 - yp*sin120, yp*cos120+xp*sin120, zp);  // rotate coords to -120 deg
      if (theta3 != 999) {
        //we succeeded - point exists
        if (theta1 <= max_t && theta2 <= max_t && theta3 <= max_t && theta1 >= min_t && theta2 >= min_t && theta3 >= min_t ) { //bounds check
          t1 = theta1;
          t2 = theta2;
          t3 = theta3;
          SetServos();
        } else {
          validPosition = false;
        }

      } else {
        validPosition = false;
      }
    } else {
      validPosition = false;
    }
  } else {
    validPosition = false;
  }
  
  //uh oh, we failed, revert to our last known good positions
  if ( !validPosition ) {
    xp = last_x;
    yp = last_y;
    zp = last_z;
  }
        
  //finally update the gui elements
  updateGuiElements();
  if ( debug ) { println("Exiting: setThetasfromXYZ()"); }
}

//a new theta position has been proposed, validate the position and calculate new servo angles.
void setXYZfromThetas() {
  if ( debug ) { println("Entering: setXYZfromThetas()"); }  
  validPosition = true;

  float t = (f-e)*tan30/2;
  float dtr = pi/(float)180.0;
 
  float theta1 = dtr*t1;
  float theta2 = dtr*t2;
  float theta3 = dtr*t3;
 
  float y1 = -(t + rf*cos(theta1));
  float z1 = -rf*sin(theta1);
   
  float y2 = (t + rf*cos(theta2))*sin30;
  float x2 = y2*tan60;
  float z2 = -rf*sin(theta2);
   
  float y3 = (t + rf*cos(theta3))*sin30;
  float x3 = -y3*tan60;
  float z3 = -rf*sin(theta3);
   
  float dnm = (y2-y1)*x3-(y3-y1)*x2;
   
  float w1 = y1*y1 + z1*z1;
  float w2 = x2*x2 + y2*y2 + z2*z2;
  float w3 = x3*x3 + y3*y3 + z3*z3;
   
  // x = (a1*z + b1)/dnm
  float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
   
  // y = (a2*z + b2)/dnm;
  float a2 = -(z2-z1)*x3+(z3-z1)*x2;
  float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
   
  // a*z^2 + b*z + c = 0
  float a = a1*a1 + a2*a2 + dnm*dnm;
  float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
  // discriminant
  float d = b*b - (float)4.0*a*c;
  if (d < 0) { validPosition = false; }

  zp = -(float)0.5*(b+sqrt(d))/a;
  xp = (a1*zp + b1)/dnm;
  yp = (a2*zp + b2)/dnm;

  if (xp >= min_x && xp <= max_x&& yp >= min_y && yp <= max_y && zp >= min_z & zp <= max_z) {  //bounds check
  } else {
    validPosition = false;
  }
    
  if ( validPosition ) {   //we're good, make it so 
    SetServos();  
  } else {                 //we failed, revert to our last known good positions
    xp = last_x;
    yp = last_y;
    zp = last_z;
    t1 = last_t1;
    t2 = last_t2;
    t3 = last_t3;  
  }
    
  //finally update the gui elemnts
  updateGuiElements();
  if ( debug ) { println("Exiting: setXYZfromThetas()"); } 
}

//our stuff has changed, update the gui elements
void updateGuiElements() { 
  if ( debug ) { println("Entering: updateGuiElements()"); } 
  //turn off the event broadcasts to stop recursion
  xtext.setBroadcast(false);
  ytext.setBroadcast(false); 
  ztext.setBroadcast(false); 
  xy.setBroadcast(false); 
  z.setBroadcast(false); 
  servo1.setBroadcast(false);
  servo2.setBroadcast(false);
  servo3.setBroadcast(false);
 
  //apply values to gui elements
  xy.setArrayValue(new float[] {xp * -1 + 20, yp * -1 + 20});
  z.setValue(zp);
  xtext.setText(String.valueOf(Math.floor(xp * precision +.5)/precision));
  ytext.setText(String.valueOf(Math.floor(yp * precision +.5)/precision));
  ztext.setText(String.valueOf(Math.floor(zp * precision +.5)/precision));
  servo1.setValue(t1);
  servo2.setValue(t2);
  servo3.setValue(t3);
  
  //turn event broadcasts back on
  xtext.setBroadcast(true);
  ytext.setBroadcast(true); 
  ztext.setBroadcast(true); 
  xy.setBroadcast(true); 
  z.setBroadcast(true);
  servo1.setBroadcast(true);
  servo2.setBroadcast(true);
  servo3.setBroadcast(true);
  if ( debug ) { println("Exiting: updateGuiElements()"); }
}

//set servo positions
void SetServos() {
  if ( debug ) { println("Entering: SetServos()"); }
  if (servosEnabled) {
    sendMove(1, (int)t1 + 90 + s1offset);  //math is -90 to +90, servo is 0 to 180
    sendMove(2, (int)t2 + 90 + s2offset);  //math is -90 to +90, servo is 0 to 180
    sendMove(3, (int)t3 + 90 + s3offset);  //math is -90 to +90, servo is 0 to 180
  }
  if ( debug ) { println("Exiting: SetServos()"); }
}

//enable or disable servos
void enable(boolean btnEnabled) {
  if ( debug ) { println("Entering: enable() (servos)"); }
  if ( btnEnabled==true ) {
    sendMove(99, 180); //turn servos on
    setThetasfromXYZ(); //set servos to current x,y,z
    servosEnabled = true;
  } 
  else {
    sendMove(99, 0); //turn servos off
    servosEnabled = false;
  }
  if ( debug ) { println("Exiting: enable() (servos)"); }
}

//send serial command
void sendMove(int servo, int val) {
  myPort.write(255);
  myPort.write(servo);
  myPort.write(val);
}

//copy current xyz values into our failback aray
void  copyLastData() {
  if ( debug ) { println("Entering: copyLastData()"); }
  last_x = xp;
  last_y = yp;
  last_z = zp;
  last_t1 = t1;
  last_t2 = t2;
  last_t3 = t3;
  if ( debug ) { println("Exiting: copyLastData()"); }  
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
float delta_calcAngleYZ(float x0, float y0, float z0) {
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735    * e;    // shift center to edge
  // z = a + b*y
  float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
  float b = (y1-y0)/z0;
  // discriminant
  float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
  if (d < 0) return 999; // non-existing point
  float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
  float zj = a + b*yj;
  return 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
} 