import processing.serial.*;
Serial port;
import saito.objloader.*;
OBJModel model;

// If you use actual objects: true.
boolean useSerial = false;
// If you output movie: true.
boolean movie = true;

String inBuffer="";
String displayedText="  ";

// Roll, Pitch, Yaw(text)
String regTextR="";
String regTextP="";
String regTextY="";

// P, Q, R
float[] pqr    = {0,0,0};
float[] pqrDot = {0,0,0};

// Euler angle
float[] eulerDot = {0,0,0};
float[] euler    = {10,30,0};

// Position
float[] x     = {0,0,0};
float[] xDot  = {0,0,0};
float[] x2Dot = {0,0,0};

// Torque
float[] torqueReq = {0,0,0};
float[] torque    = {0,0,0};

// Thrust
float[] thrust = {0,0,0,0};

// length[m] of arms
float lArm = 0.055;

// Inertial params.
float m = 0.030; // weight[kg]
float[][] Inertial = { // moment of inertia[kg m^2]
{2.42*0.001,0,0},
{0,2.42*0.001,0},
{0,0,4.84*0.001}
};
float [][] InertialInv = matInv3(Inertial);

// For Controlling graph
float rotX = 0;
float rotY = 0;

float time = 0;
float dt = 0.05;

void setup(){
  if(useSerial){
    String ESPPort = Serial.list()[0];
    port = new Serial(this, ESPPort, 115200);
  }
  
  frameRate(1/dt);
  size(600, 600, P3D); 
  model = new OBJModel(this,"ESPDrone.obj","absolute",TRIANGLES);
  model.scale(3.0);
}
 
void draw(){
  println("FrameRate: " + frameRate);
 
  background(255);
  lights();
  translate(width/2, height/2);
  rotateX(radians(90)+rotX);
  rotateY(radians(0)+rotY);
  rotateZ(radians(10));
  
// Coordinate axes
  //stroke(255,0,0);
  line(0,0,0,500,0,0);
  line(0,0,0,0,500,0);
  line(0,0,0,0,0,-500);
  
// Draw 3D model and the input of each propeller
  pushMatrix();
  rotateZ(radians( euler[0]));
  rotateY(radians(-euler[1]));
  rotateX(radians( euler[2])); 
  
  model.draw();

  line( 150,0,20, 150,0,thrust[0]*1000+20);  // Front
  line(0,-150,20,0,-150,thrust[1]*1000+20);  // Left
  line(-150,0,20,-150,0,thrust[2]*1000+20);  // Rear
  line(0, 150,20,0, 150,thrust[3]*1000+20);  // Right
  line(0,0,20,400,0,20);
  popMatrix();
  
// Draw serial input
  pushMatrix();
  text(regTextR,0,90);
  text(regTextP,0,110);
  text(displayedText,0,70);
  popMatrix();
  
  system();
  
  sim();
  
  
  if(movie){
    saveFrame("frames/######.tif");
  }
}

void serialEvent(Serial p){
  //delay(10);
  inBuffer = port.readStringUntil(10);
}

// Thrust allocation
float[][] lagrange = {
  {0,1/(2*lArm)},     // Front
  {1/(2*lArm),0},     // Left
  {0,-1/(2*lArm)},    // Rear
  {-1/(2*lArm),0}     // Right
};

// Write control system here.
void system(){
  float[] refs      = {0,0,0};
  float[] refsDot   = {0,0,0};
  //clAttitudePControl(refs);
  clAttitudePDControl(refs,refsDot,3,0.6);
  ta(torque);
}

// Thrust allocation Law
void ta(float[] torque){
  println("Torque: " + torque[0] + ", " + torque[1]+", " + torque[2]);
  for(int i=0;i<4;i++){
    thrust[i] = lagrange[i][0]*torqueReq[0]+lagrange[i][1]*torqueReq[1];
  }
  println("thrust:" + thrust[0]* +','+ thrust[1]+','+ thrust[2]+','+ thrust[3]);
}

// Control Law: P  control for attitude angle
void clAttitudePControl(float[] ref){
  for(int i=0;i<3;i++){
   torqueReq[i] = sin(radians(ref[i] - euler[i])); 
  }
}

// Control Law: PD control for attitude angle
void clAttitudePDControl(float[] ref, float[] refDot, float omega, float zeta){
  for(int i=0;i<3;i++){
   torqueReq[i] = pow(omega,2) * sin(radians(ref[i] - euler[i])) + 2*omega*zeta* radians(refDot[i] - pqr[i]); 
  }
  torqueReq = matByVec(Inertial, torqueReq);
}

void mouseDragged(){
  rotX += (mouseX - pmouseX)* 0.01;
  rotY -= (mouseY - pmouseY)* 0.01;
}

// Simulation
void sim(){
  
  if(useSerial){
    port.write("thrust:" + thrust[0]* +','+ thrust[1]+','+ thrust[2]+','+ thrust[3]);
  }
  
  time += dt;
  updatePQR();
  updateEuler();

  updateTorque();
  updatePQRDot();
  updateEulerDot();
  
  println("Time:" + time);
  println("Angle      :" + euler[0] + ", " + euler[1] + ", " + euler[2]);
  println("Angular Vel:" + pqr[0] + ", " + pqr[1] + ", " + pqr[2]);
}

void updatePQR(){
  if(useSerial){
    if(inBuffer !=null){
      println(inBuffer);
      try{
        String[] m = match(inBuffer, "P/Q/R: (.*?), (.*?), (.*?)$");
        if(m != null){
          regTextR = m[1];
          regTextP = m[2];
          regTextY = m[3];
          
          pqr[0] = Float.parseFloat(regTextR);
          pqr[1] = Float.parseFloat(regTextP);
          pqr[2] = Float.parseFloat(regTextY);
          
          displayedText = inBuffer;
        }
      }catch(NullPointerException e){}
    }
  }else {
    for(int i=0;i<3;i++){
      pqr[i] += pqrDot[i] * dt;
    }
  }
}

void updateEuler(){
  if(useSerial){
    if(inBuffer !=null){
      println(inBuffer);
      try{
        String[] m = match(inBuffer, "R/P/Y: (.*?), (.*?), (.*?)$");
        if(m != null){
          regTextR = m[1];
          regTextP = m[2];
          regTextY = m[3];
          
          euler[0] = Float.parseFloat(regTextR);
          euler[1] = Float.parseFloat(regTextP);
          euler[2] = Float.parseFloat(regTextY);
          
          displayedText = inBuffer;
        }
      } catch(Exception e) {}
    }
  } else {
    for(int i=0;i<3;i++){
      euler[i] += eulerDot[i] * dt;
    }
  }  
}

void updateXDot(){
}

void updateX(){
}

void updateTorque(){
  torque[0] = lArm * thrust[1] - lArm * thrust[3];
  torque[1] = lArm * thrust[0] - lArm * thrust[2];
  torque[2] = 0;
}

void updatePQRDot(){
  pqrDot = matByVec(InertialInv, torque);
}

void updateEulerDot(){
  eulerDot[0] = pqr[0] + pqr[1] * sin(radians(euler[0])) * tan(radians(euler[1])) + pqr[2] * cos(radians(euler[0])) * tan(radians(euler[1]));
  eulerDot[1] = pqr[1] * cos(radians(euler[0])) - pqr[2] * sin(radians(euler[0]));
  eulerDot[2] = pqr[1] * sin(radians(euler[0])) / cos(radians(euler[1])) + pqr[2] * cos(radians(euler[0])) / cos(radians(euler[1]));
  //eulerDot[0] = pqr[0];
  //eulerDot[1] = pqr[1];
  //eulerDot[2] = pqr[2];
}

void updateX2Dot(){
  
}

void updateDCME2B(){}


// Math function

// get 3 dimensional inverse matrix
float[][] matInv3(float[][] mat){
  float det = 
  mat[0][0]*(mat[1][1]*mat[2][2]-mat[2][1]*mat[1][2])
  -mat[0][1]*(mat[1][0]*mat[2][2]-mat[2][0]*mat[1][2])
  +mat[0][2]*(mat[1][0]*mat[2][1]-mat[2][0]*mat[1][1]);
  
  if(det!=0){
    float[][] inv=
      {
        {mat[1][1]*mat[2][2]-mat[1][2]*mat[2][1],mat[0][2]*mat[2][1]-mat[0][1]*mat[2][2],mat[0][1]*mat[1][2]-mat[0][2]*mat[1][1]},
        {mat[1][2]*mat[2][0]-mat[1][0]*mat[2][2],mat[0][0]*mat[2][2]-mat[0][2]*mat[2][0],mat[0][2]*mat[1][0]-mat[0][0]*mat[1][2]},
        {mat[1][0]*mat[2][1]-mat[1][1]*mat[2][0],mat[0][1]*mat[2][0]-mat[0][0]*mat[2][1],mat[0][0]*mat[1][1]-mat[0][1]*mat[1][0]}
      };
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        inv[i][j]/=det;
      }
    }
    return inv;
  }else{
    return mat;
  }
}

// matrix 
float[] matByVec(float[][] mat, float[] vec){
  float[] out = {0,0,0};
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      out[i] += mat[i][j]*vec[j];
    }
  }
  return out;
}
