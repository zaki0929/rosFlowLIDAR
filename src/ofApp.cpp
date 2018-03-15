#include "ofApp.h"

//Definition for using URG-04LX-UG01.
#define RANGE_MAX 5.6
#define VALID_MIN 44
#define VALID_MAX 725
#define INITIAL_ANGLE 2.618 

void ofApp::setup(){
  scan_sub = n.subscribe("scan", 1000, &ofApp::scanCallback, this);
  ofBackground(0, 0, 0);
  ofSetWindowShape(1280, 720);
  ofSetBackgroundAuto(false);

  fluidSimulation.setup(ofGetWidth(), ofGetHeight());
  fluidSimulation.setDissipation(0.0);

  mouseForces.setup(ofGetWidth(), ofGetHeight(),ofGetWidth(),ofGetHeight());
  lastTime = ofGetElapsedTimef();

  gui.setup();
  gui.add(mode.setup("Mode", 6, 0, 6));
  gui.add(toggleTrajectoryDraw.setup("Draw the trajectory", 1, 0, 1));
  gui.add(alpha.setup("Alpha", 255, 0, 255));

  gui2.setup();
  gui2.setPosition(10, 90);
  gui2.add(toggleRotate180.setup("Rotate 180 degrees", 0, 0, 1));

  gui3.setup();
  gui3.setPosition(10, 90);
  gui3.add(xCam.setup("The x coordinate", 300, 0, 800));
  gui3.add(yCam.setup("The y coordinate", 600, 0, 800));
  gui3.add(zCam.setup("The z coordinate", 200, 0, 800));

  cam.setDistance(800);
  cam.setPosition(ofVec3f(xCam, yCam, zCam));
  depth = 0;
}

double null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    //ROS_WARN("RANGE OVER");
  }
  return target;
}

void ofApp::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double center_number = (-msg->angle_min)/msg->angle_increment;

  center = null_check(msg->ranges[center_number]);
  left = null_check(msg->ranges[center_number+128]);
  angle_diff = msg->angle_increment;
  
  for(int i=0; i<msg->ranges.size(); i++){
    scanValues[i] = null_check(msg->ranges[i]);
  }
}

void ofApp::update(){
  deltaTime = ofGetElapsedTimef()-lastTime;
  lastTime = ofGetElapsedTimef();
  mouseForces.update(deltaTime);

  for (int i=0; i<mouseForces.getNumForces(); i++) {
    if(mouseForces.didChange(i)){
      fluidSimulation.addDensity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
      fluidSimulation.addVelocity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
    }
  }
  fluidSimulation.update();
  ros::spinOnce();
}

void ofApp::draw(){
  if(toggleTrajectoryDraw){
    ofSetColor(0, 0, 0, 30);
  }else{
    ofSetColor(0, 0, 0, 255);
  }
  ofRect(0, 0, ofGetWidth(), ofGetHeight());
  ofSetColor(255, 255, 255, alpha);
  switch(mode){
    case 0: drawFluid(); break;
    case 1: drawStar(); break;
    case 2: drawWave(); break;
    case 3: drawSun(); gui2.draw(); break;
    case 4: drawConstellation(); gui2.draw(); break;
    case 5: drawFirefly(); gui2.draw(); break;
    case 6: drawTerrain(); gui3.draw(); break;
  }
  gui.draw();
}

void ofApp::drawFluid(){
  if(center < 0.2){
    fluidSimulation.draw(0,0,1280,720);
  }
}

void ofApp::drawStar(){
  if(center < 0.5 || left < 0.5){
    if(center < 0.5) level_center = center * 1000;
    if(left < 0.5) level_left = left * 1000;
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);
    ofBeginShape();
    ofVertex(200+level_center,135+level_left);
    ofVertex(15+level_center,135+level_left);
    ofVertex(165+level_center,25+level_left);
    ofVertex(105+level_center,200+level_left);
    ofVertex(50+level_center,25+level_left);
    ofEndShape();
  }
}

void ofApp::drawWave(){
  ofSetPolyMode(OF_POLY_WINDING_NONZERO);
  ofBeginShape();
  ofVertex(VALID_MIN+280, RANGE_MAX*100+80);
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofVertex(i+280, scanValues[i]*100+80);
  }
  ofVertex(VALID_MAX+280, RANGE_MAX*100+80);
  ofEndShape();
}

void ofApp::drawSun(){
  double angle = INITIAL_ANGLE;
  if(toggleRotate180){
    angle = INITIAL_ANGLE + M_PI;
  }
  ofSetPolyMode(OF_POLY_WINDING_NONZERO);
  ofBeginShape();
  ofVertex(640, 360);
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofVertex(scanValues[i]*std::cos(angle)*60+640, -scanValues[i]*std::sin(angle)*60+360);
    angle += angle_diff;
  }
  ofVertex(640, 360);
  ofEndShape();
}

void ofApp::drawConstellation(){
  double angle = INITIAL_ANGLE;

  if(toggleRotate180){
    angle = INITIAL_ANGLE + M_PI;
  }
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofDrawCircle(scanValues[i]*std::cos(angle)*60+640, -scanValues[i]*std::sin(angle)*60+360, 0.5);
    angle += angle_diff;
  }
}

void ofApp::drawFirefly(){
  double min_value = RANGE_MAX;
  int min_i = VALID_MIN;
  double angle = INITIAL_ANGLE;

  if(toggleRotate180){
    angle = INITIAL_ANGLE + M_PI;
  }
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    if(min_value > scanValues[i] && scanValues[i] > 0.1){
      min_value = scanValues[i];
      min_i = i;
    }
  }
  for(int i=VALID_MIN; i<min_i; i++){
    angle += angle_diff;
  }
  ofDrawCircle(scanValues[min_i]*std::cos(angle)*1000+640, -scanValues[min_i]*std::sin(angle)*1000+360, 20);
}

void ofApp::drawTerrain(){
  mesh.addVertex(ofVec3f(VALID_MIN, RANGE_MAX*100, -depth));
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
   mesh.addVertex(ofVec3f(i, scanValues[i]*100, -depth));
   cam.setPosition(ofVec3f(xCam, yCam, zCam-depth));
   depth += 0.001;
  }
  mesh.addVertex(ofVec3f(VALID_MAX, RANGE_MAX*100, -depth));
  cam.begin();
  mesh.draw();
  cam.end();
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
  if(key == 'c'){
    fluidSimulation.reset();
  }
}
