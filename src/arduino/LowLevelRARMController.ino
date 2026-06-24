#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Connect PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Pulse width for SG90s and all other servos
#define S_MIN 500
#define S_MAX 2500

#define SG_MIN 500
#define SG_MAX 2400

// Servo Channels ||  0 = J0, 1 = J1, 2 = J2, 3 = J3, 4 = J4, 5 = J5, 6 = J6
const int servoPin[] = {0, 1, 2, 3, 4, 5, 6};
double servoAngles[] = {0, 0, 0, 0, 0};
double currAngles[7];

// Convert raw angles to the equivalent angle for the servo range
void convertToRead(double angles[5]){
  for (int i = 0; i < 5; i++){
    if (i == 0 ){
      angles[i] = angles[i] + 135;
    }
    else if (i == 1){
      angles[i] = 85.0 - (8.0/9.0) * angles[i];
    }
    else if (i == 2){
      angles[i] = -10.0 + angles[i] * (171.0 / 180.0);
      angles[i] += 135.0;
    }
    else {
      angles[i] = angles[i] + 90;
    }
  }
}

// Move servo on desired channel to desired angle (channel 0 & 2 are set to 270 deg, can be adjusted as necessary)
void setServoAngle(double angle, int channel){
  double pulse_ti;
  if (channel == 0 || channel == 2){
    pulse_ti =  ((S_MAX - S_MIN) * (angle/270.0) + S_MIN);
  }
  else {
    pulse_ti =  ((S_MAX - S_MIN) * (angle/180.0) + S_MIN);
  }
  pulse_ti =  pulse_ti * 4096 / 20000;
  int p_ticks = round(pulse_ti);
  pwm.setPWM(channel, 0, p_ticks);
  currAngles[channel] = angle;
}

/*moveToGoal moves the entire robot (all five solvable joints) to angles set by an array.
t_steps configures how many steps the robot will take to move to the desired angle. A higher number will result in the robot moving slower.
The movement sequence is as such (Shoulder + Elbow ----> Forearm rotation + Wrist -----> Base)*/
void moveToGoal(double angles[5]){
  double moveSteps[5];
  int t_steps = 70;
  
  for (int i = 0; i < 5; i++){
    moveSteps[i] = (angles[i] - currAngles[i]) / t_steps;
  }
  
  for (int i = 0; i < t_steps; i++){
    for (int j = 1; j < 3; j++){
      setServoAngle((currAngles[j] + moveSteps[j]), servoPin[j]);
    }
    delay(25);
  }

  for (int i = 0; i < t_steps; i++){
    for (int j = 3; j < 5; j++){
      setServoAngle((currAngles[j] + moveSteps[j]), servoPin[j]);
    }
    delay(25);
  }

  for (int i = 0; i < t_steps; i++){
    for (int j = 0; j < 1; j++){
      setServoAngle((currAngles[j] + moveSteps[j]), servoPin[j]);
    }
    delay(25);
  }
}
/*Receive angles from the serial monitor. The string for angles should be as such "1,2,3,4,5".*/
bool receiveAngles(String serialData, double* angles){
  int comms[4];
  comms[0] = serialData.indexOf(',');
  for (int i = 1; i < 4; i++){
    comms[i] = serialData.indexOf(',', comms[i-1] + 1);
  }
  angles[0] = serialData.substring(0, comms[0]).toFloat();
  for (int i = 1; i < 4; i++){
    angles[i] = serialData.substring(comms[i-1] + 1, comms[i]).toFloat();
  }
  angles[4] = serialData.substring(comms[3] + 1).toFloat();
  convertToRead(angles);

  if (comms[0] == -1 ||
      comms[1] == -1 ||
      comms[2] == -1 ||
      comms[3] == -1){
        return false;
      }

  return true;
}

// Utilizes specific MAX/MIN values for the SG90 servos
void setServoAngleSG(double angle, int channel = 5){
  double pulse_ti;
  pulse_ti =  ((SG_MAX - SG_MIN) * (angle/180.0) + SG_MIN);
  pulse_ti =  pulse_ti * 4096 / 20000;
  int p_ticks = round(pulse_ti);
  pwm.setPWM(channel, 0, p_ticks);
  currAngles[channel] = angle;
}



void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(2000);

  //Homing location for the robot
  double homeAngles[] = {0, -50, 135, 90, -30};
  convertToRead(homeAngles);


  for (int i = 0; i < 5; i++){
    setServoAngle(homeAngles[i], servoPin[i]);
    currAngles[i] = homeAngles[i];
    delay(2000);
  }

  setServoAngle(90, 6);
  delay(1000);
  setServoAngleSG(90, 5);
// Serial notification that the robot has finished its boot sequence and can now be commanded to positions
  Serial.println("BOOTED");
}

void loop() {
  if (Serial.available() > 0){
    String data = Serial.readStringUntil('\n');
    data.trim();

// Gripper controller
    if (data.startsWith("GRIP,")){
      double ang =  data.substring(5).toFloat();
      setServoAngle(ang, 6);
    }
//Wrist rotation controller
    else if (data.startsWith("WRIST,")){
      double ang =  data.substring(6).toFloat();
      setServoAngleSG(ang, 5);
    }




    else if(receiveAngles(data, servoAngles)){
      moveToGoal(servoAngles);
      Serial.println("OK");
    }
    else{
      Serial.println("ERROR: Incorrect Format!");
    }
  }
}
