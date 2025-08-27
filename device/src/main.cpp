/*
the main loop consists of fetch-execute cycle:
  FETCH PART
    -state of the next_state struct is set according the user input (in case of the autolight mode, the light setup is exercised)
    
  
  EXECUTE PART
    -the device executes instructions in the next_state
    -the execution section also checks all the rules such as max/min angle 

*/


#include <Arduino.h>
#include <AS5600.h>
#include "Adafruit_VEML7700.h"
#include <memory> //shared_ptr

#define driv_en D1
#define driv_ph D2
#define driv_nSleep D3
#define boost_en D8

#define Rbutton D9
#define Lbutton D10

enum motor_states{
  stopped,
  backwards,
  forwards
};

enum device_states{
  idle,
  homing,
  light_movement,
  manual_movement
};

struct position
{
  position(){
    angle = 0;
    rotations = 0;
  }
  position(int a, int b): angle(a), rotations(b){}
  bool operator==(const position& other) const{
    if (angle == other.angle && rotations == other.rotations){
      return true;
    }
    return false;
  }

  int angle;
  int rotations;
};

struct state{
  position desired_position;
  motor_states direction;
};

void Forward();
void Backward();
void Stop();

class Device{
  int device_index;
  position min_position = position(200,0);
  position current_position;
  position max_position = position(345,0);
  state next_state;
  device_states device_state = manual_movement;
  std::shared_ptr<position> last_home_position = std::make_shared<position>(min_position);
  const int slack_angle = 4;
  //these values prevent premature actions (since the loop basically runs on the proc. freq.)
  int home_counter = 10; // if > 10, device is set ot automatic mode(homing)
  int right_counter = 0; // if > 30, buttons can override automatic movement
  int left_counter = right_counter; 
  
  public:
    Device() = default;
    // main logic solver
    void fetch_next_state();
    //executes actions according the next state struct
    void execute_next_state();
    //sets next_state pos to min/max pos
    void home();
    //reads value from AS5600 and calculates current position, saves it to the current position var
    void update_current_position();
    //debug func
    void Serial_next_state(){
      Serial.print(current_position.angle);
      Serial.print(" ");
      Serial.print(next_state.direction);
      Serial.print(" ");
      Serial.println(device_state);
    }
};


AS5600 encoder;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Device rolleter;

void setup() {
  pinMode(driv_en, OUTPUT);
  pinMode(driv_nSleep, OUTPUT);
  pinMode(driv_ph, OUTPUT);
  pinMode(boost_en, OUTPUT);
  
  digitalWrite(boost_en, HIGH);
  digitalWrite(driv_en, HIGH);

  pinMode(Rbutton, INPUT);
  pinMode(Lbutton, INPUT);

  Serial.begin(115200);
  Wire.begin();
  //waiting for sensors
  while(!encoder.begin()){
    continue;
  }
  while (!veml.begin()) {
    continue;
  }
}

void loop() {
  rolleter.update_current_position();
  rolleter.fetch_next_state();
  rolleter.execute_next_state();
}

void Device::fetch_next_state(){
  
  int right_button = digitalRead(Rbutton);
  int left_button = digitalRead(Lbutton);
  if (right_button == HIGH && left_button == HIGH){
    if (device_state != homing && home_counter > 50){
      home_counter=0;
      device_state = homing;
      home();
    }
    else{
      home_counter++;
    }
  }
  else if (right_button == HIGH){ //set next_state movement to forwards
    if (right_counter < 100){
      right_counter++;
    }
    else{
      right_counter = 0;
      device_state = manual_movement;
      next_state.desired_position = max_position;
      next_state.direction = forwards;
    }
    
  }
  else if (left_button == HIGH){ //sets next_state movement to backwards
    if (left_counter < 100){
      left_counter++;
    }
    else{
      left_counter = 0;
      device_state = manual_movement;
      next_state.desired_position = min_position;
      next_state.direction = backwards;
    }
  }
  //the buttons aren't pressed and the device is in device_state is manual controll, change it to idle and dir to stopped
  else if (device_state == manual_movement){ 
    right_counter = 0;
    left_counter = 0;
    device_state = idle;
    next_state.direction = stopped;
    next_state.desired_position = current_position;
  }


}

void Device::execute_next_state(){
  //wheel is at desired position
  if (abs(current_position.angle-next_state.desired_position.angle) <= slack_angle){
    Stop();
    return;
  }
  //move the wheel in the next_state direction
  if (next_state.direction == forwards) {
    Forward();
  }
  else if (next_state.direction == backwards){
    Backward();
  }
  else{
    Stop();
  }
  
}

void Device::home(){
  device_state = homing;
  //set next_state position according to the last homing position
  if (*last_home_position == min_position){
    last_home_position  = std::make_shared<position>(max_position);
    next_state.direction = forwards;
    next_state.desired_position = max_position;
  }
  else{
    last_home_position  = std::make_shared<position>(min_position);
    next_state.direction = backwards;
    next_state.desired_position = min_position;
  }
}

void Device::update_current_position(){
  current_position.angle = map(encoder.readAngle(), 0, 4095, 0 ,359);
}

void Backward(){
  digitalWrite(driv_nSleep,HIGH);
  digitalWrite(driv_ph, LOW);
  digitalWrite(driv_en, HIGH);
}

void Forward(){
  digitalWrite(driv_nSleep,HIGH);
  digitalWrite(driv_ph, HIGH);
  digitalWrite(driv_en, HIGH);
}

void Stop(){
  digitalWrite(driv_nSleep,LOW);
  digitalWrite(driv_ph, LOW);
  digitalWrite(driv_en, LOW);
}