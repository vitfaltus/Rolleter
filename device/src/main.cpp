
#include <Arduino.h>
#include <AS5600.h>
#include "Adafruit_VEML7700.h"
#include <vector>
#include <map>

#define driv_en D1
#define driv_ph D2
#define driv_nSleep D3
#define boost_en D8

#define Rbutton D9
#define Lbutton D10

enum motor_state{
  stopped,
  forwards,
  backwards
};

enum device_state{
  idle,
  homing,
  light_movement,
  manual_movement
};

struct state{
  int rotation;
  motor_state direction;
  device_state internal_state;
};

class Device {
  int current_rotation;
  std::map<std::string,int> positions;
  std::string last_home = "night_shut";
  //sensors
  AS5600 encoder;
  Adafruit_VEML7700 veml = Adafruit_VEML7700();
  void update_current_position();
  void home();

  const int slack_angle = 4; //precision of the position
  //timeout vars
  int home_counter = 0; // if > 50, device is set ot automatic mode(homing)
  int right_counter = 0; // if > 30, buttons can override automatic movement
  int left_counter = 0;
  //endpoints
  const int min_rotation = 200;
  const int max_rotation = 345;
  //motor commands
  void Forward();
  void Backward();
  void Stop();

  public:
    Device();
    int dev_index = 0; //changed by the controller
    device_state internal_state;
    state next_state; // desired state
    void check_buttons();
    void physical_movement();
    friend void setup();
    
};

//TODO -- network & controll of other devices (multithread)
class Controller{
  std::vector<Device> devices;
  
  bool check_network();
  public:
    int controller_index; //changed by the server
    void fetch_next_states();
    void deliver_next_states();

};

Device dev;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(driv_en, OUTPUT);
  pinMode(driv_nSleep, OUTPUT);
  pinMode(driv_ph, OUTPUT);
  pinMode(boost_en, OUTPUT);

  digitalWrite(boost_en, HIGH);
  digitalWrite(driv_en, HIGH);

  pinMode(Rbutton, INPUT);
  pinMode(Lbutton, INPUT);

  while(!dev.encoder.begin()){
    continue;
  }
  while (!dev.veml.begin()) {
    continue;
  }
}

void loop() {
  dev.check_buttons();
  dev.physical_movement();
}

Device::Device(){
  positions["night_shut"] = this->max_rotation;
  positions["day_shut"] = this->min_rotation;
  positions["straight"] = 290;
}

void Device::check_buttons(){
  int right_button = digitalRead(Rbutton);
  int left_button = digitalRead(Lbutton);
  if (right_button == HIGH && left_button == HIGH){
    if (next_state.internal_state != homing && home_counter > 50){
      home_counter=0;
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
      next_state.internal_state = manual_movement;
      next_state.rotation = max_rotation; // needs a bit of work
      next_state.direction = forwards;
    }
    
  }
  else if (left_button == HIGH){ //sets next_state movement to backwards
    if (left_counter < 100){
      left_counter++;
    }
    else{
      left_counter = 0;
      next_state.internal_state = manual_movement;
      next_state.rotation = min_rotation;
      next_state.direction = backwards;
    }
  }
  //the buttons aren't pressed and the controll is in device_state is manual controll, change it to idle and dir to stopped
  else if (next_state.internal_state == manual_movement){ 
    right_counter = 0;
    left_counter = 0;
    next_state.internal_state = idle;
    next_state.direction = stopped;
    next_state.rotation = current_rotation;
  }
}

void Device::physical_movement(){
  update_current_position();
  //wheel is at desired position
  if (abs(current_rotation-next_state.rotation) <= slack_angle){
    Stop();
    return;
  }
  //move the wheel in the next_state direction OR to end point if out of bounds
  if (next_state.direction == forwards || current_rotation < min_rotation) {
    Forward();
  }
  else if (next_state.direction == backwards || current_rotation > max_rotation){
    Backward();
  }
  else{
    Stop();
  }
}

void Device::update_current_position(){
  current_rotation = map(encoder.readAngle(), 0, 4095, 0 ,359);
}

void Device::home(){
  next_state.internal_state = homing;
  if (last_home == "night_shut"){
    last_home = "day_shut";
    next_state.direction = backwards;
  }
  else{
    last_home = "night_shut";
    next_state.direction = forwards;
  }
  next_state.rotation = positions[last_home];
}

void Device::Backward(){
  digitalWrite(driv_nSleep,HIGH);
  digitalWrite(driv_ph, LOW);
  digitalWrite(driv_en, HIGH);
}

void Device::Forward(){
  digitalWrite(driv_nSleep,HIGH);
  digitalWrite(driv_ph, HIGH);
  digitalWrite(driv_en, HIGH);
}

void Device::Stop(){
  digitalWrite(driv_nSleep,LOW);
  digitalWrite(driv_ph, LOW);
  digitalWrite(driv_en, LOW);
}