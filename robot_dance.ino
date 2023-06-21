#include <Servo.h>

constexpr int LEFT = 12;
constexpr int RIGHT = 13;
constexpr int BUTTON_PIN = 2;
constexpr int LED_PIN = 11;
constexpr int NUMBER_OF_SENSORS = 5;
constexpr int FIRST_PIN_OF_SENSORS = 3;
constexpr int SENSORS[] = { FIRST_PIN_OF_SENSORS, FIRST_PIN_OF_SENSORS + 1, FIRST_PIN_OF_SENSORS + 2, FIRST_PIN_OF_SENSORS + 3, FIRST_PIN_OF_SENSORS + 4 };
constexpr unsigned long button_delay = 250;
constexpr byte _ = -1;
constexpr int normal_speed = 70;
constexpr int sharp_turn = -20;
constexpr int small_turn = 10;

class motor : public Servo {
public:
  motor(void) {
    dir_ = 1;
  }

  void go(int percentage) {
    writeMicroseconds(1500 + dir_ * percentage * 2);
  }
  void setDirection(bool there) {
    if (there) {
      dir_ = 1;
    } else {
      dir_ = -1;
    }
  }

private:
  int dir_;
};

class Timer{
  private:
    unsigned long time_ = 0;
    unsigned long start_time_ = millis();  
  public:
    void updateTime(){
      time_ = millis() - start_time_;
    }
    unsigned long getTime() { return time_;}
    void resetTime(){
      start_time_ = millis();
    }
};

class Button {
public:
  Button(int pin)
    : pin_(pin) {
  }
  void setup(Timer *timer) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    previously_pressed_ = !digitalRead(pin_);
    timer_ = timer;
  }
  bool clicked() {
    bool pressed = !digitalRead(pin_);
    if (pressed) {
      Serial.println("IT NOT OK");
      if (!previously_pressed_) {
        if (timer_->getTime() - lastchange_ >= button_delay) {
          lastchange_ = timer_->getTime();
          previously_pressed_ = pressed;
          return true;
        }
      }
    }
    previously_pressed_ = pressed;
    return false;
  }
  bool isPressed(){
    return !digitalRead(pin_);    
  }
private:
  bool previously_pressed_;
  unsigned long lastchange_;
  int pin_;
  Timer* timer_ = nullptr;
};

struct position {
  byte x;
  byte y;
};

struct instruction{
public:
  position instruction_position;
  unsigned long time;
  bool row_first; //if true then i drive along a row first
};

enum class Direction { LEFT, UP, RIGHT, DOWN};

class Choreography{
public:
  position initial_position = {1, 1};
  Direction initial_direction = Direction::UP;
public:
  Choreography(){
    instructions_[0] = {{5, 1}, true};
    instructions_[1] = {{2, 2}, true};
    instructions_[2] = {{1, 3}, false};
    instructions_[3] = {{3, 4}, false};
    instructions_[4] = {{4, 2}, true};

  }
  bool moveNext(){
    ++index_;
    if (index_ > 4) return true;
    return false;
  }
  instruction getCurrentInstruction(){
    return instructions_[index_];
  }
  void LoadDanceInstructions(){
  }
private:
  int index_ = -1;
  instruction instructions_[5];
};

class Robot {
public:
  enum class RobotState { FORWARD, CROSSING, ROTATE, ROTATE_LEFT, ROTATE_RIGHT, WAIT, END};
public:
  Robot(void) {}
  void setup(Timer* timer, Button* button) {
    leftMotor_.attach(LEFT, 500, 2500);
    leftMotor_.setDirection(true);
    rightMotor_.attach(RIGHT, 500, 2500);
    rightMotor_.setDirection(false);
    timer_ = timer;
    button_ = button;
  }
  bool paused(){
    return move_;
  }
  void un_pause(){
    move_ = !move_;
  }
  void setSpeed(int left_wheel, int right_wheel){
    leftMotor_.go(left_wheel);
    rightMotor_.go(right_wheel);

  }
  void goForward(int speed) {
    setSpeed(speed, speed);
  }
  void stop(){
    setSpeed(0, 0);
  }
  void followForward(){
  if (isOnSensors_(_,1,0,0,_)) setSpeed(sharp_turn, normal_speed);
  else if (isOnSensors_(_,1,1,0,_)) setSpeed(small_turn, normal_speed);
  else if (isOnSensors_(_,0,1,0,_)) goForward(normal_speed);
  else if (isOnSensors_(_,0,1,1,_)) setSpeed(normal_speed, small_turn);
  else if (isOnSensors_(_,0,0,1,_)) setSpeed(normal_speed, sharp_turn);
  }  
  void dance() {
    if(paused()) {
      stop();
      return;
    }
    readSensors_();
    switch(state_){
      case RobotState::FORWARD:
        if ((sensors_[0] && sensors_[1]) || (sensors_[3] && sensors_[4])){
          state_ = RobotState::CROSSING;
        }
        goForward(normal_speed);
        break;
      case RobotState::CROSSING:
        switch(orientation_){
          case Direction::LEFT:
            robot_position_.x -= 1;
            if (robot_position_.x == current_instruction_.instruction_position.x){
              if (robot_position_.y == current_instruction_.instruction_position.y)
                state_ = RobotState::WAIT;
              else if (robot_position_.y < current_instruction_.instruction_position.y)
                state_ = RobotState::ROTATE_RIGHT;
                else state_ = RobotState::ROTATE_LEFT;
            }
            break;
          case Direction::UP:
            robot_position_.y += 1;
            if (robot_position_.y == current_instruction_.instruction_position.y){
              if (robot_position_.x == current_instruction_.instruction_position.x)
                state_ = RobotState::WAIT;
              else if (robot_position_.x < current_instruction_.instruction_position.x)
                state_ = RobotState::ROTATE_RIGHT;
                else state_ = RobotState::ROTATE_LEFT;
            }
            break;
          case Direction::RIGHT:
            robot_position_.x += 1;
            if (robot_position_.x == current_instruction_.instruction_position.x){
             if (robot_position_.y == current_instruction_.instruction_position.y)
                state_ = RobotState::WAIT;
              else if (robot_position_.y < current_instruction_.instruction_position.y)
                state_ = RobotState::ROTATE_LEFT;
                else state_ = RobotState::ROTATE_RIGHT;
            }
            break;
          case Direction::DOWN:
            robot_position_.y -= 1;
            if (robot_position_.y == current_instruction_.instruction_position.y){
              if (robot_position_.x == current_instruction_.instruction_position.x)
                state_ = RobotState::WAIT;
              else if (robot_position_.x < current_instruction_.instruction_position.x)
                state_ = RobotState::ROTATE_LEFT;
                else state_ = RobotState::ROTATE_RIGHT;
            }
            break;
        }
      case RobotState::ROTATE:
        if (!isOnSensors_(0,0,1,0,0))
          setSpeed(sharp_turn, normal_speed);
        else state_ = RobotState::ROTATE_LEFT;
        break;
      case RobotState::ROTATE_LEFT:
        if (!isOnSensors_(0,0,1,0,0))
          setSpeed(sharp_turn, normal_speed);
        else state_ = RobotState::FORWARD;
        break;        
      case RobotState::ROTATE_RIGHT:
          if (!isOnSensors_(0,0,1,0,0))
            setSpeed(normal_speed, sharp_turn);
        else state_ = RobotState::FORWARD;
        break;
      case RobotState::WAIT:
        if (current_instruction_.time < timer_->getTime()){
          stop();
          break;
        }
        if (!robot_dance_.moveNext()){
          state_ = RobotState::END;
          stop();
          break;
        }
        current_instruction_ = robot_dance_.getCurrentInstruction();
        rotateCorrectly_();
        break;
      case RobotState::END:
        break;        
      default:
        break;
    }
  }
  void loadInstructions(){
    robot_dance_.LoadDanceInstructions();
    robot_position_ = robot_dance_.initial_position;
    orientation_ = robot_dance_.initial_direction;
    current_instruction_ = robot_dance_.getCurrentInstruction();
  }
private:
  int calculateXDifference_(){
    return currecnt_instruction_.instruction_position.x - robot_position_.x;
  }
  int calculateYDifference_(){
    return currecnt_instruction_.instruction_position.y - robot_position_.y;
  }
  Direction directionOfNextMove(){
    if (currecnt_instruction_.row_first) {
      if (calculateXDifference_() > 0) return Direction::RIGHT;
      else if (calculateXDifference_() < 0) return Direction::LEFT;
    }
    if (calculateYDifference_() > 0) return Direction::UP;
      else if (calculateXDifference_() < 0) return Direction::DOWN;
    return orientation_;
  }
  void rotateCorrectly_(){
    //TODO
    Direction new_direction = directionOfNextMove();    
    switch(orientation_){
      case Direction::LEFT:
        break;
      case Direction::UP:
        break;
      case Direction::RIGHT:
        break;
      case Direction::DOWN:
        break;
    }
  }
  void readSensors_(){
    for (short i = 0; i < 5; ++i){
      sensors_[i] = !digitalRead(SENSORS[i]);
    }
  }
  bool isOnSensors_(byte sensor0, byte sensor1, byte sensor2, byte sensor3, byte sensor4){
    if (sensor0 != _ && sensors_[0] != sensor0) return false;
    if (sensor1 != _ && sensors_[1] != sensor1) return false;
    if (sensor2 != _ && sensors_[2] != sensor2) return false;
    if (sensor3 != _ && sensors_[3] != sensor3) return false;
    if (sensor4 != _ && sensors_[4] != sensor4) return false;
    return true;
  } 

  bool sensors_[5] = {false, false, false, false, false};
  bool move_ = false;

  motor leftMotor_;
  motor rightMotor_;

  Choreography robot_dance_ = Choreography();
  position robot_position_ = robot_dance_.initial_position;
  Direction orientation_ = Direction::UP;
  RobotState state_ = RobotState::WAIT;
  Timer *timer_ = nullptr;
  Button *button_ = nullptr;
  instruction current_instruction_ = robot_dance_.getCurrentInstruction();

};

Timer timer;
Button button(BUTTON_PIN);

Robot robot;

void setupPins() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = FIRST_PIN_OF_SENSORS; i < NUMBER_OF_SENSORS + FIRST_PIN_OF_SENSORS; ++i) {
    pinMode(i, INPUT_PULLUP);
  }
}

void setup() {
  Serial.begin(9600);
  button.setup(&timer);
  robot.setup(&timer, &button);
  setupPins();
  while (!button.clicked()){
    timer.updateTime();
    robot.loadInstructions();
  }
  timer.resetTime();
}

void loop() {
  timer.updateTime();
  if (button.clicked()) {
    timer.resetTime();
    robot.un_pause();
  }
  robot.dance();
}
