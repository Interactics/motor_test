/*
 * motor_node.cpp
 *
 *      Author: arirang2067
 */
#include <ros/ros.h>
#include <pigpiod_if2.h>

#define motor_DIR1 26
#define motor_PWM1 12
#define motor_ENA1 6
#define motor_DIR2 19
#define motor_PWM2 13
#define motor_ENA2 22

using namespace std;
int PWM_limit;
void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
volatile int EncoderCounter1;
volatile int EncoderSpeedCounter1;
volatile int EncoderCounter2;
volatile int EncoderSpeedCounter2;
bool switch_direction;
int Theta_Distance_Flag;


class DcMotorForRaspberryPi
{
private:

public:
  int pinum;
  int motor_ENA;
  int motor_DIR;
  int motor_PWM;
  int PWM_range;
  int PWM_frequency;
  int current_PWM;
  bool current_Direction;
  int acceleration;
  void Motor_Controller(bool direction, int pwm);
  void Accel_Controller(bool direction, int desired_pwm);
  DcMotorForRaspberryPi(){}
  DcMotorForRaspberryPi(int motor_dir, int motor_pwm, int motor_ena)
  {
    pinum=pigpio_start(NULL, NULL);
    if(pinum<0)
    {
      ROS_INFO("Setup failed");
      ROS_INFO("pinum is %d", pinum);
    }
    motor_DIR = motor_dir;
    motor_PWM = motor_pwm;
    motor_ENA = motor_ena;
    PWM_range = 512;
    PWM_frequency = 40000; 

    set_mode(pinum, motor_dir, PI_OUTPUT);
    set_mode(pinum, motor_pwm, PI_OUTPUT);
    set_mode(pinum, motor_ena, PI_INPUT);

    set_PWM_range(pinum, motor_pwm, PWM_range);
    set_PWM_frequency(pinum, motor_pwm, PWM_frequency);
    gpio_write(pinum, motor_DIR, PI_LOW);
    set_PWM_dutycycle(pinum, motor_PWM, 0);
    
    current_PWM = 0;
    current_Direction = true;
    acceleration = 5;
    ROS_INFO("Setup Fin");
  }
};
void DcMotorForRaspberryPi::Motor_Controller(bool direction, int pwm)
{
  if(direction == true) //CW
  {
    gpio_write(pinum, motor_DIR, PI_LOW);
    set_PWM_dutycycle(pinum, motor_PWM, pwm);
    current_PWM = pwm;
    current_Direction = true;
  }
  else //CCW
  {
    gpio_write(pinum, motor_DIR, PI_HIGH);
    set_PWM_dutycycle(pinum, motor_PWM, pwm);
    current_PWM = pwm;
    current_Direction = false;
  }
}
void DcMotorForRaspberryPi::Accel_Controller(bool direction, int desired_pwm)
{
  int local_PWM;
  if(desired_pwm > current_PWM)
  {
    local_PWM = current_PWM + acceleration;
    Motor_Controller(direction, local_PWM);
  }
  else if(desired_pwm < current_PWM)
  {
    local_PWM = current_PWM - acceleration;
    Motor_Controller(direction, local_PWM);
  }
  else
  {
    local_PWM = current_PWM;
    Motor_Controller(direction, local_PWM);
  }
  //ROS_INFO("Current_PWM is %d", current_PWM);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
DcMotorForRaspberryPi motor1 = DcMotorForRaspberryPi(motor_DIR1, motor_PWM1, motor_ENA1);
DcMotorForRaspberryPi motor2 = DcMotorForRaspberryPi(motor_DIR2, motor_PWM2, motor_ENA2);

void Initialize()
{
  PWM_limit = 150;
  EncoderCounter1 = 0;
  EncoderSpeedCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderSpeedCounter2 = 0;
  callback(motor1.pinum, motor1.motor_ENA, FALLING_EDGE, Interrupt1);
  callback(motor1.pinum, motor2.motor_ENA, FALLING_EDGE, Interrupt2);

  switch_direction = true;
  Theta_Distance_Flag = 0;
  ROS_INFO("Initialize Complete");
}
void Interrupt1(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  EncoderCounter1 ++;
  EncoderSpeedCounter1 ++;
  //ROS_INFO("Interrupt1 is %d", EncoderCounter1);
}
void Interrupt2(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  EncoderCounter2 ++;
  EncoderSpeedCounter2 ++;
  //ROS_INFO("Interrupt2 is %d", EncoderCounter2);
}

int Limit_Function(int pwm)
{
  int output;
  if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)output = 0;
  else output = pwm;
  return output; 
}

void Switch_Turn_Example(int PWM1, int PWM2)
{
  int local_PWM1 = Limit_Function(PWM1);
  int local_PWM2 = Limit_Function(PWM2);
  if(switch_direction == true)
  {
    motor1.Motor_Controller(switch_direction, local_PWM1);
    motor2.Motor_Controller(switch_direction, local_PWM2);
    switch_direction = false;
    ROS_INFO("true");
  }
  else
  {
    motor1.Motor_Controller(switch_direction, local_PWM1);
    motor2.Motor_Controller(switch_direction, local_PWM2);
    switch_direction = true;
    ROS_INFO("false");
  }
  ROS_INFO("Encoder A1 is %d", EncoderCounter1);
  ROS_INFO("Encoder A2 is %d", EncoderCounter2);
}

void Theta_Turn(float Theta, int PWM)
{
  double local_encoder;
  int local_PWM = Limit_Function(PWM);
  if(Theta_Distance_Flag == 1)
  {
      EncoderCounter1 = 0;
      EncoderCounter2 = 0;
      Theta_Distance_Flag = 2;
  }
  if(Theta > 0)
  {
    local_encoder = Theta; //(Round_Encoder/360)*(Robot_Round/Wheel_Round)
    motor1.Motor_Controller(true, local_PWM);
    motor2.Motor_Controller(true, local_PWM);
    //motor1.Accel_Controller(true, local_PWM);
    //motor2.Accel_Controller(true, local_PWM);
  }
  else
  {
    local_encoder = -Theta; //(Round_Encoder/360)*(Robot_Round/Wheel_Round)
    motor1.Motor_Controller(false, local_PWM);
    motor2.Motor_Controller(false, local_PWM);
    //motor1.Accel_Controller(false, local_PWM);
    //motor2.Accel_Controller(false, local_PWM);
  }

  if(EncoderCounter1 > local_encoder)
  {
    //ROS_INFO("Encoder A1 is %d", EncoderCounter1);
    //ROS_INFO("Encoder A2 is %d", EncoderCounter2);
    EncoderCounter1 = 0;
    EncoderCounter2 = 0;
    motor1.Motor_Controller(true, 0);
    motor2.Motor_Controller(true, 0);
    //motor1.Motor_Controller(true, 0);
    //motor2.Motor_Controller(true, 0);
    Theta_Distance_Flag = 3;
  }
}
void Distance_Go(float Distance, int PWM)
{
  float local_encoder = Distance; //(Round_Encoder*Distance)/Wheel_Round
  int local_PWM = Limit_Function(PWM);
  bool Direction = 1;
  if(Distance < 0)
  {
    Direction = 0;
    local_encoder = -local_encoder;
  }
  if(Theta_Distance_Flag == 3)
  {
      EncoderCounter1 = 0;
      EncoderCounter2 = 0;
      Theta_Distance_Flag = 4;
  }

  if(EncoderCounter1 < local_encoder)
  {
    if(Direction==1)
    {
      motor1.Motor_Controller(false, local_PWM);
      motor2.Motor_Controller(true, local_PWM);
      //motor1.Accel_Controller(false, local_PWM);
      //motor2.Accel_Controller(true, local_PWM);
    }
    else
    {
      motor1.Motor_Controller(true, local_PWM);
      motor2.Motor_Controller(false, local_PWM);
      //motor1.Motor_Controller(true, local_PWM);
      //motor2.Motor_Controller(false, local_PWM);
    }
  }
  else
  {
    //ROS_INFO("Encoder A1 is %d", EncoderCounter1);
    //ROS_INFO("Encoder A2 is %d", EncoderCounter2);
    EncoderCounter1 = 0;
    EncoderCounter2 = 0;
    motor1.Motor_Controller(true, 0);
    motor2.Motor_Controller(true, 0);
    //motor1.Accel_Controller(true, 0);
    //motor2.Accel_Controller(true, 0);
    Theta_Distance_Flag = 0;
  }
}
void Theta_Distance(float Theta, int Turn_PWM, float Distance, int Go_PWM)
{
  if(Theta_Distance_Flag == 0)
  {
    Theta_Distance_Flag = 1;
  }
  else if(Theta_Distance_Flag == 1 || Theta_Distance_Flag == 2)
  {
    Theta_Turn(Theta, Turn_PWM);
  }
  else if(Theta_Distance_Flag == 3 || Theta_Distance_Flag == 4)
  {
    Distance_Go(Distance, Go_PWM);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //Switch_Turn_Example(100, 100);
    //Theta_Distance(400,100,-200,110);
    //motor1.Accel_Controller(false, 100);
    //motor2.Accel_Controller(true, 100);
    ros::spinOnce();
    loop_rate.sleep();
  }
  motor1.Motor_Controller(true, 0);
  motor2.Motor_Controller(true, 0);
  return 0;
}
