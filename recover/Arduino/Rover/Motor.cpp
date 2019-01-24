#include "motor.h"

Motor::Motor(Adafruit_DCMotor ref)
{
  m_reference = ref;
  m_error = 0;
  m_deltaError = 0;
  m_sumError = 0;


  //initialise
  setSpeed(0);
  setState(FORWARD);
  setState(RELEASE);
}

Motor::~Motor()
{
}

void Motor::setSpeed(int spd)
{
  m_reference->setSpeed(spd);
}

void Motor::setState(int state)
{
  m_reference->run(state);
}

void Motor::setCommand(float rpm)
{
  m_targetRPM = rpm;
}


float Motor::getRPM()
{
  float currentSpeed = m_tick/(encoder_pulses * REFRESH_TIME) * 60000;
  m_tick = 0;
  return currentSpeed;
}

float Motor::getCommand()
{
  return m_targetRPM;
}


void Motor::incrementTickMotor()
{
  m_tick++;
}

void Motor::clearTickMotor()
{
  m_tick = 0;
}
