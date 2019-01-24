#ifndef MOTOR_H
#define MOTOR_H

#include <Adafruit_MotorShield.h>

class Motor
{
    public:
      Motor(Adafruit_DCMotor ref);
      virtual ~Motor();

      void setSpeed(int spd);
      void setState(int state);
      void setCommand(float rpm);
      void setErrors(float error, float sumError, float deltaError);

      int getRPM(); //rpm
      int getCommand();
      float getError() { return m_error; }
      float getSumError() { return m_sumError; }
      float getDeltaError() { return m_deltaError; }

      void incrementTickMotor();
      void clearTickMotor():
    protected:

    private:
      Adafruit_DCMotor * m_reference;
      //encoder
      unsigned long m_tick;
      //state and speed
      float m_targetRPM;
      //PID
      float m_error;
      float m_sumError;
      float m_deltaError;
};

#endif // MOTOR_H
