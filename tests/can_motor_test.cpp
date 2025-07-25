/**
 * @author Stefan May
 * @date 08.05.2018
 * @brief Test program for CAN interface
 */
#include "MotorControllerCAN.h"
#include <unistd.h>
#include <iostream>
#include <cmath>

using namespace std;

void setPWM(MotorController* mc, int val)
{
  int pwm[2];
  pwm[0] = val;
  pwm[1] = val;
  if(!mc->setPWM(pwm))
  {
    std::cout << "# Failed to set PWM values for CAN ID" << mc->getCanId() << std::endl;
    usleep(1000);
  }
}

void setRPM(MotorController* mc, float val)
{
  float rpm[2];
  rpm[0] = val;
  rpm[1] = val;
  if(!mc->setRPM(rpm))
  {
    std::cout << "# Failed to set RPM values for CAN ID" << mc->getCanId() << std::endl;
    usleep(1000);
  }
}

#define _INSTANCES 2
int main(int argc, char* argv[])
{
  MotorParams motorParams;
  motorParams.frequencyScale = 32;    // PWM frequency: 1/frequencyScale x 500kHz
  motorParams.inputWeight    = 0.8f;  // Smoothing parameter for input values: smoothVal = inputWeight x prevVal + (1.f-inputWeight) x newVal
  motorParams.maxPulseWidth  = 127;   // Set maxPulse to 127 to apply full power
  motorParams.timeout        = 300;
  motorParams.gearRatio      = 250.f;
  motorParams.encoderRatio   = 48.f;
  motorParams.rpmMax         = 100;
  motorParams.responseMode   = CAN_RESPONSE_RPM;
  motorParams.kp             = 2.f;
  motorParams.ki             = 1000.f;
  motorParams.kd             = 0.f;
  motorParams.antiWindup     = 1;
  motorParams.invertEnc      = 1;

  SocketCAN can(std::string("slcan0"));
  can.startListener();

  std::vector<MotorController*> mc;
  unsigned int dev = 0;
  for(dev=0; dev<_INSTANCES; dev++)
  {
    MotorController* m = new MotorController(&can, dev, motorParams);
    mc.push_back(m);
  }

  for(int i=0; i<500; i++)
  {
    if(i%50==0)
      mc[0]->broadcastExternalSync();

    float phase = ((float)i) * (2.f*M_PI) * 0.002;
    float amplitude = 3.f;
    float val = (sin(phase) * amplitude);
    //float val = amplitude;
    for(dev=0; dev<mc.size(); dev++)
      //setPWM(mc[dev], val);
      setRPM(mc[dev], val);

    std::cout << val;
    for(dev=0; dev<mc.size(); dev++)
    {
      if(mc[dev]->waitForSync())
      {
        float response[2];
        mc[dev]->getWheelResponse(response);
        std::cout << " " << response[0] << " " << response[1];
      }
      else
      {
        std::cout << "# Error synchronizing with device" << mc[dev]->getCanId() << std::endl;
      };
    }
    std::cout << std::endl;
    usleep(10000);
  }

  for(dev=0; dev<mc.size(); dev++)
  {
    delete mc[dev];
  }
}
