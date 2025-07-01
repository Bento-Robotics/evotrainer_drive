#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "MecanumSteering.h"

int main(int argc, char *argv[])
{ 
  rclcpp::init(argc, argv);
  auto evo_drive = std::make_shared<MecanumSteering>();
  ChassisParams chassisParams;
  MotorParams motorParams;

  // Assign motor channels to motor/wheel mounting

  std::string canInterface;
  int maxPulseWidth;
  int frequencyScale;
  evo_drive->declare_parameter("track",          0.3f);
  evo_drive->declare_parameter("wheelBase",      0.3f);
  evo_drive->declare_parameter("wheelDiameter",  0.1f);
  evo_drive->declare_parameter("idFrontLeft",    0);
  evo_drive->declare_parameter("chFrontLeft",    0);
  evo_drive->declare_parameter("idFrontRight",   0);
  evo_drive->declare_parameter("chFrontRight",   1);
  evo_drive->declare_parameter("idRearLeft",     1);
  evo_drive->declare_parameter("chRearLeft",     0);
  evo_drive->declare_parameter("idRearRight",    1);
  evo_drive->declare_parameter("chRearRight",    1);
  evo_drive->declare_parameter("direction",      1);
  evo_drive->declare_parameter("canInterface",   std::string("can0"));
  evo_drive->declare_parameter("frequencyScale", 32);
  evo_drive->declare_parameter("gearRatio",      131.f);
  evo_drive->declare_parameter("encoderRatio",   64.f);
  evo_drive->declare_parameter("rpmMax",         80.f);
  evo_drive->declare_parameter("maxPulseWidth",  63);
  evo_drive->declare_parameter("kp",             1.f);
  evo_drive->declare_parameter("ki",             0.f);
  evo_drive->declare_parameter("kd",             0.f);
  evo_drive->declare_parameter("antiWindup",     1);
  evo_drive->declare_parameter("invertEnc",      0);

  chassisParams.track             = evo_drive->get_parameter("track"         ).as_double();
  chassisParams.wheelBase         = evo_drive->get_parameter("wheelBase"     ).as_double();
  chassisParams.wheelDiameter     = evo_drive->get_parameter("wheelDiameter" ).as_double();
  chassisParams.frontLeft.id      = evo_drive->get_parameter("idFrontLeft"   ).as_int();
  chassisParams.frontLeft.channel = evo_drive->get_parameter("chFrontLeft"   ).as_int();
  chassisParams.frontRight.id     = evo_drive->get_parameter("idFrontRight"  ).as_int();
  chassisParams.frontRight.channel= evo_drive->get_parameter("chFrontRight"  ).as_int();
  chassisParams.rearLeft.id       = evo_drive->get_parameter("idRearLeft"    ).as_int();
  chassisParams.rearLeft.channel  = evo_drive->get_parameter("chRearLeft"    ).as_int();
  chassisParams.rearRight.id      = evo_drive->get_parameter("idRearRight"   ).as_int();
  chassisParams.rearRight.channel = evo_drive->get_parameter("chRearRight"   ).as_int();
  chassisParams.direction         = evo_drive->get_parameter("direction"     ).as_int();
  canInterface                    = evo_drive->get_parameter("canInterface"  ).as_string();
  frequencyScale                  = evo_drive->get_parameter("frequencyScale").as_int();
  motorParams.gearRatio           = evo_drive->get_parameter("gearRatio"     ).as_double();
  motorParams.encoderRatio        = evo_drive->get_parameter("encoderRatio"  ).as_double();
  motorParams.rpmMax              = evo_drive->get_parameter("rpmMax"        ).as_double();
  maxPulseWidth                   = evo_drive->get_parameter("maxPulseWidth" ).as_int();
  motorParams.kp                  = evo_drive->get_parameter("kp"            ).as_double();
  motorParams.ki                  = evo_drive->get_parameter("ki"            ).as_double();
  motorParams.kd                  = evo_drive->get_parameter("kd"            ).as_double();
  motorParams.antiWindup          = evo_drive->get_parameter("antiWindup"    ).as_int();
  motorParams.invertEnc           = evo_drive->get_parameter("invertEnc"     ).as_int();

  if(maxPulseWidth<0)   maxPulseWidth = 0;
  if(maxPulseWidth>127) maxPulseWidth = 127;
  motorParams.maxPulseWidth = (unsigned char)maxPulseWidth;

  if(frequencyScale<1)   frequencyScale = 1;
  if(frequencyScale>100) frequencyScale = 100;
  motorParams.frequencyScale = (unsigned short)frequencyScale;

  SocketCAN can(canInterface);
  can.startListener();
  cout << "CAN Interface: " << canInterface << endl;

  evo_drive->init(chassisParams, motorParams, can);
  evo_drive->run();
}
