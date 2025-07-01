#include <iostream>

#include "MecanumSteering.h"

using namespace std;

MecanumSteering::MecanumSteering() : Node("evotrainer_drive_node")
{}

void MecanumSteering::init(ChassisParams &chassisParams, MotorParams &motorParams, SocketCAN &can)
{
  _mc[0]    = new MotorController(&can, 0, motorParams);
  _mc[1]    = new MotorController(&can, 1, motorParams);

  _motorParams  = new MotorParams(motorParams);
  _chassisParams = chassisParams;
  // ensure that the direction parameter is set properly (either 1 or -1)
  if(_chassisParams.direction>0) _chassisParams.direction = 1;
  else _chassisParams.direction = -1;

  _rad2rpm          = (chassisParams.wheelBase+chassisParams.track)/chassisParams.wheelDiameter; // (lx+ly)/2 * 1/r
  _rpm2rad          = 1.0 / _rad2rpm;
  _ms2rpm           = 60.0/(chassisParams.wheelDiameter*M_PI);
  _rpm2ms           = 1.0 / _ms2rpm;
  _vMax             = motorParams.rpmMax * _rpm2ms;
  _omegaMax         = motorParams.rpmMax * _rpm2rad;


  _joySub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&MecanumSteering::joyCallback, this, std::placeholders::_1));
  _velSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MecanumSteering::velocityCallback, this, std::placeholders::_1));
  _pubRPM = this->create_publisher<std_msgs::msg::Float32MultiArray>("rpm", 1);
  _enableSrv = this->create_service<std_srvs::srv::SetBool>("enable", std::bind(&MecanumSteering::enableCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  _rpm[0] = 0.0;
  _rpm[1] = 0.0;
  _rpm[2] = 0.0;
  _rpm[3] = 0.0;

  cout << "Initialized mechanum steering with vMax: " << _vMax << " m/s" << endl;
}

MecanumSteering::~MecanumSteering()
{
  delete _mc[0];
  delete _mc[1];
  delete _motorParams;
}

void MecanumSteering::run()
{
  rclcpp::Rate rate(25);
  _lastCmd = this->now();
  unsigned int cnt;

  std_msgs::msg::Float32MultiArray msgRPM;
  float rpm[4];
  
  bool run = true;
  while(run)
  {
    rclcpp::spin_some(shared_from_this());

    rclcpp::Duration dt = this->now() - _lastCmd;
    bool lag = (dt.seconds()>0.5);
    if(lag)
    {
      RCLCPP_WARN(this->get_logger(), "%s", "Lag detected ... deactivate motor control");
      _mc[0]->stop();
      _mc[1]->stop();
    }
    else
    {
      if(cnt++>=25)
      {
        _mc[0]->broadcastExternalSync();
        cnt=0;
      }

      for(int i=0; i<=1; i++)
      {
        if(!_mc[i]->setRPM(&(_rpm[2*i])))
        {
          std::cout << "# Failed to set RPM values for CAN ID" << _mc[i]->getCanId() << std::endl;
        }
      }

      //_mc[0]->waitForSync();
      //_mc[1]->waitForSync();
      _mc[0]->getWheelResponse(rpm);
      _mc[1]->getWheelResponse(&(rpm[2]));
              
	
	   msgRPM.data.push_back(rpm[_chassisParams.frontLeft.id  * 2 + _chassisParams.frontLeft.channel]);
	   msgRPM.data.push_back(rpm[_chassisParams.frontRight.id * 2 + _chassisParams.frontRight.channel]);
	   msgRPM.data.push_back(rpm[_chassisParams.rearLeft.id   * 2 + _chassisParams.rearLeft.channel]);
	   msgRPM.data.push_back(rpm[_chassisParams.rearRight.id  * 2 + _chassisParams.rearRight.channel]);
     _pubRPM->publish(msgRPM);
    }

    run = rclcpp::ok();

    rate.sleep();
  }

  _mc[0]->stop();
  _mc[1]->stop();
}

void MecanumSteering::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // Assignment of joystick axes to motor commands
  float fwd      = joy->axes[1];            // Range of values [-1:1]
  float left     = joy->axes[0];            // Range of values [-1:1]
  float turn     = joy->axes[2];            // Range of values [-1:1]
  float throttle = (joy->axes[3]+1.0)/2.0;  // Range of values [0:1]

  float vFwd  = throttle * fwd  * _vMax;
  float vLeft = throttle * left * _vMax;
  float omega = throttle * turn * _omegaMax;

  normalizeAndMap(vFwd, vLeft, omega);
}

void MecanumSteering::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  normalizeAndMap(cmd->linear.x, cmd->linear.y, cmd->angular.z);
}

bool MecanumSteering::enableCallback(const std::shared_ptr<rmw_request_id_t> header,
                              const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
                              const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
{
  (void)header; // suppress warning about unused variable header
  if(request->data==true)
  {
    RCLCPP_INFO(this->get_logger(), "%s", "Enabling robot");
    _mc[0]->enable();
    _mc[1]->enable();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(),  "%s", "Disabling robot");
    _mc[0]->disable();
    _mc[1]->disable();
  }
  response->success = true;
  return true;
}

void MecanumSteering::normalizeAndMap(float vFwd, float vLeft, float omega)
{
  float rpmFwd   = vFwd  * _ms2rpm;
  float rpmLeft  = vLeft * _ms2rpm;
  float rpmOmega = omega * _rad2rpm;

  //cout << "vFwd: " << vFwd << "m/s, vLeft: " << vLeft << "m/s, omega: " << omega << endl;
  //cout << "rpmFwd: " << rpmFwd << ", rpmLeft: " << rpmLeft << ", rpmOmega: " << rpmOmega << endl;

  // leading signs -> see derivation: Stefan May, Skriptum Mobile Robotik
  _rpm[_chassisParams.frontLeft.id  * 2 + _chassisParams.frontLeft.channel]  =  rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.frontRight.id * 2 + _chassisParams.frontRight.channel] = -rpmFwd - rpmLeft - rpmOmega;
  _rpm[_chassisParams.rearLeft.id   * 2 + _chassisParams.rearLeft.channel]   =  rpmFwd + rpmLeft - rpmOmega;
  _rpm[_chassisParams.rearRight.id  * 2 + _chassisParams.rearRight.channel]  = -rpmFwd + rpmLeft - rpmOmega;

  // possibility to flip directions
  _rpm[0] *= _chassisParams.direction;
  _rpm[1] *= _chassisParams.direction;
  _rpm[2] *= _chassisParams.direction;
  _rpm[3] *= _chassisParams.direction;

  // Normalize values, if any value exceeds the maximum
  float rpmMax = std::abs(_rpm[0]);
  for(int i=1; i<4; i++)
  {
    if(std::abs(_rpm[i]) > _motorParams->rpmMax)
      rpmMax = std::abs(_rpm[i]);
  }
  if(rpmMax > _motorParams->rpmMax)
  {
    float factor = _motorParams->rpmMax / rpmMax;
    for(int i=0; i<4; i++)
      _rpm[i] *= factor;
  }

  _lastCmd = this->now();
}
