#ifndef MECANUMSTEERING_H_
#define MECANUMSTEERING_H_

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>


#include "interface/MotorController.h"
#include <iostream>

using namespace std;

struct WheelParams {
  int id;      // ID of motor controller board, i.e., CAN ID
  int channel; // Channel of motor controller board, i.e., either 0 or 1
};

struct ChassisParams {
  float track;
  float wheelBase;
  float wheelDiameter;
  struct WheelParams frontLeft;
  struct WheelParams frontRight;
  struct WheelParams centerLeft;
  struct WheelParams centerRight;
  struct WheelParams rearLeft;
  struct WheelParams rearRight;
  int direction;

  ChassisParams() {
    track = 0.f;
    wheelBase = 0.f;
    wheelDiameter = 0.f;
    frontLeft.id = 0;
    frontLeft.channel = 0;
    frontRight.id = 0;
    frontRight.channel = 0;
    rearLeft.id = 0;
    rearLeft.channel = 0;
    rearRight.id = 0;
    rearRight.channel = 0;
    direction = 0;
  }
};

/**
 * @class Main class for robot drives based on mechanum steering
 * @author Stefan May
 * @date 08.10.2017
 */
class MecanumSteering : public rclcpp::Node {
public:
  /**
   * Standard Constructor
   */
  MecanumSteering();
  /**
   * @param[in] chassisParams chassis parameters, including the map for
   * assigning channels to position of wheels
   * @param[in] motorParams motor parameters
   * @param[in] can socket can interface
   */

  void init(ChassisParams &chassisParams, MotorParams &motorParams,
                  SocketCAN &can);

  /**
   * Destructor
   */
  ~MecanumSteering();

  /**
   * ROS main loop (blocking method)
   */
  void run();

  /**
   * ROS joystick callback
   * @param[in] joy message with joystick command
   */
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  
  /**
   * ROS command velocity callback
   * @param[in] cmd message with velocity command
   */
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);
  
private:
  /**
   * ROS enable service callback
   * @param[in] header service header
   * @param[in] request service component
   * @param[in] response service component
   */
  bool enableCallback(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<std_srvs::srv::SetBool_Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool_Response> response);

  /**
   * Normalize motion command and assign it to the channels
   * @param[in] vFwd forward velocity (x-axis)
   * @param[in] vLeft velocity to the left (y-axis)
   * @param[in] omega angular velocity (around z-axis)
   */
  void normalizeAndMap(float vFwd, float vLeft, float omega);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr          _joySub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      _velSub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  _pubRPM;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr              _enableSrv;

  ChassisParams _chassisParams;
  MotorParams *_motorParams;
  MotorController *_mc[2];

  // revolutions per minute for each channel
  float _rpm[4];

  // maximum velocity [m/s]
  float _vMax;

  // maximum rotating rate [rad]
  float _omegaMax;

  // conversion from [m/s] to revolutions per minute [RPM]
  float _ms2rpm;

  // conversion from revolutions per minute [RPM] to [m/s]
  float _rpm2ms;

  // conversion from [rad/s] to revolutions per minute [RPM]
  float _rad2rpm;

  // conversion from revolutions per minute [RPM] to [rad/s]
  float _rpm2rad;

  // time elapsed since last call
  rclcpp::Time _lastCmd;
};

#endif
