#include "my_panda_controllers/my_cartesian_velocity_controller.h"


#include <cmath>
#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include "my_panda_controllers/MyVelocity.h"

namespace my_panda_controllers {

std::string convToStr(std::array<double, 6> arr)
{
  std::string out;
  for (double i: arr) {
      out + std::to_string(i);
      // ROS_INFO(out);
  }

  // ROS_INFO("converted: ");
  // ROS_INFO(out);
  return out;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template<class T,size_t N>
std::array<T,N> convert( const boost::array<T, N> & v )
{
    //assert(v.size() == N);
    std::array<T,N> r;
    std::copy( v.begin(), v.end(), r.begin() );
    return r;
}

bool CartesianVelocityMyController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  // max limits:
  // this->max_velocity = {1700, 1700, 1700, 2500, 2500, 2500};
  // this->max_acceleration = {13000, 13000, 13000, 25000, 25000, 25000};
  this->max_jerk = {6500, 6500, 6500, 12500, 12500, 12500};

  this->max_velocity = {1, 1, 1, 1, 1, 1};
  this->max_acceleration = {5, 5, 5, 5, 5, 5};
  
  
  
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityMyController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityMyController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityMyController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityMyController: Could not get state interface from hardware");
    return false;
  }

  sub_vel = node_handle.subscribe("my_velocities", 20, &CartesianVelocityMyController::velocity_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void CartesianVelocityMyController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityMyController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  // catkin build my_panda_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

  // TODO: begrenzung ueber robot.control moeglich?: hat nichts gebracht
   for (size_t i = 0; i < 6; i++)
  {
    double acc = (this->velocity_goal[i]-this->velocity_old[i])/period.toSec(); 
    // Acceleration LIMITING:
    if(abs(acc) > this->max_acceleration[i]) {
      this->velocity_out[i] = this->velocity_old[i] + this->max_acceleration[i]*period.toSec()*sgn(acc);
      ROS_INFO_STREAM_THROTTLE(1, "limited acceleration. Velocity" << i << this->velocity_out[i]);
      // ROS_INFO_STREAM("Periode: " << period);
    }
    else {this->velocity_out[i] = this->velocity_goal[i];}
  
    // Jerk LIMITING:
    acc = (this->velocity_out[i]-this->velocity_old[i])/period.toSec();
    double jerk = abs(acc - last_acc[i])/period.toSec();
    if(jerk > this->max_jerk[i]) {
      this->velocity_out[i] = this->velocity_old[i] + this->max_jerk[i]*period.toSec()*period.toSec()*sgn(acc);
      ROS_INFO_STREAM_THROTTLE(1, "limited jerk. Velocity" << i << this->velocity_out[i]);
    }

    this->last_acc[i] = acc;
    this->velocity_old[i]=this->velocity_out[i];
  }
  // ROS_INFO_STREAM("setting velocity_out");
  // ROS_INFO_STREAM("velocity 0: " << this->velocity_out[0]);
  
  velocity_cartesian_handle_->setCommand(this->velocity_out);
  
}

void CartesianVelocityMyController::velocity_callback(my_panda_controllers::MyVelocity cartesian_velocities)
{
    
    this->velocity_goal = convert(cartesian_velocities.vel);
    for (size_t i = 0; i < 6; i++)
    {
      if(this->velocity_goal[i] > this->max_velocity[i]){
        this->velocity_goal[i] = this->max_velocity[i]; 
        ROS_INFO("limited velocity");
        }
    }
    ROS_INFO("changed velocity_goal");
    ROS_INFO_STREAM("State: " << velocity_cartesian_handle_->getRobotState());
    
    // velocity_cartesian_handle_->setCommand(this->velocity_old);
}

void CartesianVelocityMyController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace my_panda_controllers

PLUGINLIB_EXPORT_CLASS(my_panda_controllers::CartesianVelocityMyController,
                       controller_interface::ControllerBase)

