#include "mybot_hardware_interface/mybot_hardware_interface.h"
#include <sensor_msgs/BatteryState.h>
#include <cmath>

namespace mybot_hardware_interface
{
using namespace std::string_literals;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

// In C++, constexpr is a keyword used to indicate that a function or variable can be evaluated at compile time
constexpr double rotationsToRadians(double rots)
{
    //  Converts rotations to radians 
    return rots * 2.0 * M_PI;
}

constexpr double rpsToRadPerSec(double rps)
{
    // Converts rotations per second to radians per second
    return rps * 2.0 * M_PI;
}

constexpr double radPerSecTORPS(double rad_per_sec)
{
    //  Converts radians per second to rotations per second 
    return rad_per_sec * 0.5 * M_1_PI;
}


constexpr double convertStepsToRadians(double stepsPerSecond) {
  const double stepsPerRevolution = 200.0; // Adjust this value if necessary
  const double radiansPerSecond = (stepsPerSecond / stepsPerRevolution) * (2 * M_PI);
  return radiansPerSecond;

double convertRadiansToSteps(double radiansPerSecond) {
  const double stepsPerRevolution = 200.0; // Adjust this value if necessary
  const double stepsPerSecond = (radiansPerSecond / (2 * M_PI)) * stepsPerRevolution;
  return stepsPerSecond;
}


MybotHardwareInterface::MybotHardwareInterface(ros::NodeHandle &node_handle, ros::NodeHandle& private_node_handle)
    : node_handle_(node_handle),
      private_node_handle_(private_node_handle)
{
    setupJoint("joint_left_wheel", 0);
    setupJoint("joint_right_wheel", 1);

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocity_joint_soft_limits_interface_);

    // battery_publisher_ = node_handle_.advertise<sensor_msgs::BatteryState>("/robomagellan/battery", 1);


    //TODO:update port_name_
    port_name_ = private_node_handle_.param("port", "/dev/arduino_port"s);

    tryToOpenPort();

    controller_manager_.reset(new controller_manager::ControllerManager(this, node_handle_));
    node_handle_.param("/mybot/hardware_interface/loop_hz", loop_hz_, loop_hz_);
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    update_timer_ = node_handle_.createTimer(update_freq, &MybotHardwareInterface::update, this);
}

void MybotHardwareInterface::setupJoint(const std::string& name, int index)
{
    JointStateHandle joint_state_handle(name, &joint_positions_[index], &joint_velocities_[index], &joint_efforts_[index]);
    joint_state_interface_.registerHandle(joint_state_handle);

    JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_commands_[index]);
    JointLimits limits;
    SoftJointLimits soft_limits;
    getJointLimits(name, node_handle_, limits);
    VelocityJointSoftLimitsHandle joint_limits_handle(joint_velocity_handle, limits, soft_limits);
    velocity_joint_soft_limits_interface_.registerHandle(joint_limits_handle);
    velocity_joint_interface_.registerHandle(joint_velocity_handle);
}
}

void MybotHardwareInterface::update(const ros::TimerEvent& e)
{
    if(!serial_port_.isOpen())
        tryToOpenPort();
    auto elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}


void MybotHardwareInterface::read()
{
    if(!serial_port_.isOpen())
        return;

    try {
        if (last_sent_message_.empty())
            return;

        if (!serial_port_.waitReadable()) {
            ROS_WARN("Serial port timed out without receiving bytes");
            return;
        }

        const auto serial_message = serial_port_.readline();

        std::vector<std::string> tokens;
        boost::split(tokens, serial_message, boost::is_any_of(",$\n"));

        tokens.erase(std::remove(tokens.begin(), tokens.end(), ""),
                     tokens.end());

        if (tokens.size() != 4) {
            ROS_WARN_STREAM(
                "Received message did not contain the right number of tokens. Expected 5, but got "
                    << tokens.size() << "\n" << serial_message);
            return;
        }

        joint_positions_[0] = rotationsToRadians(std::stod(tokens[0]));
        joint_positions_[1] = rotationsToRadians(std::stod(tokens[1]));
        // joint_velocities_[0] =radPerSecTORPS(std::stod(tokens[2])); //? feedBack is the same input
        // joint_velocities_[1] = radPerSecTORPS(std::stod(tokens[3])); //? feedBack is the same input

        joint_velocities_[0] =convertRadiansToSteps(std::stod(tokens[2])); //? feedBack is the same input
        joint_velocities_[1] = convertRadiansToSteps(std::stod(tokens[3])); //? feedBack is the same input


        //^ battery_publisher_ = node_handle_.advertise<sensor_msgs::BatteryState>("/vel", 1);

        // const auto battery_voltage = std::stod(tokens[4]);

        // sensor_msgs::BatteryState battery_msg;
        // battery_msg.voltage = battery_voltage;
        // battery_msg.present = battery_voltage > 4.0;
        // battery_msg.current = NAN;
        // battery_msg.charge = NAN;
        // battery_msg.capacity = NAN;
        // battery_msg.design_capacity = NAN;
        // battery_msg.percentage = NAN;
        // battery_msg.power_supply_status = battery_msg.present
        //                                   ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
        //                                   : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        // battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        // battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
        // battery_publisher_.publish(battery_msg);

        last_sent_message_.clear();
    } catch(const std::exception& e)
    {
        ROS_WARN_STREAM("Serial exception: " << e.what());
        serial_port_.close();
    }
}

void MybotHardwareInterface::write(const ros::Duration& elapsed_time)
{
    if(!serial_port_.isOpen())
        return;
    try {
        velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);

        const auto left_steps = convertRadiansToSteps(joint_velocity_commands_[0]);
        const auto right_steps = convertRadiansToSteps(joint_velocity_commands_[1]);

        const auto serial_message =
            "$" + std::to_string(left_steps) + ", " + std::to_string(right_steps) +
            "\n";

        serial_port_.write(serial_message);

        last_sent_message_ = serial_message;
    } catch (const std::exception& e)
    {
        ROS_WARN_STREAM("Serial exception: " << e.what());
        serial_port_.close();
    }
}

void MybotHardwareInterface::tryToOpenPort()
{
    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(115200); 
        auto serial_timeout = serial::Timeout::simpleTimeout(250);
        serial_port_.setTimeout(serial_timeout);
        serial_port_.open();
        ROS_INFO_STREAM("Connected to motor control arduino on serial port " << port_name_);
        return;
    } catch (const std::exception& e)
    {
        ROS_WARN_STREAM_THROTTLE(60, "Could not open serial port, " << port_name_ << ": " << e.what());
    }
}

}