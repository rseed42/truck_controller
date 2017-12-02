#include <ros/ros.h>
#include <stdio.h>
#include <pigpiod_if2.h>
#include "std_msgs/UInt32.h"

using namespace std;
//------------------------------------------------------------------------------
// Error codes
//------------------------------------------------------------------------------
#define ERROR_CONNECTION_FAILED 1

//------------------------------------------------------------------------------
// Default configuration
//------------------------------------------------------------------------------
const int THROTTLE_PIN = 25;
const int STEERING_PIN = 18;
const int MSG_QUEUE_SIZE = 10;
const string TOPIC_THROTTLE = "/controller/throttle";
const string TOPIC_STEERING = "/controller/steering";
//------------------------------------------------------------------------------
// Controller
//------------------------------------------------------------------------------
class TruckController {
private:
  const int piHandle;
  const ros::NodeHandle nodeHandle;
  const int pinThrottle;
  const int pinSteering;

public:
  TruckController(const int pi, ros::NodeHandle& nh, const int throttle, const int steering) :
    piHandle(pi), nodeHandle(nh), pinThrottle(throttle), pinSteering(steering) {

  }

  void onThrottleMsg(const std_msgs::UInt32::ConstPtr& msg) {
    printf("Received throttle: %d\n", msg->data);
    int width = (int)msg->data;
    set_servo_pulsewidth(piHandle, pinThrottle, width);
  }

  void onSteeringMsg(const std_msgs::UInt32::ConstPtr& msg) {
    printf("Received steering: %d\n", msg->data);
    int width = (int)msg->data;
    set_servo_pulsewidth(piHandle, pinSteering, width);
  }

};
//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char** argv){

  // Initialize ROS
  ros::init(argc, argv, "hc_sr04");

  // Connect to gpiod using default parameters
  const int pi = pigpio_start(NULL, NULL);
  if(pi < 0){
    printf("Could not connect to pigpiod\n");
  }

  // Setup the GPIO pins
  // TODO: put this to the Controller class
  // PI_BAD_GPIO, PI_BAD_MODE, or PI_NOT_PERMITTED
  if(set_mode(pi, THROTTLE_PIN, PI_OUTPUT) != 0) {
    std::cerr << "Error setting GPIO " << THROTTLE_PIN << ": ";
    switch(pi) {
      case PI_BAD_GPIO: std::cerr << "BAD GPIO";
        break;
      case PI_BAD_MODE: std::cerr << "BAD GPIO MODE";
        break;
      case PI_NOT_PERMITTED: std::cerr << "Permission denied";
        break;
    }
    std::cerr << std::endl;
    return ERROR_CONNECTION_FAILED;
  }

  // Setup the GPIO pins
  // TODO: put this to the Controller class
  // PI_BAD_GPIO, PI_BAD_MODE, or PI_NOT_PERMITTED
  if(set_mode(pi, STEERING_PIN, PI_OUTPUT) != 0) {
    std::cerr << "Error setting GPIO " << STEERING_PIN << ": ";
    switch(pi) {
      case PI_BAD_GPIO: std::cerr << "BAD GPIO";
        break;
      case PI_BAD_MODE: std::cerr << "BAD GPIO MODE";
        break;
      case PI_NOT_PERMITTED: std::cerr << "Permission denied";
        break;
    }
    std::cerr << std::endl;
    return ERROR_CONNECTION_FAILED;
  }


  std::cout << "--- Starting truck controller ---" << std::endl;

  // ROS node handler
  ros::NodeHandle nodeHandler;

  // Controller
  TruckController controller = TruckController(pi, nodeHandler, THROTTLE_PIN, STEERING_PIN);

  // Subscribe message handlers
  ros::Subscriber subThrottle = nodeHandler.subscribe(
    TOPIC_THROTTLE,
    MSG_QUEUE_SIZE,
    &TruckController::onThrottleMsg,
    &controller
  );

  ros::Subscriber subSteering = nodeHandler.subscribe(
    TOPIC_STEERING,
    MSG_QUEUE_SIZE,
    &TruckController::onSteeringMsg,
    &controller
  );

  // Start the ROS node
  ROS_INFO("Starting truck controller node...");
  ros::spin();

  std::cout << "Shutdown." << std::endl << std::flush;
  return 0;
}
