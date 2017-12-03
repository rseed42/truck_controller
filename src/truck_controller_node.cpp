#include <ros/ros.h>
#include <stdio.h>
#include <pigpiod_if2.h>
#include "std_msgs/UInt32.h"

using namespace std;
//------------------------------------------------------------------------------
// Error codes
//------------------------------------------------------------------------------
#define ERROR_GPIOD_CONNECTION_FAILED 1
#define ERROR_GPIO_MODE_SETUP_FAILED  2
#define ERROR_LISTENER_REGISTER_FAILED 3
//------------------------------------------------------------------------------
// Default parameters
//------------------------------------------------------------------------------
const int THROTTLE_PIN = 25;
const int STEERING_PIN = 18;
const int MSG_QUEUE_SIZE = 10;
const string TOPIC_THROTTLE = "/controller/throttle";
const string TOPIC_STEERING = "/controller/steering";
//------------------------------------------------------------------------------
// Configuration object
//------------------------------------------------------------------------------
class TruckConfiguration {
  int throttlePin;
  int steeringPin;
  int msgQueueSize;
  string topicThrottle;
  string topicSteering;
  bool debug;
public:
  TruckConfiguration(const ros::NodeHandle& nh) {
    nh.param<int>("throttle-pin", throttlePin, THROTTLE_PIN);
    nh.param<int>("steering-pin", steeringPin, STEERING_PIN);
    nh.param<int>("message-queue-size", msgQueueSize, MSG_QUEUE_SIZE);
    nh.param<std::string>("topic-throttle", topicThrottle, TOPIC_THROTTLE);
    nh.param<std::string>("topic-steering", topicSteering, TOPIC_STEERING);
    nh.param<bool>("debug", debug, false);
  }
  int getThrottlePin() { return throttlePin; }
  int getSteeringPin() { return steeringPin; }
  int getMsgQueueSize() { return msgQueueSize; }
  std::string getTopicThrottle() { return topicThrottle; }
  std::string getTopicSteering() { return topicSteering; }
  bool getDebug() { return debug; }
};
//------------------------------------------------------------------------------
// Listener
//------------------------------------------------------------------------------
class ServoListener {
  const bool debug;
  const int piHandle;
  const int pinServo;
  ros::Subscriber subscriber;
public:
  ServoListener(const int pi, const int pin, const bool debugOn) :
    piHandle(pi), pinServo(pin), debug(debugOn){
  }
  void onServoMessage(const std_msgs::UInt32::ConstPtr& message) {
    if(debug)
      printf("Received steering: %d\n", message->data);
    // Set the pulse width specified in the message
    int pulseWidth = (int)message->data;
    if(pulseWidth > 2000) { pulseWidth == 2000; }
    else if(pulseWidth < 1000) { pulseWidth == 1000; }
    set_servo_pulsewidth(piHandle, pinServo, pulseWidth);
  }
  bool subscribe(ros::NodeHandle& nh, std::string topic, int queue_size) {
    subscriber = nh.subscribe(topic, queue_size, &ServoListener::onServoMessage, this);
     if(subscriber) return true;
     return false;
  }
};
//------------------------------------------------------------------------------
// Controller
//------------------------------------------------------------------------------
class TruckController {
private:
  const int piHandle;
  ros::NodeHandle nodeHandle;
  TruckConfiguration config;
  ServoListener throttleListener;
  ServoListener steeringListener;
  // GPIO Setup. Sets mode to OUTPUT.
  bool setupGPIO(const int pin) {
    if(set_mode(piHandle, pin, PI_OUTPUT) != 0) {
      std::cerr << "Error setting GPIO " << pin << ": ";
      switch(piHandle) {
        case PI_BAD_GPIO: std::cerr << "BAD GPIO";
          return false;
        case PI_BAD_MODE: std::cerr << "BAD GPIO MODE";
          return false;
        case PI_NOT_PERMITTED: std::cerr << "Permission denied";
          return false;
      }
      std::cerr << std::endl;
    }
    return true;
  }

public:
  TruckController(TruckConfiguration& truckConfig , const int pi, ros::NodeHandle& nh) :
    config(truckConfig),
    piHandle(pi),
    nodeHandle(nh),
    throttleListener(pi, truckConfig.getThrottlePin(), truckConfig.getDebug()),
    steeringListener(pi, truckConfig.getSteeringPin(), truckConfig.getDebug()) {
  }
  bool initGPIO() {
    // This will stop evaluation on the first failed setup and return false
    return setupGPIO(config.getThrottlePin()) && setupGPIO(config.getSteeringPin());
  }
  bool registerListeners() {
     // Subscribe both listeners or fail on error
    return throttleListener.subscribe(nodeHandle, config.getTopicThrottle(), config.getMsgQueueSize()) &&
           steeringListener.subscribe(nodeHandle, config.getTopicSteering(), config.getMsgQueueSize());
  }

};
//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char** argv){

  // Initialize ROS
  ros::init(argc, argv, "hc_sr04");

  // ROS node handler
  ros::NodeHandle nodeHandler;

  // Load configuration parameters from the launch file
  TruckConfiguration config = TruckConfiguration(nodeHandler);

  // Connect to gpiod using default parameters
  const int pi = pigpio_start(NULL, NULL);
  if(pi < 0){
    printf("Could not connect to the pigpiod daemon. Aborting.\n");
    return ERROR_GPIOD_CONNECTION_FAILED;
  }

  // Instantiate the controller and start listeners
  TruckController controller = TruckController(config, pi, nodeHandler);
  if(!controller.initGPIO()) { return ERROR_GPIO_MODE_SETUP_FAILED; }
  if(!controller.registerListeners()) { return ERROR_LISTENER_REGISTER_FAILED; }

  // Start the ROS node
  ROS_INFO("Starting truck controller node...");
  ros::spin();

  std::cout << "Shutdown." << std::endl << std::flush;
  return 0;
}
