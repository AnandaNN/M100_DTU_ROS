
/*
 * rosserial rod value publisher
 * sends the value of ros switch
 */


#include <ros.h>
#include <std_msgs/Bool.h>

// Message
std_msgs::Bool rodValue;

// ROS Handles and publisher
ros::NodeHandle  nh;
ros::Publisher rodPublisher("/dji/rod", &rodValue);

// Initialize
void setup()
{
  nh.initNode();
  nh.advertise(rodPublisher);

  pinMode(1,INPUT_PULLUP);
}

// arduino loop. Read value -> publish -> sleep -> repeat (30 Hz)
void loop()
{
  int val = digitalRead(1);
  rodValue.data = !((bool) val);
  rodPublisher.publish( &rodValue );
  
  nh.spinOnce();
  delay(33);
}
