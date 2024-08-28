/*
This ROS node is controlling the movement of the rosbot_2r robot based on object detection results 
provided by the `find_object_2d` package. 

Remember: Before running this node, Run the `object_teaching.launch` file with `teach` argument 
set as "true", to save images of the objects for recognition using find_object_2d pkg.

The node performs the following functions:
1. **Subscribes to the `/objects` topic, where it receives data about detected objects in the environment. 
Each detected object has an associated ID, which indicates the type of object recognized by the robot.

2. **Movement Control Based on Object ID**:
   - Object ID 4 (Speed_up):    The robot moves forward with a defined linear velocity.
   - Object ID 5 (Stop):        The robot stops moving.
   - Object ID 6 (Bottle_can):  The robot rotates in place with a defined angular velocity.
   
3. **Default Behavior: The robot will continue to rotate and move slowly forward to search for objects. 

The node continuously publishes `geometry_msgs::Twist` messages to the `/cmd_vel` topic to control the robot’s movement.

Reference: https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/
*/



#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

/* 
Object IDs as found in the img_data folder:
1.  Speed_up 4
2.  Stop 5
3.  Bottle_can 6
*/

geometry_msgs::Twist twist; // declaring a variable named "twist" of type geometry_msgs::Twist
ros::Publisher pub;
/* 
Callback Function:
1.  If an object with an existing ID is found in the environment then we perform 
certain actions {robot movements}, 
otherwise the robot keeps rotating, and moving around in order to find the objects.
*/
void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    if (object->data.size() > 0)
    {
        int id = object->data[0]; // data[0] gives us the ID of the detected objects

        // If the object ID matches SPEED_UP, move the robot forward
        if (id == 4)
        {
            twist.linear.x = 0.4;
            twist.angular.z = 0;
        } 
        // If the object ID matches STOP, stop the robot
        else if (id == 5)
        {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }
        // If the object ID matches BOTTLE_CAN, rotate the robot
        else if (id == 6)
        {
            twist.linear.x = 0;
            twist.angular.z = 0.5;
        }
        // For any other object IDs, do nothing and keep rotating
        else
        {
            twist.linear.x = 0.05;
            twist.angular.z = 0.25;
        }
    }
    // If no objects are detected, rotate and move forward to continue searching
    else
    {
        twist.linear.x = 0.05;
        twist.angular.z = 0.25;
    }

    pub.publish(twist); // Publish the twist message to command the robot's movement
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "object_detection_node"); // Initialize the ROS node: "object_detection_node"
   ros::NodeHandle nh("~"); // Create a NodeHandle for this node
   ros::Rate loop_rate(30); // Loop rate of 30 Hz

   // Subscribe to the "/objects" topic to receive object detection data
   ros::Subscriber sub = nh.subscribe("/objects", 1, objectCallback);

   // Advertise the "/cmd_vel" topic to publish Twist messages for controlling the robot
   pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

   while (ros::ok())
   {
    // Process incoming data and sleep to maintain loop rate
      ros::spinOnce();
      loop_rate.sleep();
   }
}
