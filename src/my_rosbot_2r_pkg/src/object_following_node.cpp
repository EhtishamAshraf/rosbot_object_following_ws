/*
This ROS node is adjusting the movement of the rosbot_2r robot based on object detection results 
provided by the `find_object_2d` package so that the robot is always focused on the center of the object. 

Remember: Before running this node, Run the `object_detection.launch` file with `teach` argument 
set as "true", to save images of the objects for recognition using find_object_2d pkg.

The node performs the following functions:
1. **Subscribes to the `/objects` topic, where it receives data about detected objects in the environment. 
   The object having an ID = 5 is the object which the robot will follow

2. **Movement Control Based on Object ID**:
   - Once the object is detected, corners of the object are stored in inPts vector.
   - Homography matrix is also fetched from the information published on the /objects topic.
   - PerspectiveTransformation is used to transform points from image plane to camera frame. 
   - So that the position of the object is known w.r.t the robot which could be used to follow the object.
   
3. **Default Behavior: The robot will continue to rotate to search for about detected objects in the environment. 
   The object having an ID = 5 is the object which the robot will follow with ID = 5. 

The node continuously publishes `geometry_msgs::Twist` messages to the `/cmd_vel` topic to control the robot’s movement.

Reference: https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/
*/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>

#define OBJECT_TO_FOLLOW 5    // the object for which the robot will look for in the environment
#define CAMERA_WIDTH    640  // camera width - can be adjusted in the URDF file {rosbot.gazebo}

#define MIN_ANG_VEL     0.10f  // angular velocity minimum limit
#define MAX_ANG_VEL     0.25f // angular velocity maximum limit
#define ANGULAR_GAIN    2e-3 // Gain factor to calculate angular velocity: 0.002 rad/s per pixel of deviation from the object center
 
geometry_msgs::Twist twist; // declaring a variable named "twist" of type geometry_msgs::Twist
ros::Publisher pub;

/* 
Callback Function:
1.  If an object with the given ID (id = 5) is found in the environment then we perform 
    object following, otherwise the robot keeps rotating to find the objects.
*/
void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    // variables to store object's center point in x and angular velocity
    int obj_x_pos; 
    float ang_vel; 

    // Reset linear and angular speed value
    twist.linear.x = 0;
    twist.angular.z = 0;
// and two vectors to hold the input and output points {each point is 2D as it has both x & y coordinates} for the transformation.
    if (object->data.size() > 0){
        // saving id, height, and width of the object
        int id = object->data[0];
        float objectWidth = object->data[1];
        float objectHeight = object->data[2];
// Initializing a 3x3 matrix to store the homography transformation (used to map object points to the image).
// and two vectors to hold the input and output points {each point is 2D as it has both x & y coordinates} for the transformation.
        cv::Mat cvHomography(3, 3, CV_32F);
        std::vector<cv::Point2f> inPts, outPts;

	    switch (id)
        {
        case OBJECT_TO_FOLLOW:

    // Homography Matrix completion (3x3): rostopic echo /objects gives us the elements of homography matrix
            for(int i=0; i<9; i++){
                cvHomography.at<float>(i%3, i/3) = object->data[i+3];
            }

            // saving corners of the object in image plane into the inPts vector
            inPts.push_back(cv::Point2f(0, 0));
            inPts.push_back(cv::Point2f(objectWidth, 0));
            inPts.push_back(cv::Point2f(0, objectHeight));
            inPts.push_back(cv::Point2f(objectWidth, objectHeight));

            // Applying the perspective transformation using the homography matrix
            // This transforms the points from the image plane to the camera frame
            cv::perspectiveTransform(inPts, outPts, cvHomography);

            // finding x-coordinate of the object's center point by averaging the x values of the outPts vector
            obj_x_pos = (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
            
            /* 
            Calculating angular velocity:
            Scenario 1: CAMERA_WIDTH/2 - obj_x_pos = 0: 
                        -   object is perfectly alligned with the camera - No Movement
            Scenario 2: CAMERA_WIDTH/2 - obj_x_pos < 0: 
                        -   object is on the right of the robot - Move Right
            Scenario 3: CAMERA_WIDTH/2 - obj_x_pos > 0:
                        -   object is on the left of the robot - Move Left
            */
            ang_vel = ANGULAR_GAIN*(CAMERA_WIDTH/2 - obj_x_pos); 

            /*
            Setting the angular speed based on calculated angular velocity and min/max limits:
            1.  To ensure that the robot only moves when the object's center is sufficiently 
                deviated from the center of the camera's view. 
            2.  The angular speed is constrained between MIN_ANG_VEL and MAX_ANG_VEL in order to 
                prevent excessive or insufficient rotation.
            */
            if(ang_vel <= -MIN_ANG_VEL || ang_vel >= MIN_ANG_VEL){
                twist.angular.z = std::max(-MAX_ANG_VEL, std::min(ang_vel, MAX_ANG_VEL));
            }
            ROS_INFO("id: %d\t | ang_vel: %f", id, twist.angular.z);

            break;
        }
    }

    pub.publish(twist); // Publish the twist message to command the robot's movement
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_following_node"); // Initialize the ROS node: "object_following_node"
    ros::NodeHandle nh("~"); // Create a NodeHandle for this node

    // Subscribe to the "/objects" topic to receive object detection data    
    ros::Subscriber sub = nh.subscribe("/objects", 1, objectCallback);
    ros::Rate loop_rate(30); // Loop rate of 30 Hz

    // Advertise the "/cmd_vel" topic to publish Twist messages for controlling the robot    
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok())
    {
    // Process incoming data and sleep to maintain loop rate        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
