/*
This ROS node is adjusting the movement of the rosbot_2r robot (so that the robot is always 
focused on the center of the object and the distance between object and robot is within the defined limits) 
based on detected object ID provided by the `find_object_2d` package. 

Remember: Before running this node, Run the `object_teaching.launch` file with `teach` argument 
set as "true", to save images of the objects for recognition using find_object_2d pkg.

The node performs the following functions:
1. **Subscribes to the `/objects` topic, where it receives data about detected objects in the environment. 
   The object having an ID = 5 is the object which the robot will follow.

2. **Movement Control Based on Object ID**:
   - Once the object is detected, corners of the object are stored in inPts vector.
   - Homography matrix is also fetched from the information published on the /objects topic.
   - PerspectiveTransformation is used to transform points from image plane to camera frame. 
   - So that the position of the object is known w.r.t the robot which could be used to follow the object.
   
3. **Default Behavior: The robot will continue to rotate to search for detected objects in the environment. 
   The object having an ID = 5 is the object which the robot will follow with ID = 5. 

4. **OpenCV integration**:
   - Once the correct object is detected, openCV is used to draw a bounding box on the object.
   - A circle is also drawn in the center of the object.
   - This node subscribes to the topic "camera/color/image_raw" to capture raw images of the objects,
     images are then converted to an OpenCV format for further processing using "cv_bridge" package.

5. **Maintaining distance to the Object**:
   - Once the correct object is detected, the distance of the object is detected with both TOF sensor and Lidar
   - If use_lidar is set to `true` then the lidar is used to detect distance to the object.
   - Otherwise TOF Sensors (front left and front right) are used to detect the distance to the object.

The node continuously publishes `geometry_msgs::Twist` messages to the `/cmd_vel` topic to control the robot’s movement.

Reference: https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/
*/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

#define OBJECT_TO_FOLLOW 5    // the object for which the robot will look for in the environment
#define CAMERA_WIDTH    640  // camera width - can be adjusted in the URDF file {rosbot.gazebo}
#define CAMERA_HEIGHT   480  // camera height - can be adjusted in the URDF file {rosbot.gazebo}

#define MIN_ANG_VEL     0.10f  // angular velocity minimum limit
#define MAX_ANG_VEL     0.25f // angular velocity maximum limit
#define ANGULAR_GAIN    2e-3 // Gain factor to calculate angular velocity: 0.002 rad/s per pixel of deviation from the object center

#define DESIRED_DIST          0.7f // It is only used for TOF distance sensor
#define DESIRED_DIST_lidar    1.65f // Lidar
#define MIN_LIN_VEL           0.05f
#define MAX_LIN_VEL           0.4f
#define LINEAR_GAIN           0.4f // 0.4 m/s for each meter of difference

// variables to store distance values for the TOF distance sensors and Lidar:
float rangeFL = 0;
float rangeFR = 0;
float smallest_dist = 0;
float desired_distance = 0;
bool lidarParam_variable; //variable to store ROS parameter value
float avg_dist; // distance from the object with ID 5
int id;  // ID of the object

ros::Publisher pub;
geometry_msgs::Twist twist; // declaring a variable named "twist" of type geometry_msgs::Twist

// variables to store image, object width, object height, object x and y position, and name.
cv::Mat global_image; 
float objectWidth;
float objectHeight;
int obj_x_pos, obj_y_pos;
std::string object_name_ID5;

// and two vectors to hold the input and output points {each point is 2D as it has both x & y coordinates} for the transformation.
std::vector<cv::Point2f> inPts, outPts;

/* 
Callback Function:
1.  If an object with the given ID (id = 5) is found in the environment then we perform 
    object following, otherwise the robot keeps rotating to find the objects.
*/
void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
    
    float ang_vel;
    float x_vel;

    // Reset linear and angular speed value
    twist.linear.x = 0;
    twist.angular.z = 0;

    if (object->data.size() > 0){
        // saving id, height, and width of the object
        id = object->data[0];
        objectWidth = object->data[1];
        objectHeight = object->data[2];

        // Initializing a 3x3 matrix to store the homography transformation (used to map object points to the image).
        cv::Mat cvHomography(3, 3, CV_32F);
	    switch (id)
        {
        case OBJECT_TO_FOLLOW:
            object_name_ID5 = "STOP";

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

            // finding x and y coordinate of the object's center point by averaging the x & y values of the outPts vector
            obj_x_pos = (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
            obj_y_pos = (outPts.at(0).y + outPts.at(1).y + outPts.at(2).y + outPts.at(3).y) / 4;

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
            -   if the parameter value is `False`, TOF Sensors will be used to detect distance of the object
            -   otherwise Lidar will be used for this purpose
            */
            if (!lidarParam_variable)
            {
                avg_dist = (rangeFL + rangeFR) / 2.0; // average distance value
                desired_distance = DESIRED_DIST;
            }
            else
            {
                avg_dist = smallest_dist;
                desired_distance = DESIRED_DIST_lidar;
            }

            // calculating the linear velocity required to maintain a constant distance to the object
            x_vel = LINEAR_GAIN*(avg_dist - desired_distance); 

            /*
            Setting the angular speed based on calculated angular velocity and min/max limits:
            1.  To ensure that the robot only moves when the object's center is sufficiently 
                deviated from the center of the camera's view. 
            2.  The angular speed is constrained between MIN_ANG_VEL and MAX_ANG_VEL in order to 
                prevent excessive or insufficient rotation.
            */
            if(ang_vel <= -MIN_ANG_VEL || ang_vel >= MIN_ANG_VEL)
            {
                twist.angular.z = std::max(-MAX_ANG_VEL, std::min(ang_vel, MAX_ANG_VEL));
            }
            
            /*
            Setting the linear speed based on calculated linear velocity and min/max limits:
            1.  To ensure that the robot stays within the defined limit (that is maintains
                a fixed distance to the object).
            */
            else if(x_vel <= -MIN_LIN_VEL || x_vel >= MIN_LIN_VEL)
            {
                twist.linear.x = std::max(-MAX_LIN_VEL, std::min(x_vel, MAX_LIN_VEL));
            }
            
            if (!lidarParam_variable)
            {
            ROS_INFO("ID: %-5d | Angular Velocity: %-10.6f | Avg Distance: %-10.6f | X Velocity: %-10.6f", id, twist.angular.z, avg_dist, twist.linear.x);
            }
            else
            {
            ROS_INFO("ID: %-5d | Angular Velocity: %-10.6f | Smallest Distance: %-10.6f | X Velocity: %-10.6f", id, twist.angular.z, avg_dist, twist.linear.x);
            }

            break;

        default: // rotate the robot
            twist.linear.x = 0;
            twist.angular.z = 0.25;
            break;
        }
    }
    else // rotate the robot
    {
        twist.linear.x = 0;
        twist.angular.z = 0.25;
    }

    pub.publish(twist); // Publish the twist message to command the robot's movement

    // ROS_INFO("ang_vel: %f", twist.angular.z);
/*
The bounding box will only be displayed when the angular speed of the robot is almost 0, 
which indicates that the robot has been detected.
*/
    if (!global_image.empty())
    {
        if(fabs(twist.angular.z) < MIN_ANG_VEL) 
        {

            if (avg_dist >= 0.85)
            {
            // Draw a bounding box at a predefined location
            cv::Rect bounding_box(((CAMERA_WIDTH - objectWidth) / 2) + 15, 
                                 (CAMERA_HEIGHT - objectHeight) / 2, 
                                 objectWidth-30, objectHeight-30); // x, y, width, height
            cv::rectangle(global_image, bounding_box, cv::Scalar(0, 255, 0), 3); // Green bbox
            // Display name of the sign on the Bounding Box
            cv::putText(global_image, object_name_ID5, 
                        cv::Point(((CAMERA_WIDTH - objectWidth) / 2) + 15, 
                        ((CAMERA_HEIGHT - objectHeight) / 2) + 20), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 3);
            }
            else
            {
            // Draw a bounding box at a predefined location
            cv::Rect bounding_box(((CAMERA_WIDTH - objectWidth) / 2) - 50, 
                                 ((CAMERA_HEIGHT - objectHeight) / 2) - 90,  
                                 objectWidth+100, objectHeight+70); // x, y, width, height
            cv::rectangle(global_image, bounding_box, cv::Scalar(0, 255, 0), 3); // Green bbox

            // Display name of the sign on the Bounding Box
            cv::putText(global_image, object_name_ID5, 
                        cv::Point(((CAMERA_WIDTH - objectWidth) / 2) - 50, 
                        ((CAMERA_HEIGHT - objectHeight) / 2) - 70), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 3);
            }

            cv::Point center(obj_x_pos, obj_y_pos);
            //  Draw a circle with Green line borders.
            cv::circle(global_image, center, 10, cv::Scalar(0, 255, 0), -1);

        }
        // Displaying the image with the bounding box
        cv::imshow("Captured Image with Bounding Box", global_image);
        cv::waitKey(1); //1 ms delay: Wait for a key press.
    }
}

// Callback function to handle the incoming camera image
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // Create a pointer to store the converted image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Converting the ROS image message to an OpenCV-compatible image (cv::Mat object)
        // The image encoding is specified as BGR8, which corresponds to an 8-bit BGR image.
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Now: cv_ptr->image contains the captured image as a cv::Mat object          
        global_image = cv_ptr->image.clone();
    }
    // Handle exceptions that may occur during the conversion process
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

// Getting the ditance to the obstacles with Front Left TOF Sensor
void rangeFLCallback(const sensor_msgs::RangeConstPtr &range)
{
    rangeFL = range->range;
    // ROS_INFO("rangeFL: %f", rangeFL);
}

// Getting the ditance to the obstacles with Front Right TOF Sensor
void rangeFRCallback(const sensor_msgs::RangeConstPtr &range)
{
    rangeFR = range->range;
    // ROS_INFO("rangeFR: %f", rangeFR);
} 

// Getting the distance to the closest obstacle with the LiDAR Sensor
void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_range)
{
    // The below lines compute the smallest distance to the obstacles:
    const std::vector<float>& ranges = scan_range->ranges;
    smallest_dist = *std::min_element(ranges.begin(), ranges.end());
    // ROS_INFO("Smallest distance is: %f", smallest_dist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_following_and_visualization_node");  // Initialize the ROS node: "object_following_and_visualization_node"
    ros::NodeHandle nh; // Create a NodeHandle for this node
   
    // Retrieve the lidarParam value from the parameter server
    if (!nh.getParam("lidarParam", lidarParam_variable))
    {
        ROS_WARN("Could not retrieve lidarParam, setting default to false.");
        lidarParam_variable = false;
    }

    // Subscribe to the "/objects" topic to receive object detection data   
    ros::Subscriber sub = nh.subscribe("/objects", 1, objectCallback);

    // Subscribe to the camera image topic to get image data in the form of a ROS sensor_msgs/Image message.
    ros::Subscriber image_sub = nh.subscribe("camera/color/image_raw", 1, imageCallback);
    
    // Advertise the "/cmd_vel" topic to publish Twist messages for controlling the robot    
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Subscribe to the range topic to get distance of the object.
    ros::Subscriber rangeFL_sub = nh.subscribe("/range/fl", 1, rangeFLCallback);

    // Subscribe to the range topic to get distance of the object.
    ros::Subscriber rangeFR_sub = nh.subscribe("/range/fr", 1, rangeFRCallback);

    // Subscribe to the scan topic to get distance of the object.
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::Rate loop_rate(30); // Loop rate of 30 Hz

    // Process incoming data and sleep to maintain loop rate 
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
