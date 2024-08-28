/*
This ROS node draws a bounding box at a defined position in the image captured by the Robot.

The node performs the following functions:
1. **Subscribes to the `/camera/color/image_raw` topic, where it receives image data.

2. The received ROS image message is converted to a opencv image so that image processing techniques can be used.

Reference: https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
    }
    catch (cv_bridge::Exception &e)
    {
        // Handle exceptions that may occur during the conversion process
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Now: cv_ptr->image contains the captured image as a cv::Mat object
    
    // Draw a bounding box at a predefined location
    cv::Rect bounding_box(100, 100, 200, 150); // x, y, width, height random values
    cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2); // Green color bbox

    // Displaying the image with the bounding box
    cv::imshow("Captured Image with Bounding Box", cv_ptr->image);
    cv::waitKey(1); // 1 ms delay: Wait for a key press
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_and_visualize"); // Initialize the ROS node: "capture_and_visualize"
    ros::NodeHandle nh; // Create a NodeHandle for this node

    // Subscribe to the camera image topic to get image data in the form of a ROS sensor_msgs/Image message.
    ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);

    ros::spin(); // Keep the node running
}
