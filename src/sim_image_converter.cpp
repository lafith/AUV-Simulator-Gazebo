/*
This is a template that can be used to image conversion in future.
For example: to add noise to images.
If that need arises, change the topic name of gazebo_camera_plugin to /front_camera/image_raw
and /bottom_camera/image_raw.
Also uncomment last two lines in CMakeLists.txt
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SimImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber frontCameraSub;
  image_transport::Publisher frontCameraPub;
  image_transport::Subscriber bottomCameraSub;
  image_transport::Publisher bottomCameraPub;
public: SimImageConverter():it_(nh_){
  frontCameraSub=it_.subscribe("/front_camera/image_raw",1,&SimImageConverter::frontCameraCallback,this);
  frontCameraPub=it_.advertise("/front_camera/image_rect_color",1);
  bottomCameraSub=it_.subscribe("/bottom_camera/image_raw",1,&SimImageConverter::bottomCameraCallback,this);
  bottomCameraPub=it_.advertise("/bottom_camera/image_rect_color",1);
  cv::namedWindow("front_camera");
  cv::namedWindow("bottom_camera");
}

~SimImageConverter(){
  cv::destroyWindow("front_camera");
  cv::destroyWindow("bottom_camera");
}

void bottomCameraCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


              //Manipulation can be done here

// Update GUI Window
 cv::imshow("bottom_camera", cv_ptr->image);
 cv::waitKey(3);
  bottomCameraPub.publish(cv_ptr->toImageMsg());
}

void frontCameraCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

                  //Manipulation can be done here

// Update GUI Window
 cv::imshow("front_camera", cv_ptr->image);
 cv::waitKey(3);
  frontCameraPub.publish(cv_ptr->toImageMsg());
}
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_image_converter");
  SimImageConverter ic;
  ros::spin();
  return 0;
}
