
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "BuoyDetector.hpp"
#include "ros/ros.h"
#include "rowbot_vision/BuoyDetect.h"

bool detect(rowbot_vision::BuoyDetect::Request &req, rowbot_vision::BuoyDetect::Response &res){

  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
    //std::vector<double> confidences  = {1};
    //std::vector<std::string> types  = {"red_buoy"};

    BuoyDetector bd;
    bd.classifyBuoys(cv_ptr->image,false);
    Object *obj = bd.getBestAtBearing(req.bearing,req.ObjInFront);
    cv::Mat m_mask = bd.getMask();
    cv_bridge::CvImagePtr out_image_ptr(new cv_bridge::CvImage);
    out_image_ptr->image = m_mask;
    out_image_ptr->encoding = "bgr8";
    sensor_msgs::Image wow = *out_image_ptr->toImageMsg();
    res.mask = wow;
    if (obj == NULL){
      std::cout << "Found nothing "  <<std::endl;
      res.types = {};
      res.confidences = {};
      //cv::imshow("Test", cv_ptr->image);
      //cv::imshow("Test", m_mask);
      //cv::waitKey(1);

      return true;
    }
    res.types = obj->m_types;
    res.confidences  = std::vector<double>(obj->m_confidences.begin(), obj->m_confidences.end());
    std::cout << "Gotr Types "  <<std::endl;
    //res.confidences = std::vector<double>(obj->m_confidences.begin(), obj->m_confidences.end());
    //std::cout << "Gotr Confidences "  <<std::endl;
    //cv::imshow("Test", cv_ptr->image);
    //cv::imshow("Test", m_mask);
    //cv::waitKey(1);

  return true;
}


int main(int argc, char* argv[]) {
  ros::init(argc,argv,"buoy_detector_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("detect_buoy",detect);
  ros::spin();
  return 0;
}
