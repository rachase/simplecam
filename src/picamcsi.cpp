#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class SimpleCam : public rclcpp::Node
{
public:
  SimpleCam()
      : Node("SimpleCam")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    timer_ = this->create_wall_timer(1.666ms, std::bind(&SimpleCam::read, this));
    setup_stream();
  }

  std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
  {
    return "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  }

  void setup_stream()
  {
    pipeline = gstreamer_pipeline(capture_width, 
                                  capture_height,
                                  display_width,
                                  display_height,
                                  framerate,
                                  flip_method);

    RCLCPP_INFO(this->get_logger(), "Using pipeline: \n\t%s",pipeline.c_str());

    cap_.open(pipeline, cv::CAP_GSTREAMER);
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    }

  }

  void read()
  {
    if (!cap_.read(frame_))
    {
      RCLCPP_WARN(this->get_logger(),"Capture read error");
    }
    else
    {
      // this is gross, but it seems to be working.  
      // Convert the OpenCV image to a ROS Image message
      auto msg_ = cv_bridge::CvImage(hdr_, "bgr8", frame_).toImageMsg();
      msg_->header.stamp = this->get_clock()->now();
      publisher_->publish(*msg_);
      //RCLCPP_INFO(this->get_logger(),"Read Message");
    }

    //cap.release();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  //sensor_msgs::msg::Image msg_;
  std_msgs::msg::Header hdr_;

  //for image data
  cv::VideoCapture cap_;
  cv::Mat frame_;

  std::string pipeline; 

  //parameters we are going to need
  int capture_width = 1280;
  int capture_height = 720;
  int display_width = 1280;
  int display_height = 720;
  int framerate = 60;
  int flip_method = 2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleCam>());
  rclcpp::shutdown();
  return 0;
}