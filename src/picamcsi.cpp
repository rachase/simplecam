#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"

class SimpleCam : public rclcpp::Node
{
public:
  SimpleCam()
      : Node("SimpleCam")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  }

  std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
  {
    return "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
  }

  void setup()
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
    if (!cap_.read(img_))
    {
      RCLCPP_WARN(this->get_logger(),"Capture read error");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),"Read Message");
    }

    // std::cout << "Frame size: " << img.cols << " x " << img.rows << std::endl;
    // std::cout << "Frame type: " << img.type() << std::endl;
    // // Save the frame to a file
    // std::ostringstream filename;
    // filename << "frame_" << std::setw(5) << std::setfill('0') << frameNumber << ".bmp";
    // cv::imwrite(filename.str(), img);
    // std::cout << "Saved: " << filename.str() << std::endl;

    //cap.release();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  cv::Mat img_;

  std::string pipeline; 

  int frameNumber = 0;
  int capture_width = 1280;
  int capture_height = 720;
  int display_width = 1280;
  int display_height = 720;
  int framerate = 30;
  int flip_method = 2;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleCam>());
  rclcpp::shutdown();
  return 0;
}