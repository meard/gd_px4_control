#ifndef GDCONTROL_CAM_HPP
#define GDCONTROL_CAM_HPP

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "camera_context.hpp"

namespace gd_video_cam
{

  class GDCameraNode : public rclcpp::Node
  {
    GDCameraContext cxt_;

    std::thread thread_;
    std::atomic<bool> canceled_;

    std::shared_ptr<cv::VideoCapture> capture_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int publish_fps_;
    rclcpp::Time next_stamp_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  public:
    explicit GDCameraNode(const rclcpp::NodeOptions &options);

    ~GDCameraNode() override;

  private:
    void validate_parameters();

    void loop();
  };

} // namespace gd_video_cam

#endif // GDCONTROL_CAM_HPP
