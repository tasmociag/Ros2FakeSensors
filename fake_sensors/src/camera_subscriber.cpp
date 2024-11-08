#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

typedef sensor_msgs::msg::Image ImageMsg;

class CameraSubscriber: public rclcpp::Node
{
public:
    CameraSubscriber();
public:
    std::string mp4_file_name = "file.mp4";

private:
    rclcpp::Subscription<ImageMsg>::SharedPtr _camera_scan_subscribtion;

private:
    void _camera_scan_callback(ImageMsg::UniquePtr msg);
    std::string _camera_data_to_string(std::vector<uint8_t> data);

};

CameraSubscriber::CameraSubscriber(): Node("camera_sub")
{
    this->declare_parameter<std::string>("mp4_file_name", "file.mp4");
    mp4_file_name= this->get_parameter("mp4_file_name").as_string();
    RCLCPP_INFO(this->get_logger(), "Parameter mp4_file_name= %s", mp4_file_name.c_str());
    
    _camera_scan_subscribtion = this->create_subscription<ImageMsg>(
        "camera_scan",
        10,
        std::bind(&CameraSubscriber::_camera_scan_callback, this, std::placeholders::_1)
    );
}


std::string CameraSubscriber::_camera_data_to_string(std::vector<uint8_t> data){
    std::string result= "";
    for(long unsigned int i=0;i<data.size();i++){
        result+= std::to_string(data[i]);
        if(i!=data.size()-1)
            result+=',';
    }
    return result;
}

void CameraSubscriber::_camera_scan_callback(ImageMsg::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), "RECEIVED - %s", _camera_data_to_string(msg->data).c_str());
}

//=================================================================================================
//                                            MAIN 
//=================================================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CameraSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
