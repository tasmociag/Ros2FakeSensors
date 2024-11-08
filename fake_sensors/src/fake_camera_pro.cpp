//=================================================================================================
//                                          INCLUDES
//=================================================================================================
#include <chrono>
#include <memory>
#include <cmath>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


//=================================================================================================
//                                        CAMERA CONFIG 
//=================================================================================================
class CameraConfig 
{
public:
    const char* PARAM_HEIGHT = "height";
    const char* PARAM_WIDTH = "width";
    const char* PARAM_ENCODING = "encoding";
    const char* PARAM_IS_BIGENDIAN = "is_bigendian";
    const char* PARAM_STEP = "step";

    //uintx_t nie dzialaja 

    const int DEFAULT_HEIGHT = 5;
    const int DEFAULT_WIDTH = 5;
    const std::string DEFAULT_ENCODING = "rgb8";
    const int  DEFAULT_IS_BIGENDIAN = 1;
    const int DEFAULT_STEP = 5;


    const char* DESCRIPTION_HEIGHT =
    "Height of an image";
    const char* DESCRIPTION_WIDTH =
    "Width of an image";
    const char* DESCRIPTION_ENCODING =
    "Code encoding of colors";
    const char* DESCRIPTION_IS_BIGENDIAN =
    "Is bigendian";
    const char* DESCRIPTION_STEP =
    "Full row length in bytes";

public:
    std::pair<int,int> size;
    std::string encoding;
    int is_bigendian;
    int step;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);
};
    

void CameraConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_HEIGHT;
    node->declare_parameter(PARAM_HEIGHT, DEFAULT_HEIGHT, descriptor);
    descriptor.description = DESCRIPTION_WIDTH;
    node->declare_parameter(PARAM_WIDTH, DEFAULT_WIDTH, descriptor);
    descriptor.description = DESCRIPTION_ENCODING;
    node->declare_parameter(PARAM_ENCODING, DEFAULT_ENCODING, descriptor);
    descriptor.description = DESCRIPTION_IS_BIGENDIAN;
    node->declare_parameter(PARAM_IS_BIGENDIAN, DEFAULT_IS_BIGENDIAN, descriptor);
    descriptor.description = DESCRIPTION_STEP;
    node->declare_parameter(PARAM_STEP, DEFAULT_STEP, descriptor);
}

void CameraConfig::update_parameters(rclcpp::Node *node)
{
    this->size.first = node->get_parameter(PARAM_HEIGHT).as_int();
    this->size.second = node->get_parameter(PARAM_WIDTH).as_int();
    this->encoding= node->get_parameter(PARAM_ENCODING).as_string();
    this->is_bigendian = node->get_parameter(PARAM_IS_BIGENDIAN).as_int();
    this->step = node->get_parameter(PARAM_STEP).as_int();
}

void CameraConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %u", PARAM_HEIGHT, this->size.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %u", PARAM_WIDTH, this->size.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_ENCODING, this->encoding.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %u", PARAM_IS_BIGENDIAN, this->is_bigendian);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %u", PARAM_STEP, this->step);
}

//=================================================================================================
//                                         FAKE CAMERA NODE 
//=================================================================================================
class FakeCamera: public rclcpp::Node
{
public:
    FakeCamera();

private:
    CameraConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;

private:
    void _publish_fake_scan();
    int _count_data_size();
};

FakeCamera::FakeCamera(): Node("fake_camera")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera_scan",10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&FakeCamera::_publish_fake_scan, this)
    );
}

int FakeCamera::_count_data_size(){
    return _config.size.first*_config.step;
}
    
void FakeCamera::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::Image();

    msg.header.stamp = this->now();
    msg.header.frame_id = "image_frame";

    msg.height = _config.size.first;
    msg.width = _config.size.second;
    msg.encoding = _config.encoding;
    msg.is_bigendian= _config.is_bigendian;
    msg.step = _config.step;

    int ranges_size = _count_data_size();
    std::vector<uint8_t> data(ranges_size);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0,255);

    for(int i=0; i<ranges_size; i++)
    {
        int random_range = distrib(gen);
        data[i] = random_range; 
    }

    // std::vector<unsigned char> image_data(ranges_size);
    // for (int i = 0; i < ranges_size; ++i) {
    //   image_data[i] = static_cast<unsigned char>(data[i]);
    // }
    // msg.data = image_data;
    //
    msg.data = data;

    _scan_publisher->publish(msg);
}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

