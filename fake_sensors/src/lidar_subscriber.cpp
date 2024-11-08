#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

typedef sensor_msgs::msg::LaserScan LaserMsg;

class LidarSubscriber: public rclcpp::Node
{
public:
    LidarSubscriber();

private:
    rclcpp::Subscription<LaserMsg>::SharedPtr _lidar_scan_subscribtion;

private:
    void _lidar_scan_callback(LaserMsg::UniquePtr msg);
    std::string _lidar_data_to_string(std::vector<float> data);

};

LidarSubscriber::LidarSubscriber(): Node("lidar_sub")
{
    
    _lidar_scan_subscribtion = this->create_subscription<LaserMsg>(
        "lidar_scan",
        10,
        std::bind(&LidarSubscriber::_lidar_scan_callback, this, std::placeholders::_1)
    );
}


std::string LidarSubscriber::_lidar_data_to_string(std::vector<float> data){
    std::string result= "";
    for(long unsigned int i=0;i<data.size();i++){
        result+= std::to_string(data[i]);
        if(i!=data.size()-1)
            result+=',';
    }
    return result;
}

void LidarSubscriber::_lidar_scan_callback(LaserMsg::UniquePtr msg)
{
    RCLCPP_INFO(this->get_logger(), "RECEIVED - %s", _lidar_data_to_string(msg->ranges).c_str());
}

//=================================================================================================
//                                            MAIN 
//=================================================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LidarSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
