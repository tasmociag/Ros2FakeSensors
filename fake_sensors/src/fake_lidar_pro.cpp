//=================================================================================================
//                                          INCLUDES
//=================================================================================================
#include <chrono>
#include <memory>
#include <cmath>
#include <utility>
#include <fstream>
#include <random>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


//=================================================================================================
//                                        LIDAR CONFIG 
//=================================================================================================
class LidarConfig
{
public:
    const char* PARAM_MIN_RANGE = "min_range";
    const char* PARAM_MAX_RANGE = "max_range";
    const char* PARAM_MIN_ANGLE = "min_angle";
    const char* PARAM_MAX_ANGLE = "max_angle";
    const char* PARAM_SAMPLE_COUNT = "sample_count";
    const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";

    const double DEFAULT_MIN_RANGE = 0.2;
    const double DEFAULT_MAX_RANGE = 2.0;
    const double DEFAULT_MIN_ANGLE = -M_PI;
    const double DEFAULT_MAX_ANGLE = M_PI;
    const int DEFAULT_SAMPLE_COUNT = 360;
    const double DEFAULT_SAMPLING_FREQUENCY = 10.0;

    const char* DESCRIPTION_MIN_RANGE =
    "minimum range value [m]";
    const char* DESCRIPTION_MAX_RANGE =
    "maximum range value [m]";
    const char* DESCRIPTION_MIN_ANGLE =
    "start angle of the scan [rad]";
    const char* DESCRIPTION_MAX_ANGLE =
    "end angle of the scan [rad]";
    const char* DESCRIPTION_SAMPLE_COUNT =
    "Number of samples per full laser scan";
    const char* DESCRIPTION_SAMPLING_FREQUENCY =
    "Number of full Scans per second.";

public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);

    double get_scan_step() const;
    int get_scan_period_ms() const;
    double get_scaled_sample(double scale) const;


};
    
double LidarConfig::get_scaled_sample(double scale) const
{
    return range.first + (range.second - range.first) * scale;
}

int LidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double LidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
}

void LidarConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,
                this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());
}

//=================================================================================================
//                                         FAKE LIDAR NODE 
//=================================================================================================
class FakeLidar: public rclcpp::Node
{
public:
    FakeLidar();

public: 
    int csvLine=0;
    std::string lidar_csv_file = "lidar.csv";

private:
    LidarConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;
    std::vector<std::vector<float>> _csv_ranges;
    double stddev;

private:
    void _publish_fake_scan();
    void _read_csv_file();
    std::vector<float> _split_string(std::string s);
    std::vector<float> _add_gaussian_noise(std::vector<float>);
    std::vector<float> _filtr_ranges(std::vector<float>);
};

FakeLidar::FakeLidar(): Node("fake_lidar")
{

    this->declare_parameter<double>("stddev", 0);
    stddev = this->get_parameter("stddev").as_double();
    RCLCPP_INFO(this->get_logger(), "Parameter stddev = %f", stddev);

    this->declare_parameter<std::string>("lidar_csv_file", "lidar.csv");
    lidar_csv_file= this->get_parameter("lidar_csv_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Parameter lidar_csv_file = %s", lidar_csv_file.c_str());

    if(this->stddev == 0){
        RCLCPP_ERROR(this->get_logger(), "standard deviation equals 0");
    }


    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar_scan",10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&FakeLidar::_publish_fake_scan, this)
    );
}



std::vector<float> FakeLidar::_split_string(std::string s){
    std::stringstream ss(s);
    std::string range;
    std::vector<float> result;

    while(!ss.eof()){
        getline(ss,range,';');
        result.push_back(std::stof(range));
    }

    return result;
}

void FakeLidar::_read_csv_file(){

    std::string rangesFromCsv = "";
    _csv_ranges.clear();

    std::ifstream file;

    std::string current_path = std::filesystem::current_path().generic_string()+"/src/"+lidar_csv_file;
    file.open(current_path);
    if(!file.is_open())
        RCLCPP_ERROR(this->get_logger(), "file is not open");

    while(file >> rangesFromCsv){
        _csv_ranges.push_back(_split_string(rangesFromCsv));
    }
}

std::vector<float> FakeLidar::_add_gaussian_noise(std::vector<float> tab){
    

    const double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, this->stddev);

    for (long unsigned int i=0;i<tab.size();i++) {
        float before = tab[i];
        tab[i] =  tab[i] + dist(generator);
        float after = tab[i];
        RCLCPP_INFO(this->get_logger(), "%f = %f", before, after);

    }

    return tab;
}

std::vector<float> FakeLidar::_filtr_ranges(std::vector<float> tab){

    std::vector<float> new_ranges; 
    for(long unsigned int i=0;i<tab.size();i++){
        if(tab[i] >= _config.range.first && tab[i] <= _config.range.second)
            new_ranges.push_back(tab[i]);
    }
    return new_ranges;

}
    
void FakeLidar::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::LaserScan();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.angle_increment = _config.get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    
    _read_csv_file();
    std::vector<float> ranges= _csv_ranges[csvLine];
    
    if(this->stddev > 0){
        ranges = _add_gaussian_noise(ranges);
    }
    ranges = _filtr_ranges(ranges);

    csvLine= (csvLine+1)%_csv_ranges.size();

    msg.ranges = ranges;

    _scan_publisher->publish(msg);

}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

