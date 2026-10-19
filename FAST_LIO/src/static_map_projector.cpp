#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "static_map_projection.hpp"

class StaticMapProjector:public rclcpp::Node {
    StaticMapProjection projection_;
    pcl::PointCloud<pcl::PointXYZI> map_;
    std_msgs::msg::Header header_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_,terrain_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double resolution_;
    bool dirty_=false;
public:
    StaticMapProjector():Node("static_map_projector") {
        resolution_=declare_parameter<double>("resolution",.05);
        if(!std::isfinite(resolution_)||resolution_<.01||resolution_>1.)
            throw std::invalid_argument("Static projected map resolution must be in [0.01, 1.0] m");
        pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/projected_map",rclcpp::QoS(1).transient_local().reliable());
        map_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/Laser_map",rclcpp::QoS(1).reliable(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
                if(msg->header.frame_id!="odom") {RCLCPP_ERROR(get_logger(),"Static map must be in odom frame");return;}
                pcl::fromROSMsg(*msg,map_);header_=msg->header;dirty_=true;
            });
        terrain_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map_ext",rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
                if(msg->header.frame_id!="odom")return;
                pcl::PointCloud<pcl::PointXYZI> terrain;pcl::fromROSMsg(*msg,terrain);
                projection_.terrain(terrain);dirty_=true;
            });
        timer_=create_wall_timer(std::chrono::seconds(1),[this](){publish();});
    }
    void publish() {
        if(!dirty_)return;
        StaticMapProjection::Grid grid;
        grid.resolution=resolution_;
        try {
            if(!projection_.project(map_,grid)) {
                // Clear the latched map as well when the last static voxel disappears.
                if(map_.empty()) {
                    nav_msgs::msg::OccupancyGrid msg;msg.header=header_;msg.header.frame_id="map";
                    msg.info.resolution=resolution_;msg.info.width=msg.info.height=1;
                    msg.info.origin.orientation.w=1.;msg.data={-1};pub_->publish(msg);
                }
                return;
            }
        }catch(const std::exception &e){RCLCPP_ERROR(get_logger(),"%s",e.what());return;}
        nav_msgs::msg::OccupancyGrid msg;msg.header=header_;msg.header.frame_id="map";
        msg.info.map_load_time=header_.stamp;msg.info.resolution=grid.resolution;
        msg.info.width=grid.width;msg.info.height=grid.height;
        msg.info.origin.position.x=grid.x;msg.info.origin.position.y=grid.y;
        msg.info.origin.orientation.w=1.;msg.data=std::move(grid.data);
        pub_->publish(msg);dirty_=false;
    }
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<StaticMapProjector>());rclcpp::shutdown();}
