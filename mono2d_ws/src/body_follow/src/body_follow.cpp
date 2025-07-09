#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

//判断是否有人物的标志
bool found = 0;

class BodyRoiSubscriber : public rclcpp::Node
{
public:
    BodyRoiSubscriber() : Node("body_follow")
    {
        subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "/hobot_mono2d_body_detection",
            10,
            std::bind(&BodyRoiSubscriber::topic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /hobot_mono2d_body_detection");


        // 发布速度控制命令
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "body follower node started.");

        //最开始先不进跟随逻辑
        found = false;
    }

private:
    void topic_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
        sensor_msgs::msg::RegionOfInterest largest_roi;
        
        //每次订阅都要处理一下
        uint32_t max_area = 0;
        uint32_t area =0;

        for (const auto &target : msg->targets){ 
            if (target.type != "person") 
                continue;
    
            for (const auto &roi : target.rois) {
                if (roi.type != "body") continue;
        
                const auto &rect = roi.rect;
                area = rect.width * rect.height;
        
                if (area > max_area) {
                max_area = area;
                largest_roi = rect;
            }

            if (max_area > 100*200) {
                found = true;
            }
            else
                found = false;
            }

        }

        if (max_area > 0) {
        int x1 = largest_roi.x_offset;
        int y1 = largest_roi.y_offset;
        int x2 = x1 + largest_roi.width;
        int y2 = y1 + largest_roi.height;
    
        // std::cout << "最大人体框四个坐标点为：" << std::endl;
        // std::cout << "左上: (" << x1 << ", " << y1 << ")" << std::endl;
        // std::cout << "右上: (" << x2 << ", " << y1 << ")" << std::endl;
        // std::cout << "右下: (" << x2 << ", " << y2 << ")" << std::endl;
        // std::cout << "左下: (" << x1 << ", " << y2 << ")" << std::endl;
        } else {
            RCLCPP_INFO(this->get_logger(), "No person detected!");
        }

        //需要填充的速度话题
        geometry_msgs::msg::Twist cmd_vel_msg;

        if (found && (max_area > 0)) {
            // 图像中心
            RCLCPP_INFO(this->get_logger(), "进入跟随逻辑!");
            RCLCPP_INFO(this->get_logger(), "max_area: %u", max_area);

            const int image_center_x = 320;
      
            // 框中心
            int roi_center_x = largest_roi.x_offset + largest_roi.width / 2;
            int error_x = roi_center_x - image_center_x;
      
            // 设置角速度：误差越大，转得越快
            double k_ang = 0.004;  // 角速度增益
            cmd_vel_msg.angular.z = -error_x * k_ang;  // 负号让小车朝人体方向转动
      
            // 设置线速度：框越小说明人越远，走得越快（限制最大/最小速度）
            double max_speed = 0.3;
            double min_speed = 0.1;
            double target_area = 350 * 400;  // 期望的框面积大小
            double area_ratio = static_cast<double>(target_area) / max_area;
      
            cmd_vel_msg.linear.x = std::clamp(area_ratio * max_speed, min_speed, max_speed);
      
            RCLCPP_INFO(this->get_logger(),
                        "Tracking person: center_x=%d, error=%d, angular.z=%.2f, linear.x=%.2f",
                        roi_center_x, error_x, cmd_vel_msg.angular.z, cmd_vel_msg.linear.x);

            cmd_vel_msg.linear.y  = 0.0, cmd_vel_msg.linear.z  = 0.0;
            cmd_vel_msg.angular.x = 0.0, cmd_vel_msg.angular.y = 0.0;

            // 框太大了要停止
            if(max_area>350 * 400){
                cmd_vel_msg.angular.z = 0;
                cmd_vel_msg.linear.x = 0;
                cmd_vel_msg.linear.y  = 0.0, cmd_vel_msg.linear.z  = 0.0;
                cmd_vel_msg.angular.x = 0.0, cmd_vel_msg.angular.y = 0.0;
            }
          } else {
            // 没检测到人，停止
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;

            cmd_vel_msg.linear.y  = 0.0, cmd_vel_msg.linear.z  = 0.0;
            cmd_vel_msg.angular.x = 0.0, cmd_vel_msg.angular.y = 0.0;

            RCLCPP_WARN(this->get_logger(), "The car is stoping!");
          }


        //发布速度
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
  
    //ai消息的订阅者
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr subscription_;
    //速度消息的发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyRoiSubscriber>());
    rclcpp::shutdown();
    return 0;
}
