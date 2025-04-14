#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

class SpacebarPublisher : public rclcpp::Node {
public:
    SpacebarPublisher() : Node("spacebar_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/query", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&SpacebarPublisher::check_keypress, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool last_pressed_ = false;

    void check_keypress() {
        char key = get_keypress();
        
        if (key == ' ' && !last_pressed_) { // Spacebar detected & was not pressed before
            last_pressed_ = true;
            std_msgs::msg::Bool msg;
            msg.data = true;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Spacebar pressed: Published True");
        } 
        else if (key != ' ') {
            last_pressed_ = false; // Reset when the key is released
        }
    }

    char get_keypress() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        int bytesRead = read(STDIN_FILENO, &ch, 1);
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return (bytesRead > 0) ? ch : '\0';
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpacebarPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
