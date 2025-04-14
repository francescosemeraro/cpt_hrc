#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <franka_msgs/action/grasp.hpp>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <rclcpp/qos.hpp>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>


namespace action_trajectory_selector{
class PandaMotionController : public rclcpp::Node {
public:
    using Grasp = franka_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

    /// Constructor
    explicit PandaMotionController(const rclcpp::NodeOptions &options) : Node("pose_commander", options),
                                        node_(std::make_shared<rclcpp::Node>("example_group_node")),
                                        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
                                        //, last_executed_letter_('\0'), current_letter_('\0') {
    
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/command", rclcpp::QoS(1), std::bind(&PandaMotionController::executeCommand, this, std::placeholders::_1));
        
        //subscription_2 = this->create_subscription<std_msgs::msg::Bool>(
        //    "/query", rclcpp::QoS(1), std::bind(&PandaMotionController::executeCommand, this, std::placeholders::_1));
        
        gripper_client_ = rclcpp_action::create_client<Grasp>(this, "/fr3_gripper/grasp");
        
        //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PandaMotionController::checkAndExecute, this));

        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "fr3_manipulator");

        move_group_interface_->setPlannerId("PRMkConfigDefault");
        move_group_interface_->setMaxVelocityScalingFactor(1.0);  // 1.0 means 100% of the max velocity
        move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 1.0 means 100% of the max acceleration
        
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        pick_poses_ = {
            {'L', {-1.13795467,  0.94422313,  0.09250245, -1.43989663, -0.14835299,  2.39808239,  -0.16406095}},
            {'K', {-1.08210414,  0.73652894,  0.09250245, -1.85877565, -0.17453293,  2.60577657, -0.05235988}},
            {'J', {-1.00007366,  0.58643063,  0.09250245, -2.17293492, -0.21816616,  2.76460154,  0.09075712}},
            {'I', {-0.89186325,  0.47822022,  0.09075712, -2.41728101, -0.30194196,  2.89724656,  0.29496064}},
            {'H', {-0.75572757,  0.40317106,  0.08552113, -2.59879526, -0.46774824,  2.994985  ,0.60039326}},
            {'G', {-0.40142573,  0.66671577,  0.08726646, -2.00014732, -0.18849556,  2.65988178,  0.64751715}},
            {'F', {-0.23911011,  0.63879051,  0.08726646, -2.07868714, -0.20245819,  2.70351501,  0.82728607}},
            {'E', {-0.06632251,  0.62831853,  0.08726646, -2.0943951 , -0.20071286,  2.70700567,  1.00007366}},
            {'D', {0.10995574,  0.65100781,  0.08726646, -2.04727121, -0.18675023,  2.67558974,  1.16064395}},
            {'C', {0.3403392 ,  0.52185345,  0.08552113, -2.33350521, -0.22514747,  2.82568806,  1.44338729}},
            {'B', {0.51661746,  0.60213859,  0.08552113, -2.1537363 , -0.17627825,  2.72445896,   1.56556034}},
            {'A', {0.66147979,  0.71733032,  0.08552113, -1.9093902 , -0.14486233,  2.59181394,  1.66678944}}
        };

    }
    
private:

    
    void executeCommand(const std_msgs::msg::String::SharedPtr msg){

        if (!msg->data.empty()) {
            std::string command = msg->data.c_str();
            
            if (command == "B1"){
                current_letter_ = 'A';
            }
            else if (command =="B2"){
                current_letter_ = 'B';
            }
            else if (command =="B3"){
                current_letter_ = 'C';
            }
            else if (command =="B4"){
                current_letter_ = 'D';
            }
            else if (command =="L12"){
                current_letter_ = 'E';
            }
            else if (command =="L23"){
                current_letter_ = 'F';
            }
            else if (command =="L34"){
                current_letter_ = 'G';
            }
            else if (command =="L14"){
                current_letter_ = 'H';
            }
            else if (command =="T1"){
                current_letter_ = 'I';
            }
            else if (command =="T2"){
                current_letter_ = 'J';
            }
            else if (command =="T3"){
                current_letter_ = 'K';
            }
            else if (command =="T4"){
                current_letter_ = 'L';
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("panda_motion"), "Invalid command");
            }
        }
        else{
            return;
        }
        last_executed_letter_ = current_letter_;
        
        // Define the initial position (original_pose)
        std::vector<double> original_pose = {0.00174533, -0.696132, 0.00174533, -2.35504, -0.00349066, 1.57313, 0.79724};
        
        // Define the hand-over pose
        std::vector<double> hand_over_pose = {0.887291, 0.311029, -0.303998, -1.931102, -0.158394, 3.859018, 0.969510};
        
        std::vector<std::vector<double>> waypoints;
        
        // Check if the current letter is one of the first 6 keys of pick_poses_
        if (pick_poses_.find(current_letter_) != pick_poses_.end()) {
            
            
            // Add the pick pose corresponding to the current letter
            waypoints.push_back(pick_poses_[current_letter_]);
            
            // If it's one of the first 6 pick poses, move to original position first, then hand-over pose
            //if (current_letter_ == 'A' || current_letter_ == 'B' || current_letter_ == 'C' || current_letter_ == 'D' ||
            //    current_letter_ == 'E' || current_letter_ == 'F') {
            //    // Add original position to the waypoints first
            //    waypoints.push_back(original_pose);
            //}// Add hand-over pose
            
            waypoints.push_back(hand_over_pose);
            
            // Add original pose to complete the cycle
            waypoints.push_back(original_pose);
        } else {
            // If it's not one of the first 6, just go to the pick pose and hand-over pose directly
            waypoints.push_back(pick_poses_[current_letter_]);
            waypoints.push_back(hand_over_pose);
            waypoints.push_back(original_pose);
        }

        // Execute the waypoints
        for (size_t i = 0; i < waypoints.size(); ++i) {
            move_group_interface_->setJointValueTarget(waypoints[i]);
            move_group_interface_->move();
            
            if (i == 0) {
                sendGraspCommand(0.02, 60.0);
            }
            if (i == waypoints.size()-2) {
                sendGraspCommand(0.08, 60.0);
            }

            // Wait for 1 second after reaching the hand-over pose (index 2 in the waypoints)
            if (i == 0) {
                RCLCPP_INFO(rclcpp::get_logger("panda_motion"), "Reached rod, closing gripper...");
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Delay for 1 second
            }
            else if (i == waypoints.size()-2) {
                RCLCPP_INFO(rclcpp::get_logger("panda_motion"), "Reached hand-over pose, waiting for 1 second to release");
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Delay for 1 second
            }
        }
    }
    

    void sendGraspCommand(double width, double force) {
        auto goal_msg = Grasp::Goal();
        goal_msg.width = width;
        goal_msg.epsilon.inner = 0.005;
        goal_msg.epsilon.outer = 0.005;
        goal_msg.speed = 0.1;
        goal_msg.force = force;

        auto send_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleGrasp::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("gripper_client"), "Grasp successful.");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("gripper_client"), "Grasp failed.");
            }
        };

        gripper_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_2;
    rclcpp_action::Client<Grasp>::SharedPtr gripper_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    std::unordered_map<char, std::vector<double>> pick_poses_;
    char last_executed_letter_;
    char current_letter_;
};
}

/*
PandaMotionController::PandaMotionController(const rclcpp::NodeOptions &options) : Node("pose_commander", options),
                         panda_motion_controller               node_(std::make_shared<rclcpp::Node>("example_group_node")),
                                        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
                                        //, last_executed_letter_('\0'), current_letter_('\0') {
    
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/marker_sequence", rclcpp::QoS(1), std::bind(&PandaMotionController::updateCommand, this, std::placeholders::_1));
    
    gripper_client_ = rclcpp_action::create_client<Grasp>(this, "/franka_gripper/grasp");
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PandaMotionController::checkAndExecute, this));

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");

    //move_group_interface_->setPlannerId("PRMkConfigDefault");
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

    pick_poses_ = {
        {'A', {-1.13795467,  0.94422313,  0.09250245, -1.43989663, -0.14835299,  2.39808239,  -0.16406095}},
        {'B', {-1.08210414,  0.73652894,  0.09250245, -1.85877565, -0.17453293,  2.60577657, -0.05235988}},
        {'C', {-1.00007366,  0.58643063,  0.09250245, -2.17293492, -0.21816616,  2.76460154,  0.09075712}},
        {'D', {-0.89186325,  0.47822022,  0.09075712, -2.41728101, -0.30194196,  2.89724656,  0.29496064}},
        {'E', {-0.75572757,  0.40317106,  0.08552113, -2.59879526, -0.46774824,  2.994985  ,0.60039326}},
        {'F', {-0.40142573,  0.66671577,  0.08726646, -2.00014732, -0.18849556,  2.65988178,  0.64751715}},
        {'G', {-0.23911011,  0.63879051,  0.08726646, -2.07868714, -0.20245819,  2.70351501,  0.82728607}},
        {'H', {-0.06632251,  0.62831853,  0.08726646, -2.0943951 , -0.20071286,  2.70700567,  1.00007366}},
        {'I', {0.10995574,  0.65100781,  0.08726646, -2.04727121, -0.18675023,  2.67558974,  1.16064395}},
        {'J', {0.3403392 ,  0.52185345,  0.08552113, -2.33350521, -0.22514747,  2.82568806,  1.44338729}},
        {'K', {0.51661746,  0.60213859,  0.08552113, -2.1537363 , -0.17627825,  2.72445896,   1.56556034}},
        {'L', {0.66147979,  0.71733032,  0.08552113, -1.9093902 , -0.14486233,  2.59181394,  1.66678944}}
    };

};
*/
RCLCPP_COMPONENTS_REGISTER_NODE(action_trajectory_selector::PandaMotionController)
/*
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<PandaMotionController>());
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto trajectory_selector = std::make_shared<PandaMotionController>(node_options);

    rclcpp::spin(trajectory_selector);
    rclcpp::shutdown();
    return 0;
}
*/