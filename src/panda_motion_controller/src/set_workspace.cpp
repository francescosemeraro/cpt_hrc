#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <unordered_map>
#include <chrono>
#include <thread>

class WorkspaceSetup : public rclcpp::Node {
public:
    explicit WorkspaceSetup(const rclcpp::NodeOptions &options) 
        : Node("pose_commander", options),
          node_(std::make_shared<rclcpp::Node>("example_group_node")),
                                        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
        
        // Define the home pose (original position)
        home_pose_ = {0.00174533, -0.696132, 0.00174533, -2.35504, -0.00349066, 1.57313, 0.79724};
        
        // Define the pick poses
        pick_poses_ = {
            {'A', {-1.13795467,  0.94422313,  0.09250245, -1.43989663, -0.14835299,  2.39808239,  -0.16406095}},
            {'B', {-1.08210414,  0.73652894,  0.09250245, -1.85877565, -0.17453293,  2.60577657, -0.05235988}},
            {'C', {-1.00007366,  0.58643063,  0.09250245, -2.17293492, -0.21816616,  2.76460154,  0.09075712}},
            {'D', {-0.89186325,  0.47822022,  0.09075712, -2.41728101, -0.30194196,  2.89724656,  0.29496064}},
            {'E', {-0.75572757,  0.40317106,  0.08552113, -2.59879526, -0.46774824,  2.994985, 0.60039326}},
            {'F', {-0.40142573,  0.66671577,  0.08726646, -2.00014732, -0.18849556,  2.65988178,  0.64751715}},
            {'G', {-0.23911011,  0.63879051,  0.08726646, -2.07868714, -0.20245819,  2.70351501,  0.82728607}},
            {'H', {-0.06632251,  0.62831853,  0.08726646, -2.0943951, -0.20071286,  2.70700567,  1.00007366}},
            {'I', {0.10995574,  0.65100781,  0.08726646, -2.04727121, -0.18675023,  2.67558974,  1.16064395}},
            {'J', {0.3403392,  0.52185345,  0.08552113, -2.33350521, -0.22514747,  2.82568806,  1.44338729}},
            {'K', {0.51661746,  0.60213859,  0.08552113, -2.1537363, -0.17627825,  2.72445896,  1.56556034}},
            {'L', {0.66147979,  0.71733032,  0.08552113, -1.9093902, -0.14486233,  2.59181394,  1.66678944}}
        };
        
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "fr3_manipulator");

        move_group_interface_->setPlannerId("PRMkConfigDefault");
        move_group_interface_->setMaxVelocityScalingFactor(1.0);  // 1.0 means 100% of the max velocity
        move_group_interface_->setMaxAccelerationScalingFactor(1.0);  // 1.0 means 100% of the max acceleration
        
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });
        // Start the motion
        moveToHomeAndPickPoses();
    }

private:
    void moveToHomeAndPickPoses() {
        std::vector<std::vector<double>> waypoints;
        
        // First go to home pose
        waypoints.push_back(home_pose_);

        // Then alternate between pick poses and home pose
        for (const auto& pair : pick_poses_) {
            waypoints.push_back(pair.second);
            waypoints.push_back(home_pose_);
        }

        // Execute the motion to all waypoints
        for (size_t i = 0; i < waypoints.size(); ++i) {
            move_group_interface_->setJointValueTarget(waypoints[i]);
            move_group_interface_->move();
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Adding a small delay between movements
        }

        // Shut down the node after completing the movement cycle
        rclcpp::shutdown();
    }

    // Home pose (original position)
    std::vector<double> home_pose_;
    
    // Pick poses
    std::unordered_map<char, std::vector<double>> pick_poses_;
    
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
};


//RCLCPP_COMPONENTS_REGISTER_NODE(set_workspace::workspaceSetup)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto workspace_starter = std::make_shared<WorkspaceSetup>(node_options);

    rclcpp::spin(workspace_starter);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
