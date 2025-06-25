# Template Method Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Template Method Pattern là một mẫu thiết kế thuộc nhóm Behavioral Pattern, định nghĩa khung sườn của một thuật toán trong một phương thức, để các lớp con có thể ghi đè các bước cụ thể mà không thay đổi cấu trúc thuật toán. Trong ROS2 và robotics, pattern này thường được sử dụng cho:
- Robot behavior implementations
- Sensor processing pipelines
- Motion planning algorithms
- Control system frameworks
- Navigation strategies

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Tái sử dụng logic chung cho nhiều loại robots
- Customize các bước xử lý sensor data
- Thay đổi chiến lược điều khiển
- Mở rộng các thuật toán navigation
- Implement các biến thể của cùng một behavior

## 3. Giải pháp

Template Method Pattern giải quyết các vấn đề trên bằng cách:
1. Định nghĩa skeleton của thuật toán trong base class
2. Tách các bước thành các phương thức riêng biệt
3. Cho phép override các bước cụ thể
4. Giữ nguyên cấu trúc tổng thể

## 4. Ví dụ thực tế: Robot Navigation System

```cpp
// Abstract base class định nghĩa template method
class RobotNavigator {
public:
    virtual ~RobotNavigator() = default;

    // Template method định nghĩa thuật toán navigation
    bool navigateToGoal(const geometry_msgs::msg::PoseStamped& goal) {
        RCLCPP_INFO(logger_, "Starting navigation to goal");

        if (!validateGoal(goal)) {
            RCLCPP_ERROR(logger_, "Invalid goal pose");
            return false;
        }

        if (!checkSafety()) {
            RCLCPP_ERROR(logger_, "Safety check failed");
            return false;
        }

        auto path = planPath(goal);
        if (path.poses.empty()) {
            RCLCPP_ERROR(logger_, "Failed to plan path");
            return false;
        }

        return executePath(path);
    }

protected:
    // Các hook methods có thể được override bởi các lớp con
    virtual bool validateGoal(const geometry_msgs::msg::PoseStamped& goal) {
        // Default implementation
        return goal.header.frame_id != "";
    }

    virtual bool checkSafety() = 0;  // Pure virtual method
    virtual nav_msgs::msg::Path planPath(
        const geometry_msgs::msg::PoseStamped& goal) = 0;  // Pure virtual method
    virtual bool executePath(const nav_msgs::msg::Path& path) = 0;  // Pure virtual method

    rclcpp::Logger logger_ = rclcpp::get_logger("RobotNavigator");
};

// Concrete implementation cho Differential Drive Robot
class DiffDriveNavigator : public RobotNavigator {
public:
    explicit DiffDriveNavigator(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , tf_buffer_(node->get_clock())
        , tf_listener_(tf_buffer_) {
        
        // Initialize parameters
        max_velocity_ = node_->declare_parameter("max_velocity", 0.5);
        min_turning_radius_ = node_->declare_parameter("min_turning_radius", 0.3);
        
        // Create collision checker
        collision_checker_ = std::make_unique<CollisionChecker>(node_);
    }

protected:
    bool validateGoal(const geometry_msgs::msg::PoseStamped& goal) override {
        // First call parent's validation
        if (!RobotNavigator::validateGoal(goal)) {
            return false;
        }

        // Additional diff drive specific validation
        try {
            // Transform goal to robot frame
            auto goal_in_base = tf_buffer_.transform(
                goal, "base_link", tf2::durationFromSec(1.0));

            // Check if goal is reachable with diff drive constraints
            double distance = std::hypot(
                goal_in_base.pose.position.x,
                goal_in_base.pose.position.y);

            if (distance < min_turning_radius_) {
                RCLCPP_WARN(node_->get_logger(),
                    "Goal too close for minimum turning radius");
                return false;
            }

            return true;
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(),
                "Could not transform goal pose: %s", ex.what());
            return false;
        }
    }

    bool checkSafety() override {
        // Check for obstacles using laser scan
        return collision_checker_->isSafe();
    }

    nav_msgs::msg::Path planPath(
        const geometry_msgs::msg::PoseStamped& goal) override {
        
        nav_msgs::msg::Path path;
        path.header = goal.header;

        // Simple path planning for diff drive
        try {
            auto current_pose = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            // Generate path points considering differential drive constraints
            geometry_msgs::msg::PoseStamped pose;
            pose.header = goal.header;

            // Start point
            pose.pose = current_pose.transform;
            path.poses.push_back(pose);

            // Intermediate points (simplified)
            double dx = goal.pose.position.x - current_pose.transform.translation.x;
            double dy = goal.pose.position.y - current_pose.transform.translation.y;
            int steps = std::max(5, static_cast<int>(std::hypot(dx, dy) / 0.5));

            for (int i = 1; i < steps; ++i) {
                pose.pose.position.x = current_pose.transform.translation.x + (dx * i) / steps;
                pose.pose.position.y = current_pose.transform.translation.y + (dy * i) / steps;
                
                // Interpolate orientation
                double t = static_cast<double>(i) / steps;
                tf2::Quaternion q1, q2, q_interp;
                tf2::fromMsg(current_pose.transform.rotation, q1);
                tf2::fromMsg(goal.pose.orientation, q2);
                q_interp = q1.slerp(q2, t);
                pose.pose.orientation = tf2::toMsg(q_interp);

                path.poses.push_back(pose);
            }

            // End point
            pose.pose = goal.pose;
            path.poses.push_back(pose);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to get current pose: %s", ex.what());
            return path;
        }

        return path;
    }

    bool executePath(const nav_msgs::msg::Path& path) override {
        RCLCPP_INFO(node_->get_logger(), "Executing path with %zu points",
            path.poses.size());

        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto& current = path.poses[i-1];
            const auto& next = path.poses[i];

            // Calculate velocity commands
            double dx = next.pose.position.x - current.pose.position.x;
            double dy = next.pose.position.y - current.pose.position.y;
            double distance = std::hypot(dx, dy);
            double angle = std::atan2(dy, dx);

            // Create velocity command
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = std::min(max_velocity_,
                distance / node_->get_parameter("control_frequency").as_double());
            cmd_vel.angular.z = angle;

            // Publish velocity command
            vel_publisher_->publish(cmd_vel);

            // Wait for movement to complete
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            // Check if we should stop
            if (!checkSafety()) {
                RCLCPP_WARN(node_->get_logger(), "Safety check failed during execution");
                return false;
            }
        }

        // Stop the robot
        geometry_msgs::msg::Twist stop_cmd;
        vel_publisher_->publish(stop_cmd);

        return true;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    double max_velocity_;
    double min_turning_radius_;
};

// Concrete implementation cho Omnidirectional Robot
class OmniNavigator : public RobotNavigator {
public:
    explicit OmniNavigator(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , tf_buffer_(node->get_clock())
        , tf_listener_(tf_buffer_) {
        
        // Initialize parameters
        max_velocity_ = node_->declare_parameter("max_velocity", 0.5);
        
        // Create collision checker
        collision_checker_ = std::make_unique<CollisionChecker>(node_);
    }

protected:
    bool checkSafety() override {
        // Omni-specific safety checks
        return collision_checker_->isOmniSafe();
    }

    nav_msgs::msg::Path planPath(
        const geometry_msgs::msg::PoseStamped& goal) override {
        
        nav_msgs::msg::Path path;
        path.header = goal.header;

        try {
            auto current_pose = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            // For omni robot, we can move directly to the goal
            geometry_msgs::msg::PoseStamped pose;
            pose.header = goal.header;

            // Start point
            pose.pose = current_pose.transform;
            path.poses.push_back(pose);

            // Direct path to goal
            pose.pose = goal.pose;
            path.poses.push_back(pose);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to get current pose: %s", ex.what());
        }

        return path;
    }

    bool executePath(const nav_msgs::msg::Path& path) override {
        if (path.poses.size() < 2) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid path");
            return false;
        }

        const auto& goal = path.poses.back();

        try {
            auto current = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            // Calculate direct movement
            double dx = goal.pose.position.x - current.transform.translation.x;
            double dy = goal.pose.position.y - current.transform.translation.y;
            
            // For omni robot, we can move in any direction
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = dx * max_velocity_;
            cmd_vel.linear.y = dy * max_velocity_;

            // Execute movement
            vel_publisher_->publish(cmd_vel);
            rclcpp::sleep_for(std::chrono::seconds(1));

            // Stop the robot
            geometry_msgs::msg::Twist stop_cmd;
            vel_publisher_->publish(stop_cmd);

            return true;
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to get current pose: %s", ex.what());
            return false;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    double max_velocity_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Template Method Pattern trong một ROS2 node:

```cpp
class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        // Determine robot type from parameter
        auto robot_type = declare_parameter("robot_type", "diff_drive");

        // Create appropriate navigator
        if (robot_type == "diff_drive") {
            navigator_ = std::make_unique<DiffDriveNavigator>(
                shared_from_this());
        } else if (robot_type == "omni") {
            navigator_ = std::make_unique<OmniNavigator>(
                shared_from_this());
        } else {
            throw std::runtime_error("Unknown robot type: " + robot_type);
        }

        // Create action server
        action_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&NavigationNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationNode::handleCancel, this, std::placeholders::_1),
            std::bind(&NavigationNode::handleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Navigation node initialized with %s robot",
            robot_type.c_str());
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
        
        RCLCPP_INFO(get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>
        goal_handle) {
        
        RCLCPP_INFO(get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>
        goal_handle) {
        
        // Spawn thread to execute navigation
        std::thread{std::bind(&NavigationNode::executeNavigation, this, std::placeholders::_1),
                   goal_handle}.detach();
    }

    void executeNavigation(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>
        goal_handle) {
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

        // Execute navigation using template method
        if (navigator_->navigateToGoal(goal->pose)) {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
        } else {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "Navigation failed");
        }
    }

    std::unique_ptr<RobotNavigator> navigator_;
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr action_server_;
};
```

## 6. Lợi ích

1. **Code Reuse**:
   - Tránh duplicate code
   - Chia sẻ logic chung
   - Dễ maintain

2. **Flexibility**:
   - Dễ thêm implementations mới
   - Customize từng bước
   - Giữ cấu trúc chung

3. **Consistency**:
   - Đảm bảo các bước cần thiết
   - Thống nhất workflow
   - Dễ debug

## 7. Khi nào sử dụng

- Có thuật toán với cấu trúc cố định
- Cần customize một số bước
- Muốn tránh duplicate code
- Có nhiều variants của cùng process
- Cần đảm bảo consistency

## 8. Lưu ý

1. Thiết kế:
   - Xác định rõ skeleton
   - Tách biệt các bước
   - Cân nhắc default implementations
   - Document rõ constraints

2. Implementation:
   - Tránh quá nhiều abstract methods
   - Xử lý errors hợp lý
   - Consider hooks points
   - Maintain SOLID principles

3. Trong ROS2:
   - Handle timeouts
   - Consider preemption
   - Thread safety
   - Resource management 