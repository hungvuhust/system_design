# Bridge Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Bridge Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, tách rời abstraction (giao diện) khỏi implementation (triển khai), cho phép hai phần có thể thay đổi độc lập. Pattern này đặc biệt hữu ích trong robotics khi:
- Phát triển drivers cho nhiều loại hardware
- Xử lý nhiều loại communication protocols
- Implement các thuật toán navigation khác nhau
- Hỗ trợ nhiều platforms khác nhau
- Quản lý các subsystems phức tạp

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống sau:
- Cần support nhiều loại robot platforms
- Muốn thay đổi thuật toán mà không ảnh hưởng interface
- Cần xử lý nhiều loại sensors/actuators
- Muốn tách biệt high-level logic và low-level implementation
- Cần maintain nhiều versions của cùng một functionality

## 3. Giải pháp

Bridge Pattern giải quyết các vấn đề trên bằng cách:
1. Tách interface (abstraction) khỏi implementation
2. Đặt implementation vào interface riêng
3. Client làm việc với high-level abstraction
4. Cho phép thay đổi implementation runtime

## 4. Ví dụ thực tế: Robot Navigation System

```cpp
// Implementation interface
class INavigationImpl {
public:
    virtual ~INavigationImpl() = default;
    virtual bool initialize() = 0;
    virtual bool computePath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal,
        nav_msgs::msg::Path& path) = 0;
    virtual bool followPath(const nav_msgs::msg::Path& path) = 0;
    virtual void stop() = 0;
    virtual std::string getImplName() const = 0;
};

// Concrete implementation for DWA (Dynamic Window Approach)
class DWANavigation : public INavigationImpl {
public:
    bool initialize() override {
        RCLCPP_INFO(logger_, "Initializing DWA navigation...");
        // Initialize DWA parameters
        return true;
    }

    bool computePath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal,
        nav_msgs::msg::Path& path) override {
        
        RCLCPP_INFO(logger_, "Computing path using DWA...");
        // DWA path planning implementation
        return true;
    }

    bool followPath(const nav_msgs::msg::Path& path) override {
        RCLCPP_INFO(logger_, "Following path with DWA controller...");
        // DWA path following implementation
        return true;
    }

    void stop() override {
        RCLCPP_INFO(logger_, "Stopping DWA navigation...");
        // Stop implementation
    }

    std::string getImplName() const override {
        return "DWA Navigation";
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("DWANavigation");
};

// Concrete implementation for TEB (Timed Elastic Band)
class TEBNavigation : public INavigationImpl {
public:
    bool initialize() override {
        RCLCPP_INFO(logger_, "Initializing TEB navigation...");
        // Initialize TEB parameters
        return true;
    }

    bool computePath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal,
        nav_msgs::msg::Path& path) override {
        
        RCLCPP_INFO(logger_, "Computing path using TEB...");
        // TEB path planning implementation
        return true;
    }

    bool followPath(const nav_msgs::msg::Path& path) override {
        RCLCPP_INFO(logger_, "Following path with TEB controller...");
        // TEB path following implementation
        return true;
    }

    void stop() override {
        RCLCPP_INFO(logger_, "Stopping TEB navigation...");
        // Stop implementation
    }

    std::string getImplName() const override {
        return "TEB Navigation";
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("TEBNavigation");
};

// Abstraction
class RobotNavigator {
public:
    explicit RobotNavigator(std::shared_ptr<INavigationImpl> impl)
        : impl_(impl) {
        RCLCPP_INFO(logger_, "Creating navigator with %s", impl_->getImplName().c_str());
    }

    virtual ~RobotNavigator() = default;

    virtual bool initialize() {
        return impl_->initialize();
    }

    virtual bool navigateToGoal(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Pose& goal_pose) {
        
        RCLCPP_INFO(logger_,
            "Starting navigation from (%.2f, %.2f) to (%.2f, %.2f)",
            current_pose.position.x, current_pose.position.y,
            goal_pose.position.x, goal_pose.position.y);

        nav_msgs::msg::Path path;
        if (!impl_->computePath(current_pose, goal_pose, path)) {
            RCLCPP_ERROR(logger_, "Failed to compute path");
            return false;
        }

        return impl_->followPath(path);
    }

    virtual void stop() {
        RCLCPP_INFO(logger_, "Stopping navigation");
        impl_->stop();
    }

    // Runtime implementation switching
    void setImplementation(std::shared_ptr<INavigationImpl> impl) {
        RCLCPP_INFO(logger_,
            "Switching navigation implementation from %s to %s",
            impl_->getImplName().c_str(),
            impl->getImplName().c_str());
        impl_ = impl;
    }

protected:
    std::shared_ptr<INavigationImpl> impl_;
    rclcpp::Logger logger_ = rclcpp::get_logger("RobotNavigator");
};

// Refined Abstraction for Differential Drive Robots
class DiffDriveNavigator : public RobotNavigator {
public:
    explicit DiffDriveNavigator(std::shared_ptr<INavigationImpl> impl)
        : RobotNavigator(impl) {
        RCLCPP_INFO(logger_, "Initializing differential drive navigator");
    }

    bool navigateToGoal(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Pose& goal_pose) override {
        
        // Additional differential drive specific checks
        if (!checkKinematicConstraints(current_pose, goal_pose)) {
            RCLCPP_ERROR(logger_, "Goal violates differential drive constraints");
            return false;
        }

        return RobotNavigator::navigateToGoal(current_pose, goal_pose);
    }

private:
    bool checkKinematicConstraints(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Pose& goal_pose) {
        // Implement differential drive specific constraint checks
        return true;
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("DiffDriveNavigator");
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Bridge Pattern trong một ROS2 node:

```cpp
class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        // Khởi tạo implementations
        dwa_impl_ = std::make_shared<DWANavigation>();
        teb_impl_ = std::make_shared<TEBNavigation>();

        // Khởi tạo navigator với DWA làm default
        navigator_ = std::make_unique<DiffDriveNavigator>(dwa_impl_);
        navigator_->initialize();

        // Tạo action server cho navigation
        nav_action_server_ = rclcpp_action::create_server<NavigateToGoal>(
            this,
            "navigate_to_goal",
            std::bind(&NavigationNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationNode::handleCancel, this, std::placeholders::_1),
            std::bind(&NavigationNode::handleAccepted, this, std::placeholders::_1));

        // Service để switch navigation implementation
        switch_impl_service_ = create_service<std_srvs::srv::SetString>(
            "switch_navigation_impl",
            std::bind(&NavigationNode::handleSwitchImpl, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Navigation node initialized");
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToGoal::Goal> goal) {
        
        RCLCPP_INFO(get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToGoal>> goal_handle) {
        
        RCLCPP_INFO(get_logger(), "Received cancel request");
        navigator_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToGoal>> goal_handle) {
        
        // Spawn thread to execute navigation
        std::thread{std::bind(&NavigationNode::executeNavigation, this, std::placeholders::_1),
                   goal_handle}.detach();
    }

    void executeNavigation(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToGoal>> goal_handle) {
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<NavigateToGoal::Result>();

        // Get current pose from tf or odometry
        geometry_msgs::msg::Pose current_pose;
        // ... get current pose ...

        if (navigator_->navigateToGoal(current_pose, goal->target_pose)) {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
        } else {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "Navigation failed");
        }
    }

    void handleSwitchImpl(
        const std::shared_ptr<std_srvs::srv::SetString::Request> request,
        std::shared_ptr<std_srvs::srv::SetString::Response> response) {
        
        if (request->data == "dwa") {
            navigator_->setImplementation(dwa_impl_);
            response->success = true;
            response->message = "Switched to DWA navigation";
        } else if (request->data == "teb") {
            navigator_->setImplementation(teb_impl_);
            response->success = true;
            response->message = "Switched to TEB navigation";
        } else {
            response->success = false;
            response->message = "Unknown navigation implementation";
        }
    }

    std::shared_ptr<DWANavigation> dwa_impl_;
    std::shared_ptr<TEBNavigation> teb_impl_;
    std::unique_ptr<DiffDriveNavigator> navigator_;
    
    rclcpp_action::Server<NavigateToGoal>::SharedPtr nav_action_server_;
    rclcpp::Service<std_srvs::srv::SetString>::SharedPtr switch_impl_service_;
};
```

## 6. Lợi ích

1. **Tách biệt interface và implementation**:
   - Thay đổi implementation không ảnh hưởng client code
   - Dễ dàng thêm implementations mới
   - Giảm coupling giữa các components

2. **Linh hoạt trong runtime**:
   - Switch implementation khi cần
   - Test nhiều implementations khác nhau
   - Adapt theo điều kiện môi trường

3. **Mở rộng dễ dàng**:
   - Thêm abstractions mới độc lập
   - Thêm implementations mới độc lập
   - Tái sử dụng code tốt hơn

4. **Quản lý phức tạp**:
   - Tách biệt concerns
   - Code rõ ràng và có tổ chức
   - Dễ maintain và test

## 7. Khi nào sử dụng

- Cần tách biệt interface và implementation
- Muốn thay đổi implementation runtime
- Có nhiều variants của cùng một functionality
- Cần support nhiều platforms hoặc hardware
- Muốn tránh tight coupling

## 8. Lưu ý

1. Thiết kế Bridge:
   - Xác định rõ abstraction và implementation
   - Giữ interface đơn giản và rõ ràng
   - Cân nhắc lifecycle management

2. Hiệu suất:
   - Xem xét overhead của virtual calls
   - Cẩn thận với resource management
   - Tối ưu critical paths

3. Trong ROS2:
   - Xử lý thread safety
   - Quản lý node lifecycle
   - Implement proper error handling
   - Cân nhắc real-time constraints 