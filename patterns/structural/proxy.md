# Proxy Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Proxy Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, cung cấp một đối tượng đại diện (proxy) để kiểm soát truy cập đến một đối tượng khác. Pattern này đặc biệt hữu ích trong robotics khi:
- Kiểm soát truy cập vào tài nguyên quan trọng
- Lazy loading cho các tài nguyên nặng
- Logging và monitoring các thao tác
- Caching dữ liệu từ sensors
- Điều khiển robot từ xa

## 2. Vấn đề

Trong hệ thống robotics, chúng ta thường gặp các tình huống sau:
- Cần kiểm soát truy cập vào hardware của robot
- Muốn cache dữ liệu sensor để tránh đọc quá nhiều
- Cần logging mọi thao tác điều khiển robot
- Muốn lazy load các model AI nặng
- Cần xử lý remote control an toàn

## 3. Giải pháp

Proxy Pattern giải quyết các vấn đề trên bằng cách:
1. Tạo interface chung cho proxy và real object
2. Proxy implement interface này và wrap real object
3. Client tương tác với proxy thay vì real object
4. Proxy thêm các chức năng như caching, logging, access control

## 4. Ví dụ thực tế: Robot Arm Control

```cpp
// Common interface for robot arm control
class IRobotArm {
public:
    virtual ~IRobotArm() = default;
    virtual bool initialize() = 0;
    virtual bool moveToPosition(double x, double y, double z) = 0;
    virtual bool grab() = 0;
    virtual bool release() = 0;
    virtual std::vector<double> getCurrentPosition() = 0;
};

// Real robot arm implementation
class RealRobotArm : public IRobotArm {
public:
    bool initialize() override {
        RCLCPP_INFO(logger_, "Initializing real robot arm...");
        // Real hardware initialization
        return true;
    }

    bool moveToPosition(double x, double y, double z) override {
        RCLCPP_INFO(logger_, "Moving arm to position: (%.2f, %.2f, %.2f)", x, y, z);
        // Real hardware movement
        current_position_ = {x, y, z};
        return true;
    }

    bool grab() override {
        RCLCPP_INFO(logger_, "Grabbing object");
        // Real gripper control
        return true;
    }

    bool release() override {
        RCLCPP_INFO(logger_, "Releasing object");
        // Real gripper control
        return true;
    }

    std::vector<double> getCurrentPosition() override {
        return current_position_;
    }

private:
    std::vector<double> current_position_{0.0, 0.0, 0.0};
    rclcpp::Logger logger_ = rclcpp::get_logger("RealRobotArm");
};

// Proxy for robot arm with safety checks and logging
class RobotArmProxy : public IRobotArm {
public:
    explicit RobotArmProxy(std::shared_ptr<IRobotArm> real_arm)
        : real_arm_(real_arm) {
        // Initialize workspace limits
        workspace_limits_ = {
            {-0.5, 0.5},  // x limits
            {-0.5, 0.5},  // y limits
            {0.0, 0.8}    // z limits
        };
    }

    bool initialize() override {
        if (initialized_) {
            RCLCPP_WARN(logger_, "Arm already initialized");
            return true;
        }

        RCLCPP_INFO(logger_, "Proxy: Initializing arm with safety checks");
        initialized_ = real_arm_->initialize();
        return initialized_;
    }

    bool moveToPosition(double x, double y, double z) override {
        if (!initialized_) {
            RCLCPP_ERROR(logger_, "Arm not initialized");
            return false;
        }

        // Safety check
        if (!isPositionSafe(x, y, z)) {
            RCLCPP_ERROR(logger_, 
                "Position (%.2f, %.2f, %.2f) outside safe workspace",
                x, y, z);
            return false;
        }

        // Rate limiting
        auto now = std::chrono::steady_clock::now();
        if (now - last_move_ < std::chrono::milliseconds(100)) {
            RCLCPP_WARN(logger_, "Moving too fast, throttling");
            return false;
        }
        last_move_ = now;

        // Log movement
        RCLCPP_INFO(logger_, 
            "Proxy: Moving arm from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
            last_position_[0], last_position_[1], last_position_[2],
            x, y, z);

        // Cache position
        if (real_arm_->moveToPosition(x, y, z)) {
            last_position_ = {x, y, z};
            return true;
        }
        return false;
    }

    bool grab() override {
        if (!initialized_) {
            RCLCPP_ERROR(logger_, "Arm not initialized");
            return false;
        }

        RCLCPP_INFO(logger_, "Proxy: Executing grab operation");
        return real_arm_->grab();
    }

    bool release() override {
        if (!initialized_) {
            RCLCPP_ERROR(logger_, "Arm not initialized");
            return false;
        }

        RCLCPP_INFO(logger_, "Proxy: Executing release operation");
        return real_arm_->release();
    }

    std::vector<double> getCurrentPosition() override {
        // Return cached position to avoid frequent hardware reads
        return last_position_;
    }

private:
    bool isPositionSafe(double x, double y, double z) {
        return x >= workspace_limits_[0][0] && x <= workspace_limits_[0][1] &&
               y >= workspace_limits_[1][0] && y <= workspace_limits_[1][1] &&
               z >= workspace_limits_[2][0] && z <= workspace_limits_[2][1];
    }

    std::shared_ptr<IRobotArm> real_arm_;
    bool initialized_ = false;
    std::vector<double> last_position_{0.0, 0.0, 0.0};
    std::chrono::steady_clock::time_point last_move_ = std::chrono::steady_clock::now();
    std::vector<std::array<double, 2>> workspace_limits_;
    rclcpp::Logger logger_ = rclcpp::get_logger("RobotArmProxy");
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Proxy Pattern trong một ROS2 node:

```cpp
class RobotArmNode : public rclcpp::Node {
public:
    RobotArmNode() : Node("robot_arm_node") {
        // Khởi tạo real arm và proxy
        real_arm_ = std::make_shared<RealRobotArm>();
        arm_proxy_ = std::make_shared<RobotArmProxy>(real_arm_);

        // Initialize arm
        arm_proxy_->initialize();

        // Create services
        move_service_ = create_service<custom_msgs::srv::MoveArm>(
            "move_arm",
            std::bind(&RobotArmNode::handleMoveRequest, this,
                     std::placeholders::_1, std::placeholders::_2));

        grab_service_ = create_service<std_srvs::srv::Trigger>(
            "grab",
            std::bind(&RobotArmNode::handleGrabRequest, this,
                     std::placeholders::_1, std::placeholders::_2));

        release_service_ = create_service<std_srvs::srv::Trigger>(
            "release",
            std::bind(&RobotArmNode::handleReleaseRequest, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Create position publisher
        position_publisher_ = create_publisher<geometry_msgs::msg::Point>(
            "arm_position", 10);

        // Create timer for position updates
        position_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotArmNode::publishPosition, this));
    }

private:
    void handleMoveRequest(
        const custom_msgs::srv::MoveArm::Request::SharedPtr request,
        custom_msgs::srv::MoveArm::Response::SharedPtr response) {
        
        response->success = arm_proxy_->moveToPosition(
            request->x, request->y, request->z);
    }

    void handleGrabRequest(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        
        response->success = arm_proxy_->grab();
        response->message = "Grab operation completed";
    }

    void handleReleaseRequest(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        
        response->success = arm_proxy_->release();
        response->message = "Release operation completed";
    }

    void publishPosition() {
        auto position = arm_proxy_->getCurrentPosition();
        auto msg = geometry_msgs::msg::Point();
        msg.x = position[0];
        msg.y = position[1];
        msg.z = position[2];
        position_publisher_->publish(msg);
    }

    std::shared_ptr<RealRobotArm> real_arm_;
    std::shared_ptr<RobotArmProxy> arm_proxy_;
    
    rclcpp::Service<custom_msgs::srv::MoveArm>::SharedPtr move_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr grab_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;
    rclcpp::TimerBase::SharedPtr position_timer_;
};
```

## 6. Lợi ích

1. **Kiểm soát truy cập**:
   - Bảo vệ tài nguyên quan trọng
   - Thêm các kiểm tra an toàn
   - Giới hạn quyền truy cập

2. **Tối ưu hiệu suất**:
   - Caching dữ liệu
   - Lazy loading
   - Rate limiting

3. **Monitoring và Logging**:
   - Theo dõi mọi thao tác
   - Debug dễ dàng hơn
   - Audit trail

4. **Tách biệt logic**:
   - Code sạch và module hóa
   - Dễ bảo trì và mở rộng
   - Single Responsibility Principle

## 7. Khi nào sử dụng

- Cần kiểm soát truy cập vào tài nguyên quan trọng
- Muốn thêm logging hoặc monitoring
- Cần caching để tối ưu hiệu suất
- Xử lý lazy loading cho tài nguyên nặng
- Implement remote control an toàn

## 8. Lưu ý

1. Thiết kế Proxy:
   - Giữ interface đơn giản
   - Xử lý lỗi cẩn thận
   - Tránh thêm quá nhiều logic

2. Hiệu suất:
   - Cân nhắc overhead của proxy
   - Tối ưu caching strategy
   - Xử lý concurrent access

3. Trong ROS2:
   - Đảm bảo thread safety
   - Xử lý timeout hợp lý
   - Theo dõi latency
   - Implement proper cleanup