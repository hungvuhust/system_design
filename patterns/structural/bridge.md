# Bridge Pattern in ROS2

## What is the Bridge Pattern?

The Bridge pattern is a structural design pattern that lets you split a large class or a set of closely related classes into two separate hierarchies—abstraction and implementation—which can be developed independently of each other.

## Use Case in Robotics

In robotics, the Bridge pattern is useful when you need to decouple an abstraction from its implementation. For example, you might have a high-level motion planning interface (the abstraction) that can work with different underlying robot platforms (the implementations). The motion planning algorithms don't need to know the specifics of how a robot moves; they just command the robot through a generic interface. The implementation part translates these generic commands into platform-specific instructions (e.g., for a differential drive robot, a holonomic robot, or a legged robot).

This allows you to add new robot platforms or new motion planning algorithms without modifying the existing code on the other side of the bridge.

## C++ Example

Here is a C++ example of the Bridge pattern for controlling different types of robot platforms.

```cpp
// Implementor interface
class RobotPlatformImpl {
public:
    virtual ~RobotPlatformImpl() {}
    virtual void move(double x, double y) = 0;
    virtual void turn(double angle) = 0;
    virtual void stop() = 0;
};

// Concrete Implementor A
class DifferentialDrivePlatform : public RobotPlatformImpl {
public:
    void move(double x, double y) override {
        // Send commands to a differential drive robot
    }

    void turn(double angle) override {
        // Send commands to a differential drive robot
    }

    void stop() override {
        // Send commands to a differential drive robot
    }
};

// Concrete Implementor B
class LeggedPlatform : public RobotPlatformImpl {
public:
    void move(double x, double y) override {
        // Send commands to a legged robot
    }

    void turn(double angle) override {
        // Send commands to a legged robot
    }

    void stop() override {
        // Send commands to a legged robot
    }
};

// Abstraction
class RobotController {
protected:
    RobotPlatformImpl* platform;

public:
    RobotController(RobotPlatformImpl* impl) : platform(impl) {}
    virtual ~RobotController() {}

    virtual void executeMotion() = 0;
};

// Refined Abstraction
class AutonomousNavigator : public RobotController {
public:
    AutonomousNavigator(RobotPlatformImpl* impl) : RobotController(impl) {}

    void executeMotion() override {
        // High-level navigation logic
        platform->move(1.0, 0.0);
        platform->turn(90.0);
        platform->move(0.0, 1.0);
        platform->stop();
    }
};
```

## Best Practices

*   **Decoupling:** Use the Bridge pattern when you want to decouple an abstraction from its implementation so that they can vary independently.
*   **Single Responsibility Principle:** The abstraction should focus on high-level logic, and the implementation should focus on platform-specific details.
*   **Composition over Inheritance:** The Bridge pattern uses composition to link the abstraction and implementation, which is more flexible than using inheritance.

## Extensions and Variations

*   **Multiple Bridges:** You can have multiple bridges in your system for different abstractions (e.g., one for motion, one for manipulation).
*   **Runtime Implementation Selection:** The implementation can be chosen or switched at runtime.

## Testing

*   **Unit Testing:** Test the abstraction and implementation hierarchies separately.
*   **Mocking:** Use mock objects for the implementations to test the abstractions in isolation.

## BRIDGE PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Tưởng tượng bạn đang xây dựng một hệ thống điều khiển robot với nhiều loại robot khác nhau (mobile robot, robot arm, drone) và nhiều loại điều khiển khác nhau (position control, velocity control, force control). Thay vì phải viết code riêng cho mỗi kết hợp robot-controller, Bridge Pattern cho phép bạn:

- Tách riêng robot interface và controller interface
- Kết hợp linh hoạt bất kỳ robot nào với bất kỳ controller nào
- Thêm robot mới hoặc controller mới mà không ảnh hưởng đến code hiện có

#### 2. Định nghĩa chi tiết
Bridge Pattern là một mẫu thiết kế cấu trúc cho phép bạn tách một class lớn hoặc một tập các class có liên quan thành hai hệ thống phân cấp riêng biệt - abstraction và implementation - có thể phát triển độc lập với nhau.

#### Các thành phần chính:
1. **Abstraction**:
   - Interface cao cấp cho client sử dụng
   - Trong ROS2: Robot interface

2. **Implementation**:
   - Interface cho các concrete implementations
   - Trong ROS2: Controller interface

3. **Refined Abstraction**:
   - Mở rộng của abstraction
   - Trong ROS2: Specific robot types

4. **Concrete Implementation**:
   - Implementation cụ thể
   - Trong ROS2: Specific controllers

#### 3. Ví dụ thực tế trong ROS2
Giả sử chúng ta đang xây dựng một hệ thống điều khiển robot đa năng:

```cpp
// 1. Implementation Interface (Bridge)
class RobotController {
public:
    virtual ~RobotController() = default;
    
    // Các phương thức điều khiển cơ bản
    virtual void initialize() = 0;
    virtual bool setTarget(const geometry_msgs::msg::Pose& target) = 0;
    virtual bool executeMotion() = 0;
    virtual void stop() = 0;
    virtual std::string getControllerType() const = 0;
    
    // Getters cho trạng thái
    virtual geometry_msgs::msg::Pose getCurrentPose() const = 0;
    virtual bool isMoving() const = 0;
    virtual double getControlFrequency() const = 0;
};

// 2. Concrete Implementations

// 2.1 Position Controller
class PositionController : public RobotController {
public:
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Position Controller");
        // Khởi tạo PID controllers
        setupPIDControllers();
    }
    
    bool setTarget(const geometry_msgs::msg::Pose& target) override {
        target_pose_ = target;
        return validateTarget(target);
    }
    
    bool executeMotion() override {
        if (!isTargetValid()) {
            RCLCPP_ERROR(logger_, "Invalid target pose");
            return false;
        }
        
        // Thực hiện position control
        return runPositionControl();
    }
    
    void stop() override {
        RCLCPP_INFO(logger_, "Stopping position controller");
        stopMotion();
    }
    
    std::string getControllerType() const override {
        return "POSITION_CONTROLLER";
    }
    
    geometry_msgs::msg::Pose getCurrentPose() const override {
        return current_pose_;
    }
    
    bool isMoving() const override {
        return is_moving_;
    }
    
    double getControlFrequency() const override {
        return 100.0;  // 100Hz
    }

private:
    void setupPIDControllers() {
        // Setup PID gains
    }
    
    bool validateTarget(const geometry_msgs::msg::Pose& target) {
        // Kiểm tra target có hợp lệ
        return true;
    }
    
    bool runPositionControl() {
        // Implement position control loop
        return true;
    }
    
    void stopMotion() {
        is_moving_ = false;
        // Dừng động cơ
    }
    
    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose current_pose_;
    bool is_moving_ = false;
    rclcpp::Logger logger_{rclcpp::get_logger("PositionController")};
};

// 2.2 Velocity Controller
class VelocityController : public RobotController {
public:
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Velocity Controller");
        // Khởi tạo velocity control parameters
    }
    
    bool setTarget(const geometry_msgs::msg::Pose& target) override {
        // Chuyển đổi target pose thành velocity commands
        return calculateVelocityProfile(target);
    }
    
    bool executeMotion() override {
        // Thực hiện velocity control
        return runVelocityControl();
    }
    
    void stop() override {
        RCLCPP_INFO(logger_, "Stopping velocity controller");
        setZeroVelocity();
    }
    
    std::string getControllerType() const override {
        return "VELOCITY_CONTROLLER";
    }
    
    geometry_msgs::msg::Pose getCurrentPose() const override {
        return current_pose_;
    }
    
    bool isMoving() const override {
        return current_velocity_ > 0.001;
    }
    
    double getControlFrequency() const override {
        return 200.0;  // 200Hz
    }

private:
    bool calculateVelocityProfile(const geometry_msgs::msg::Pose& target) {
        // Tính toán velocity profile
        return true;
    }
    
    bool runVelocityControl() {
        // Implement velocity control loop
        return true;
    }
    
    void setZeroVelocity() {
        current_velocity_ = 0.0;
        // Dừng động cơ
    }
    
    geometry_msgs::msg::Pose current_pose_;
    double current_velocity_ = 0.0;
    rclcpp::Logger logger_{rclcpp::get_logger("VelocityController")};
};

// 3. Abstraction
class Robot {
public:
    explicit Robot(std::shared_ptr<RobotController> controller)
        : controller_(controller) {
        if (!controller_) {
            throw std::runtime_error("Null controller provided");
        }
    }
    
    virtual ~Robot() = default;
    
    // Common interface for all robots
    virtual void initialize() = 0;
    virtual bool moveToTarget(const geometry_msgs::msg::Pose& target) = 0;
    virtual void emergencyStop() = 0;
    virtual std::string getRobotType() const = 0;
    
    // Getters
    geometry_msgs::msg::Pose getCurrentPose() const {
        return controller_->getCurrentPose();
    }
    
    bool isMoving() const {
        return controller_->isMoving();
    }
    
    std::string getControllerType() const {
        return controller_->getControllerType();
    }

protected:
    std::shared_ptr<RobotController> controller_;
};

// 4. Refined Abstractions

// 4.1 Mobile Robot
class MobileRobot : public Robot {
public:
    explicit MobileRobot(std::shared_ptr<RobotController> controller)
        : Robot(controller) {}
    
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Mobile Robot");
        controller_->initialize();
        setupSafetyLimits();
    }
    
    bool moveToTarget(const geometry_msgs::msg::Pose& target) override {
        if (!isSafeTarget(target)) {
            RCLCPP_ERROR(logger_, "Target pose outside safety limits");
            return false;
        }
        
        if (!controller_->setTarget(target)) {
            RCLCPP_ERROR(logger_, "Failed to set target");
            return false;
        }
        
        return controller_->executeMotion();
    }
    
    void emergencyStop() override {
        RCLCPP_WARN(logger_, "Emergency stop triggered for mobile robot");
        controller_->stop();
    }
    
    std::string getRobotType() const override {
        return "MOBILE_ROBOT";
    }

private:
    void setupSafetyLimits() {
        // Setup safety boundaries
    }
    
    bool isSafeTarget(const geometry_msgs::msg::Pose& target) {
        // Check if target is within safety limits
        return true;
    }
    
    rclcpp::Logger logger_{rclcpp::get_logger("MobileRobot")};
};

// 4.2 Robot Arm
class RobotArm : public Robot {
public:
    explicit RobotArm(std::shared_ptr<RobotController> controller)
        : Robot(controller) {}
    
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Robot Arm");
        controller_->initialize();
        loadKinematicsModel();
    }
    
    bool moveToTarget(const geometry_msgs::msg::Pose& target) override {
        if (!isReachable(target)) {
            RCLCPP_ERROR(logger_, "Target pose not reachable");
            return false;
        }
        
        if (!controller_->setTarget(target)) {
            RCLCPP_ERROR(logger_, "Failed to set target");
            return false;
        }
        
        return controller_->executeMotion();
    }
    
    void emergencyStop() override {
        RCLCPP_WARN(logger_, "Emergency stop triggered for robot arm");
        controller_->stop();
        engageBrakes();
    }
    
    std::string getRobotType() const override {
        return "ROBOT_ARM";
    }

private:
    void loadKinematicsModel() {
        // Load robot arm kinematics
    }
    
    bool isReachable(const geometry_msgs::msg::Pose& target) {
        // Check if target is within workspace
        return true;
    }
    
    void engageBrakes() {
        // Engage mechanical brakes
    }
    
    rclcpp::Logger logger_{rclcpp::get_logger("RobotArm")};
};

// 5. Client code trong ROS2 node
class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode() : Node("robot_control") {
        // Khởi tạo robot và controller
        setupRobotSystem();
        
        // Tạo subscribers và services
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&RobotControlNode::targetCallback, this, std::placeholders::_1));
            
        stop_service_ = create_service<std_srvs::srv::Trigger>(
            "emergency_stop",
            std::bind(&RobotControlNode::stopCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    void setupRobotSystem() {
        try {
            // Đọc parameters
            std::string robot_type = get_parameter("robot_type").as_string();
            std::string controller_type = get_parameter("controller_type").as_string();
            
            // Tạo controller phù hợp
            std::shared_ptr<RobotController> controller;
            if (controller_type == "position") {
                controller = std::make_shared<PositionController>();
            } else if (controller_type == "velocity") {
                controller = std::make_shared<VelocityController>();
            } else {
                throw std::runtime_error("Unknown controller type");
            }
            
            // Tạo robot phù hợp
            if (robot_type == "mobile") {
                robot_ = std::make_unique<MobileRobot>(controller);
            } else if (robot_type == "arm") {
                robot_ = std::make_unique<RobotArm>(controller);
            } else {
                throw std::runtime_error("Unknown robot type");
            }
            
            // Khởi tạo hệ thống
            robot_->initialize();
            RCLCPP_INFO(get_logger(), 
                "Initialized %s with %s",
                robot_->getRobotType().c_str(),
                robot_->getControllerType().c_str());
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to setup robot: %s", e.what());
            throw;
        }
    }
    
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        try {
            if (robot_->isMoving()) {
                RCLCPP_WARN(get_logger(), "Robot is already moving, ignoring target");
                return;
            }
            
            if (robot_->moveToTarget(msg->pose)) {
                RCLCPP_INFO(get_logger(), "Robot moving to target");
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to move to target");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in target callback: %s", e.what());
        }
    }
    
    void stopCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        try {
            robot_->emergencyStop();
            response->success = true;
            response->message = "Emergency stop executed";
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Failed to stop: ") + e.what();
        }
    }
    
    std::unique_ptr<Robot> robot_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};
```

#### 4. Giải thích chi tiết cách hoạt động
1. **Phân tách interface**:
   - Robot interface (Abstraction)
   - Controller interface (Implementation)
   - Hai hệ thống phát triển độc lập

2. **Kết hợp linh hoạt**:
   - Bất kỳ robot nào với bất kỳ controller nào
   - Runtime configuration
   - Dễ dàng mở rộng

3. **Xử lý lỗi**:
   - Kiểm tra null pointers
   - Validation input
   - Exception handling

#### 5. Ưu điểm trong ROS2
1. **Tính mở rộng**:
   - Thêm robot mới dễ dàng
   - Thêm controller mới độc lập
   - Không ảnh hưởng code hiện có

2. **Tái sử dụng code**:
   - Controllers dùng chung cho nhiều robots
   - Common functionality trong base classes
   - Reduced code duplication

3. **Dễ bảo trì**:
   - Các thành phần độc lập
   - Dễ test riêng từng phần
   - Clear responsibilities

#### 6. Các trường hợp sử dụng trong ROS2
1. **Robot Control**:
   - Different robot types
   - Multiple control strategies
   - Hardware abstraction

2. **Sensor Systems**:
   - Different sensor types
   - Processing algorithms
   - Data fusion

3. **Navigation**:
   - Path planning algorithms
   - Local planners
   - Global planners

#### 7. Best Practices trong ROS2
1. **Error Handling**:
```cpp
try {
    if (!robot->initialize()) {
        throw std::runtime_error("Robot initialization failed");
    }
} catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error: %s", e.what());
    // Fallback behavior
}
```

2. **Resource Management**:
```cpp
class SafeRobot {
    std::unique_ptr<Robot> robot_;
public:
    ~SafeRobot() {
        if (robot_) {
            robot_->emergencyStop();
        }
    }
};
```

3. **Parameter Management**:
```cpp
void loadParameters() {
    auto params = node_->get_parameters({
        "robot_type",
        "controller_type",
        "safety_limits"
    });
    // Process parameters
}
```

#### 8. Mở rộng và tùy chỉnh
1. **New Robot Type**:
```cpp
class DroneRobot : public Robot {
public:
    void initialize() override {
        // Drone specific initialization
    }
    
    bool moveToTarget(const geometry_msgs::msg::Pose& target) override {
        // Drone specific movement
    }
};
```

2. **New Controller Type**:
```cpp
class ForceController : public RobotController {
public:
    bool executeMotion() override {
        // Force control implementation
    }
};
```

3. **Combined Controllers**:
```cpp
class HybridController : public RobotController {
    std::vector<std::shared_ptr<RobotController>> controllers_;
public:
    bool executeMotion() override {
        // Choose best controller based on situation
    }
};
```

#### 9. Testing
1. **Mock Objects**:
```cpp
class MockController : public RobotController {
public:
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(bool, executeMotion, (), (override));
};
```

2. **Unit Tests**:
```cpp
TEST(RobotTest, InitializationTest) {
    auto mock_controller = std::make_shared<MockController>();
    EXPECT_CALL(*mock_controller, initialize())
        .WillOnce(Return(true));
        
    MobileRobot robot(mock_controller);
    EXPECT_NO_THROW(robot.initialize());
}
```

3. **Integration Tests**:
```cpp
TEST(RobotSystemTest, FullSystemTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    RobotControlNode control_node;
    
    // Test robot movement
    geometry_msgs::msg::Pose target;
    EXPECT_TRUE(control_node.moveToTarget(target));
    
    // Test emergency stop
    EXPECT_NO_THROW(control_node.emergencyStop());
}
```

#### 10. Kết luận
Bridge Pattern là một mẫu thiết kế cấu trúc mạnh mẽ trong phát triển phần mềm robotics với ROS2, đặc biệt trong việc tách biệt abstraction và implementation. Pattern này mang lại nhiều lợi thế quan trọng:

1. **Thiết kế linh hoạt**:
   - Tách biệt rõ ràng giữa interface và implementation
   - Cho phép thay đổi độc lập giữa robot types và controllers
   - Dễ dàng mở rộng cả về chiều rộng (thêm robots) và chiều sâu (thêm controllers)

2. **Tối ưu cho robotics**:
   - Phù hợp với các hệ thống điều khiển robot phức tạp
   - Hỗ trợ đa dạng các loại robot và chiến lược điều khiển
   - Dễ dàng chuyển đổi giữa các modes điều khiển

3. **Quản lý code hiệu quả**:
   - Cấu trúc code rõ ràng và có tổ chức
   - Giảm thiểu code trùng lặp
   - Dễ dàng maintain và update từng phần độc lập

4. **Giá trị thực tiễn**:
   - Tăng tính tái sử dụng của code
   - Giảm thời gian phát triển cho các robot mới
   - Tăng độ tin cậy của hệ thống

Trong ví dụ về hệ thống điều khiển robot, chúng ta đã thấy Bridge Pattern giúp xây dựng một kiến trúc linh hoạt, cho phép kết hợp bất kỳ loại robot nào với bất kỳ loại controller nào. Pattern này là lựa chọn xuất sắc cho các dự án robotics cần sự linh hoạt cao và khả năng mở rộng dễ dàng.