# SINGLETON PATTERN 

## 1. Giới thiệu

Singleton Pattern là một mẫu thiết kế thuộc nhóm Creational Pattern, đảm bảo một class chỉ có một instance duy nhất và cung cấp một điểm truy cập toàn cục đến instance đó. Trong ROS2 và robotics, pattern này thường được sử dụng cho:
- Hardware resource management
- Configuration management
- Logging systems
- Robot state management
- Shared resource coordination

**Lưu ý quan trọng**: Mặc dù Singleton là một pattern phổ biến, nó thường được coi là anti-pattern vì có thể gây ra các vấn đề về:
- Global state
- Tight coupling
- Testing difficulty
- Thread safety concerns

Trong ROS2, thay vì sử dụng Singleton, chúng ta nên ưu tiên sử dụng:
- Node parameters
- Services
- Actions
- Topics
- Component lifecycle management

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Quản lý truy cập vào hardware resources
- Chia sẻ configuration giữa nhiều components
- Đồng bộ hóa trạng thái robot
- Logging tập trung
- Quản lý shared memory

## 3. Giải pháp

Singleton Pattern giải quyết các vấn đề trên bằng cách:
1. Private constructor để ngăn tạo instance trực tiếp
2. Static method để truy cập instance duy nhất
3. Lazy initialization để tối ưu resources
4. Thread-safe implementation

## 4. Ví dụ thực tế: Robot Hardware Manager

```cpp
class RobotHardwareManager {
public:
    // Delete copy constructor và assignment operator
    RobotHardwareManager(const RobotHardwareManager&) = delete;
    RobotHardwareManager& operator=(const RobotHardwareManager&) = delete;

    // Phương thức static để lấy instance
    static RobotHardwareManager& getInstance() {
        static RobotHardwareManager instance;  // Thread-safe từ C++11
        return instance;
    }

    // Hardware control methods
    bool initializeHardware() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_initialized_) {
            RCLCPP_WARN(logger_, "Hardware already initialized");
            return true;
        }

        RCLCPP_INFO(logger_, "Initializing robot hardware...");
        // Initialize hardware components
        is_initialized_ = true;
        return true;
    }

    bool setMotorPower(const std::string& motor_id, double power) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_initialized_) {
            RCLCPP_ERROR(logger_, "Hardware not initialized");
            return false;
        }

        if (motor_states_.find(motor_id) == motor_states_.end()) {
            RCLCPP_ERROR(logger_, "Motor %s not found", motor_id.c_str());
            return false;
        }

        RCLCPP_DEBUG(logger_, "Setting motor %s power to %.2f",
                     motor_id.c_str(), power);
        motor_states_[motor_id] = power;
        return true;
    }

    double getMotorPower(const std::string& motor_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_initialized_ || 
            motor_states_.find(motor_id) == motor_states_.end()) {
            return 0.0;
        }
        return motor_states_[motor_id];
    }

    void emergencyStop() {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_WARN(logger_, "Emergency stop triggered!");
        for (auto& motor : motor_states_) {
            motor.second = 0.0;
        }
        is_emergency_ = true;
    }

    bool resetEmergencyStop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_emergency_) {
            return true;
        }
        RCLCPP_INFO(logger_, "Resetting emergency stop...");
        is_emergency_ = false;
        return true;
    }

    // Diagnostic methods
    std::string getHardwareStatus() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::stringstream ss;
        ss << "Hardware Status:\n"
           << "Initialized: " << (is_initialized_ ? "Yes" : "No") << "\n"
           << "Emergency Stop: " << (is_emergency_ ? "Active" : "Inactive") << "\n"
           << "Motor States:\n";
        
        for (const auto& motor : motor_states_) {
            ss << "  " << motor.first << ": " << motor.second << "\n";
        }
        return ss.str();
    }

private:
    // Private constructor
    RobotHardwareManager() 
        : logger_(rclcpp::get_logger("RobotHardwareManager"))
        , is_initialized_(false)
        , is_emergency_(false) {
        // Initialize motor states
        motor_states_["left_wheel"] = 0.0;
        motor_states_["right_wheel"] = 0.0;
        motor_states_["arm_joint1"] = 0.0;
        motor_states_["arm_joint2"] = 0.0;
        RCLCPP_INFO(logger_, "RobotHardwareManager created");
    }

    // Member variables
    rclcpp::Logger logger_;
    std::mutex mutex_;
    bool is_initialized_;
    bool is_emergency_;
    std::map<std::string, double> motor_states_;
};
```

## 5. Sử dụng trong ROS2

Mặc dù không khuyến khích sử dụng Singleton trong ROS2, đây là ví dụ về cách tích hợp nếu thực sự cần thiết:

```cpp
class HardwareNode : public rclcpp::Node {
public:
    HardwareNode() : Node("hardware_node") {
        // Khởi tạo hardware
        auto& hw_manager = RobotHardwareManager::getInstance();
        hw_manager.initializeHardware();

        // Create services
        power_service_ = create_service<custom_msgs::srv::SetMotorPower>(
            "set_motor_power",
            std::bind(&HardwareNode::handleSetPower, this,
                     std::placeholders::_1, std::placeholders::_2));

        emergency_service_ = create_service<std_srvs::srv::Trigger>(
            "emergency_stop",
            std::bind(&HardwareNode::handleEmergencyStop, this,
                     std::placeholders::_1, std::placeholders::_2));

        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "reset_emergency",
            std::bind(&HardwareNode::handleResetEmergency, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Create status publisher
        status_publisher_ = create_publisher<std_msgs::msg::String>(
            "hardware_status", 10);

        // Create timer for status updates
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareNode::publishStatus, this));

        RCLCPP_INFO(get_logger(), "Hardware node initialized");
    }

private:
    void handleSetPower(
        const custom_msgs::srv::SetMotorPower::Request::SharedPtr request,
        custom_msgs::srv::SetMotorPower::Response::SharedPtr response) {
        
        auto& hw_manager = RobotHardwareManager::getInstance();
        response->success = hw_manager.setMotorPower(
            request->motor_id, request->power);
        
        if (!response->success) {
            response->message = "Failed to set motor power";
        }
    }

    void handleEmergencyStop(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        
        auto& hw_manager = RobotHardwareManager::getInstance();
        hw_manager.emergencyStop();
        
        response->success = true;
        response->message = "Emergency stop activated";
    }

    void handleResetEmergency(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        
        auto& hw_manager = RobotHardwareManager::getInstance();
        response->success = hw_manager.resetEmergencyStop();
        
        if (response->success) {
            response->message = "Emergency stop reset";
        } else {
            response->message = "Failed to reset emergency stop";
        }
    }

    void publishStatus() {
        auto& hw_manager = RobotHardwareManager::getInstance();
        auto msg = std_msgs::msg::String();
        msg.data = hw_manager.getHardwareStatus();
        status_publisher_->publish(msg);
    }

    rclcpp::Service<custom_msgs::srv::SetMotorPower>::SharedPtr power_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};
```

## 6. Lợi ích và Nhược điểm

### Lợi ích:
1. **Truy cập tập trung**:
   - Single point of access
   - Consistent state
   - Resource sharing

2. **Resource management**:
   - Controlled initialization
   - Efficient resource use
   - Coordinated access

3. **Đơn giản hóa**:
   - Không cần pass references
   - Global access
   - Easy to implement

### Nhược điểm:
1. **Global state**:
   - Hidden dependencies
   - Hard to track changes
   - Testing difficulties

2. **Tight coupling**:
   - Hard to modify
   - Hard to extend
   - Violates Single Responsibility

3. **Concurrency issues**:
   - Thread safety complexity
   - Performance bottlenecks
   - Race conditions

## 7. Khi nào sử dụng

Trong ROS2, hạn chế sử dụng Singleton và chỉ cân nhắc khi:
- Thực sự cần một instance duy nhất
- Không thể sử dụng ROS2 mechanisms
- Cần quản lý hardware resources
- Cần shared state có kiểm soát
- Legacy code integration

## 8. Lưu ý

1. Thiết kế:
   - Ưu tiên ROS2 mechanisms
   - Giữ interface đơn giản
   - Implement thread safety
   - Xử lý initialization đúng

2. Testing:
   - Mock singleton cho unit tests
   - Dependency injection khi có thể
   - Test thread safety
   - Test error cases

3. Trong ROS2:
   - Tránh global state
   - Sử dụng node parameters
   - Implement proper lifecycle
   - Follow ROS2 patterns 