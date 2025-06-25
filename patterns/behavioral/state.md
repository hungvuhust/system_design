# State Pattern trong ROS2 và Robotics

## 1. Giới thiệu

State Pattern là một behavioral pattern cho phép một object thay đổi hành vi của nó khi trạng thái nội bộ thay đổi. Trong ROS2 và robotics, pattern này đặc biệt hữu ích cho:

- Robot behavior management
- Task execution flow
- Error handling states
- Sensor processing modes
- Navigation states
- Hardware control modes

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống:
- Robot cần chuyển đổi giữa nhiều modes hoạt động
- Xử lý các trạng thái lỗi khác nhau
- Quản lý các giai đoạn trong một nhiệm vụ phức tạp
- Điều chỉnh behavior dựa trên sensor data
- Chuyển đổi giữa các control modes

## 3. Giải pháp

State Pattern giải quyết các vấn đề trên bằng cách:
1. Định nghĩa các state riêng biệt
2. Encapsulate state-specific behavior
3. Manage state transitions
4. Provide clean interface for state changes

## 4. Ví dụ thực tế: Robot Task Execution System

```cpp
// Abstract state interface
class RobotState {
public:
    virtual ~RobotState() = default;
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void update() = 0;
    virtual std::string getName() const = 0;
    
    void setContext(class RobotContext* context) {
        context_ = context;
    }

protected:
    RobotContext* context_{nullptr};
};

// Context class that maintains the current state
class RobotContext : public rclcpp::Node {
public:
    explicit RobotContext(const std::string& node_name)
        : Node(node_name)
        , logger_(get_logger()) {
        
        // Initialize publishers
        state_pub_ = create_publisher<std_msgs::msg::String>(
            "robot_state", 10);
        
        // Initialize services
        transition_service_ = create_service<std_srvs::srv::SetBool>(
            "trigger_transition",
            std::bind(&RobotContext::handleTransition, this,
                std::placeholders::_1, std::placeholders::_2));
        
        // Initialize timer for state updates
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotContext::updateState, this));
        
        // Initialize states
        states_["idle"] = std::make_unique<IdleState>();
        states_["moving"] = std::make_unique<MovingState>();
        states_["scanning"] = std::make_unique<ScanningState>();
        states_["charging"] = std::make_unique<ChargingState>();
        states_["error"] = std::make_unique<ErrorState>();
        
        // Set initial state
        transitionTo("idle");
    }

    void transitionTo(const std::string& state_name) {
        if (current_state_) {
            RCLCPP_INFO(logger_, "Exiting state: %s",
                current_state_->getName().c_str());
            current_state_->exit();
        }

        auto it = states_.find(state_name);
        if (it != states_.end()) {
            current_state_ = it->second.get();
            current_state_->setContext(this);
            RCLCPP_INFO(logger_, "Entering state: %s",
                current_state_->getName().c_str());
            current_state_->enter();
            
            // Publish state change
            auto msg = std_msgs::msg::String();
            msg.data = state_name;
            state_pub_->publish(msg);
        } else {
            RCLCPP_ERROR(logger_, "Invalid state: %s", state_name.c_str());
        }
    }

    void updateState() {
        if (current_state_) {
            current_state_->update();
        }
    }

    // Getters for state-specific data
    double getBatteryLevel() const { return battery_level_; }
    geometry_msgs::msg::Pose getCurrentPose() const { return current_pose_; }
    bool hasObstacle() const { return has_obstacle_; }
    std::string getCurrentTask() const { return current_task_; }

private:
    void handleTransition(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        if (current_state_) {
            // Example transition logic
            if (request->data) {
                // Trigger next state based on current conditions
                if (battery_level_ < 0.2) {
                    transitionTo("charging");
                } else if (has_obstacle_) {
                    transitionTo("scanning");
                } else if (!current_task_.empty()) {
                    transitionTo("moving");
                } else {
                    transitionTo("idle");
                }
                response->success = true;
                response->message = "State transition triggered";
            }
        } else {
            response->success = false;
            response->message = "No current state";
        }
    }

    rclcpp::Logger logger_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr transition_service_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::map<std::string, std::unique_ptr<RobotState>> states_;
    RobotState* current_state_{nullptr};

    // Robot state data
    double battery_level_{1.0};
    geometry_msgs::msg::Pose current_pose_;
    bool has_obstacle_{false};
    std::string current_task_;
};

// Concrete states
class IdleState : public RobotState {
public:
    void enter() override {
        RCLCPP_INFO(context_->get_logger(), "Entering Idle state");
    }

    void exit() override {
        RCLCPP_INFO(context_->get_logger(), "Exiting Idle state");
    }

    void update() override {
        // Check conditions for state transition
        if (context_->getBatteryLevel() < 0.2) {
            context_->transitionTo("charging");
        } else if (!context_->getCurrentTask().empty()) {
            context_->transitionTo("moving");
        }
    }

    std::string getName() const override { return "idle"; }
};

class MovingState : public RobotState {
public:
    void enter() override {
        RCLCPP_INFO(context_->get_logger(), "Entering Moving state");
        setupMovement();
    }

    void exit() override {
        RCLCPP_INFO(context_->get_logger(), "Exiting Moving state");
        stopMovement();
    }

    void update() override {
        // Check conditions for state transition
        if (context_->hasObstacle()) {
            context_->transitionTo("scanning");
        } else if (context_->getBatteryLevel() < 0.2) {
            context_->transitionTo("charging");
        }
        
        // Update movement
        updateMovement();
    }

    std::string getName() const override { return "moving"; }

private:
    void setupMovement() {
        // Initialize movement controllers
    }

    void updateMovement() {
        // Update movement based on current task
    }

    void stopMovement() {
        // Stop all movement
    }
};

class ScanningState : public RobotState {
public:
    void enter() override {
        RCLCPP_INFO(context_->get_logger(), "Entering Scanning state");
        startScanning();
    }

    void exit() override {
        RCLCPP_INFO(context_->get_logger(), "Exiting Scanning state");
        stopScanning();
    }

    void update() override {
        // Check conditions for state transition
        if (!context_->hasObstacle()) {
            context_->transitionTo("moving");
        } else if (context_->getBatteryLevel() < 0.2) {
            context_->transitionTo("charging");
        }
        
        // Update scanning
        updateScanning();
    }

    std::string getName() const override { return "scanning"; }

private:
    void startScanning() {
        // Initialize sensors and scanning procedure
    }

    void updateScanning() {
        // Process sensor data and update obstacle detection
    }

    void stopScanning() {
        // Clean up scanning resources
    }
};

class ChargingState : public RobotState {
public:
    void enter() override {
        RCLCPP_INFO(context_->get_logger(), "Entering Charging state");
        startCharging();
    }

    void exit() override {
        RCLCPP_INFO(context_->get_logger(), "Exiting Charging state");
        stopCharging();
    }

    void update() override {
        // Check conditions for state transition
        if (context_->getBatteryLevel() > 0.9) {
            if (!context_->getCurrentTask().empty()) {
                context_->transitionTo("moving");
            } else {
                context_->transitionTo("idle");
            }
        }
        
        // Update charging
        updateCharging();
    }

    std::string getName() const override { return "charging"; }

private:
    void startCharging() {
        // Initialize charging procedure
    }

    void updateCharging() {
        // Monitor charging progress
    }

    void stopCharging() {
        // Clean up charging resources
    }
};

class ErrorState : public RobotState {
public:
    void enter() override {
        RCLCPP_ERROR(context_->get_logger(), "Entering Error state");
        handleError();
    }

    void exit() override {
        RCLCPP_INFO(context_->get_logger(), "Exiting Error state");
        clearError();
    }

    void update() override {
        // Check if error is resolved
        if (checkErrorResolved()) {
            context_->transitionTo("idle");
        }
        
        // Update error handling
        updateErrorHandling();
    }

    std::string getName() const override { return "error"; }

private:
    void handleError() {
        // Initialize error handling procedure
    }

    void updateErrorHandling() {
        // Monitor error status
    }

    void clearError() {
        // Clean up error handling resources
    }

    bool checkErrorResolved() {
        // Check if error conditions are resolved
        return false;
    }
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng State Pattern trong một ROS2 node:

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create and spin robot context node
    auto robot_node = std::make_shared<RobotContext>("robot_state_node");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_node);
    
    RCLCPP_INFO(robot_node->get_logger(), "Robot state node started");
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

## 6. Lợi ích

1. **Clean State Management**:
   - Rõ ràng và dễ maintain
   - Dễ dàng thêm states mới
   - Encapsulated state behavior

2. **Improved Code Organization**:
   - Separate state logic
   - Reduced complexity
   - Better testability

3. **Flexible Transitions**:
   - Dynamic state changes
   - Condition-based transitions
   - Clear transition logic

## 7. Khi nào sử dụng

- Complex state machines
- Multiple operation modes
- State-dependent behavior
- Clear state transitions
- Error handling states
- Mode switching requirements

## 8. Lưu ý

1. Thiết kế:
   - State granularity
   - Transition logic
   - State data sharing
   - Error handling

2. Implementation:
   - State initialization
   - Resource management
   - Thread safety
   - Performance impact

3. Trong ROS2:
   - Node lifecycle
   - Message handling
   - Service integration
   - State publishing 