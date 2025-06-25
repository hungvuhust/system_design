# Mediator Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Mediator Pattern là một behavioral pattern cho phép giảm sự phụ thuộc giữa các objects bằng cách tạo ra một object trung gian (mediator) để xử lý tương tác giữa chúng. Trong ROS2 và robotics, pattern này thường được sử dụng cho:

- Robot behavior coordination
- Sensor fusion
- Task scheduling
- Resource management
- Multi-robot coordination
- System state management

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Điều phối nhiều subsystems
- Xử lý conflicts giữa các behaviors
- Quản lý shared resources
- Coordinate nhiều robots
- Handle complex state transitions
- Manage dependencies giữa các components

## 3. Giải pháp

Mediator Pattern giải quyết các vấn đề trên bằng cách:
1. Tạo một central coordinator
2. Giảm coupling giữa các components
3. Encapsulate complex interactions
4. Provide single point of control

## 4. Ví dụ thực tế: Robot Task Coordinator

```cpp
// Forward declarations
class RobotComponent;
class NavigationSystem;
class ManipulationSystem;
class PerceptionSystem;
class TaskScheduler;

// Mediator interface
class RobotCoordinator {
public:
    virtual ~RobotCoordinator() = default;
    virtual void registerComponent(const std::string& name,
                                 std::shared_ptr<RobotComponent> component) = 0;
    virtual void notify(const std::string& sender,
                       const std::string& event,
                       const std::shared_ptr<rclcpp::SerializedMessage>& data) = 0;
};

// Component base class
class RobotComponent {
public:
    explicit RobotComponent(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , logger_(node->get_logger()) {}
    
    virtual ~RobotComponent() = default;
    
    void setMediator(std::shared_ptr<RobotCoordinator> mediator) {
        mediator_ = mediator;
    }

    virtual void handleEvent(const std::string& event,
                           const std::shared_ptr<rclcpp::SerializedMessage>& data) = 0;

protected:
    void notifyMediator(const std::string& event,
                       const std::shared_ptr<rclcpp::SerializedMessage>& data) {
        if (mediator_) {
            mediator_->notify(getName(), event, data);
        }
    }

    virtual std::string getName() const = 0;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    std::shared_ptr<RobotCoordinator> mediator_;
};

// Concrete coordinator
class ConcreteRobotCoordinator : public RobotCoordinator {
public:
    explicit ConcreteRobotCoordinator(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , logger_(node->get_logger()) {}

    void registerComponent(const std::string& name,
                         std::shared_ptr<RobotComponent> component) override {
        components_[name] = component;
        component->setMediator(shared_from_this());
    }

    void notify(const std::string& sender,
               const std::string& event,
               const std::shared_ptr<rclcpp::SerializedMessage>& data) override {
        RCLCPP_INFO(logger_, "Received event '%s' from '%s'", event.c_str(), sender.c_str());

        // Handle specific event types
        if (event == "obstacle_detected") {
            handleObstacleDetected(sender, data);
        } else if (event == "target_reached") {
            handleTargetReached(sender, data);
        } else if (event == "object_detected") {
            handleObjectDetected(sender, data);
        } else if (event == "manipulation_complete") {
            handleManipulationComplete(sender, data);
        }
    }

private:
    void handleObstacleDetected(const std::string& sender,
                               const std::shared_ptr<rclcpp::SerializedMessage>& data) {
        // Stop current task
        if (auto scheduler = getComponent<TaskScheduler>("scheduler")) {
            scheduler->handleEvent("pause_task", data);
        }

        // Request new path
        if (auto navigation = getComponent<NavigationSystem>("navigation")) {
            navigation->handleEvent("replan_path", data);
        }
    }

    void handleTargetReached(const std::string& sender,
                            const std::shared_ptr<rclcpp::SerializedMessage>& data) {
        // Start perception
        if (auto perception = getComponent<PerceptionSystem>("perception")) {
            perception->handleEvent("start_detection", data);
        }
    }

    void handleObjectDetected(const std::string& sender,
                            const std::shared_ptr<rclcpp::SerializedMessage>& data) {
        // Start manipulation
        if (auto manipulation = getComponent<ManipulationSystem>("manipulation")) {
            manipulation->handleEvent("start_grasp", data);
        }
    }

    void handleManipulationComplete(const std::string& sender,
                                  const std::shared_ptr<rclcpp::SerializedMessage>& data) {
        // Continue with next task
        if (auto scheduler = getComponent<TaskScheduler>("scheduler")) {
            scheduler->handleEvent("continue_task", data);
        }
    }

    template<typename T>
    std::shared_ptr<T> getComponent(const std::string& name) {
        auto it = components_.find(name);
        if (it != components_.end()) {
            return std::dynamic_pointer_cast<T>(it->second);
        }
        return nullptr;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    std::map<std::string, std::shared_ptr<RobotComponent>> components_;
};

// Concrete components
class NavigationSystem : public RobotComponent {
public:
    explicit NavigationSystem(std::shared_ptr<rclcpp::Node> node)
        : RobotComponent(node) {
        // Setup navigation action client
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node, "navigate_to_pose");
        
        // Setup collision detection subscription
        collision_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&NavigationSystem::laserCallback, this, std::placeholders::_1));
    }

    void handleEvent(const std::string& event,
                    const std::shared_ptr<rclcpp::SerializedMessage>& data) override {
        if (event == "replan_path") {
            RCLCPP_INFO(logger_, "Replanning path due to obstacle");
            // Implement path replanning logic
        }
    }

    std::string getName() const override { return "navigation"; }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Simple obstacle detection
        auto min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        if (min_distance < 0.5) {  // 0.5m threshold
            auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
            // Serialize obstacle data
            notifyMediator("obstacle_detected", serialized_msg);
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr collision_sub_;
};

class PerceptionSystem : public RobotComponent {
public:
    explicit PerceptionSystem(std::shared_ptr<rclcpp::Node> node)
        : RobotComponent(node) {
        // Setup object detection client
        detect_client_ = node->create_client<vision_msgs::srv::Detect>("detect_objects");
    }

    void handleEvent(const std::string& event,
                    const std::shared_ptr<rclcpp::SerializedMessage>& data) override {
        if (event == "start_detection") {
            RCLCPP_INFO(logger_, "Starting object detection");
            detectObjects();
        }
    }

    std::string getName() const override { return "perception"; }

private:
    void detectObjects() {
        auto request = std::make_shared<vision_msgs::srv::Detect::Request>();
        auto result = detect_client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (!result.get()->detections.empty()) {
                auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                // Serialize detection data
                notifyMediator("object_detected", serialized_msg);
            }
        }
    }

    rclcpp::Client<vision_msgs::srv::Detect>::SharedPtr detect_client_;
};

class ManipulationSystem : public RobotComponent {
public:
    explicit ManipulationSystem(std::shared_ptr<rclcpp::Node> node)
        : RobotComponent(node) {
        // Setup manipulation action client
        manip_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node, "gripper_command");
    }

    void handleEvent(const std::string& event,
                    const std::shared_ptr<rclcpp::SerializedMessage>& data) override {
        if (event == "start_grasp") {
            RCLCPP_INFO(logger_, "Starting grasp execution");
            executeGrasp();
        }
    }

    std::string getName() const override { return "manipulation"; }

private:
    void executeGrasp() {
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = 0.0;  // Close gripper
        goal.command.max_effort = 50.0;

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                    notifyMediator("manipulation_complete", serialized_msg);
                }
            };

        manip_client_->async_send_goal(goal, send_goal_options);
    }

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr manip_client_;
};

class TaskScheduler : public RobotComponent {
public:
    explicit TaskScheduler(std::shared_ptr<rclcpp::Node> node)
        : RobotComponent(node)
        , paused_(false) {}

    void handleEvent(const std::string& event,
                    const std::shared_ptr<rclcpp::SerializedMessage>& data) override {
        if (event == "pause_task") {
            RCLCPP_INFO(logger_, "Pausing current task");
            paused_ = true;
        } else if (event == "continue_task") {
            RCLCPP_INFO(logger_, "Continuing with next task");
            paused_ = false;
            executeNextTask();
        }
    }

    std::string getName() const override { return "scheduler"; }

private:
    void executeNextTask() {
        if (!paused_ && !task_queue_.empty()) {
            auto task = task_queue_.front();
            task_queue_.pop();
            // Execute task
        }
    }

    std::queue<std::string> task_queue_;
    bool paused_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Mediator Pattern trong một ROS2 node:

```cpp
class RobotSystemNode : public rclcpp::Node {
public:
    RobotSystemNode() : Node("robot_system") {
        // Create coordinator
        coordinator_ = std::make_shared<ConcreteRobotCoordinator>(shared_from_this());

        // Create and register components
        auto navigation = std::make_shared<NavigationSystem>(shared_from_this());
        auto perception = std::make_shared<PerceptionSystem>(shared_from_this());
        auto manipulation = std::make_shared<ManipulationSystem>(shared_from_this());
        auto scheduler = std::make_shared<TaskScheduler>(shared_from_this());

        coordinator_->registerComponent("navigation", navigation);
        coordinator_->registerComponent("perception", perception);
        coordinator_->registerComponent("manipulation", manipulation);
        coordinator_->registerComponent("scheduler", scheduler);

        RCLCPP_INFO(get_logger(), "Robot system initialized with all components");
    }

private:
    std::shared_ptr<ConcreteRobotCoordinator> coordinator_;
};
```

## 6. Lợi ích

1. **Loose Coupling**:
   - Giảm dependencies giữa các components
   - Dễ thay đổi components
   - Dễ maintain và test

2. **Centralized Control**:
   - Single point of coordination
   - Dễ quản lý complex logic
   - Dễ debug và monitor

3. **Flexibility**:
   - Dễ thêm/bớt components
   - Dễ thay đổi interaction logic
   - Support dynamic behavior

## 7. Khi nào sử dụng

- Có nhiều components tương tác với nhau
- Logic điều phối phức tạp
- Cần giảm coupling giữa các components
- Cần central point of control
- Quản lý shared resources
- Handle complex state transitions

## 8. Lưu ý

1. Thiết kế:
   - Xác định rõ responsibilities
   - Cân nhắc scalability
   - Plan for error handling
   - Consider threading model

2. Implementation:
   - Thread safety
   - Error propagation
   - Performance impact
   - Memory management

3. Trong ROS2:
   - Message passing overhead
   - Component lifecycle
   - Resource cleanup
   - Error recovery 