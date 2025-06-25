# Command Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Command Pattern là một behavioral pattern cho phép đóng gói một request thành một object, cho phép bạn tham số hóa clients với các requests khác nhau, queue hoặc log requests, và hỗ trợ các thao tác undo. Trong ROS2 và robotics, pattern này thường được sử dụng cho:

- Robot task execution
- Motion planning
- Sensor configuration
- Hardware control
- Mission planning
- System recovery

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Quản lý nhiều loại commands khác nhau
- Queue và schedule commands
- Undo/redo operations
- Log và monitor command execution
- Handle command failures
- Coordinate multiple subsystems

## 3. Giải pháp

Command Pattern giải quyết các vấn đề trên bằng cách:
1. Tách biệt object gửi request và object thực hiện request
2. Đóng gói request thành object
3. Cho phép handle requests theo nhiều cách khác nhau
4. Hỗ trợ queuing, logging, và undoing

## 4. Ví dụ thực tế: Robot Task Management System

```cpp
// Command interface
class RobotCommand {
public:
    virtual ~RobotCommand() = default;
    virtual bool execute() = 0;
    virtual bool undo() = 0;
    virtual std::string getName() const = 0;
};

// Concrete commands

// Move command
class MoveCommand : public RobotCommand {
public:
    MoveCommand(std::shared_ptr<rclcpp::Node> node,
                const geometry_msgs::msg::Pose& target)
        : node_(node)
        , target_(target)
        , action_client_(rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node, "navigate_to_pose"))
    {
        // Store current pose for undo
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    bool execute() override {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available");
            return false;
        }

        // Store current pose for potential undo
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            start_pose_ = transform.transform;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Could not get current pose: %s", ex.what());
            return false;
        }

        // Send navigation goal
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_->now();
        goal.pose.pose = target_;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveCommand::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MoveCommand::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveCommand::resultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal, send_goal_options);

        // Wait for result
        std::unique_lock<std::mutex> lock(mutex_);
        if (cv_.wait_for(lock, std::chrono::seconds(60)) == std::cv_status::timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation timed out");
            return false;
        }

        return success_;
    }

    bool undo() override {
        // Create a new goal to return to start pose
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_->now();
        goal.pose.pose.position.x = start_pose_.translation.x;
        goal.pose.pose.position.y = start_pose_.translation.y;
        goal.pose.pose.position.z = start_pose_.translation.z;
        goal.pose.pose.orientation = start_pose_.rotation;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        success_ = false;
        action_client_->async_send_goal(goal, send_goal_options);

        // Wait for result
        std::unique_lock<std::mutex> lock(mutex_);
        if (cv_.wait_for(lock, std::chrono::seconds(60)) == std::cv_status::timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Undo navigation timed out");
            return false;
        }

        return success_;
    }

    std::string getName() const override {
        return "MoveCommand to (" + 
               std::to_string(target_.position.x) + ", " +
               std::to_string(target_.position.y) + ")";
    }

private:
    void goalResponseCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
    }

    void feedbackCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr&,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(node_->get_logger(), "Distance remaining: %.2f",
            feedback->distance_remaining);
    }

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        cv_.notify_one();
    }

    std::shared_ptr<rclcpp::Node> node_;
    geometry_msgs::msg::Pose target_;
    geometry_msgs::msg::Transform start_pose_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool success_{false};
};

// Gripper command
class GripperCommand : public RobotCommand {
public:
    GripperCommand(std::shared_ptr<rclcpp::Node> node, bool close)
        : node_(node)
        , close_(close)
        , client_(node->create_client<control_msgs::srv::GripperCommand>("gripper_command"))
    {}

    bool execute() override {
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Gripper service not available");
            return false;
        }

        auto request = std::make_shared<control_msgs::srv::GripperCommand::Request>();
        request->command.position = close_ ? 0.0 : 0.1;  // 0.0 for close, 0.1 for open
        request->command.max_effort = 50.0;

        auto result = client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(node_, result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call gripper service");
            return false;
        }

        return result.get()->success;
    }

    bool undo() override {
        // Simply do the opposite action
        close_ = !close_;
        return execute();
    }

    std::string getName() const override {
        return std::string("GripperCommand ") + (close_ ? "Close" : "Open");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    bool close_;
    rclcpp::Client<control_msgs::srv::GripperCommand>::SharedPtr client_;
};

// Command invoker with history
class RobotCommandInvoker {
public:
    explicit RobotCommandInvoker(size_t history_size = 10)
        : max_history_size_(history_size) {}

    bool executeCommand(std::shared_ptr<RobotCommand> command) {
        if (command->execute()) {
            if (command_history_.size() >= max_history_size_) {
                command_history_.pop_front();
            }
            command_history_.push_back(command);
            return true;
        }
        return false;
    }

    bool undoLastCommand() {
        if (command_history_.empty()) {
            return false;
        }

        auto command = command_history_.back();
        if (command->undo()) {
            command_history_.pop_back();
            return true;
        }
        return false;
    }

    void clearHistory() {
        command_history_.clear();
    }

    std::vector<std::string> getHistory() const {
        std::vector<std::string> history;
        for (const auto& cmd : command_history_) {
            history.push_back(cmd->getName());
        }
        return history;
    }

private:
    std::deque<std::shared_ptr<RobotCommand>> command_history_;
    size_t max_history_size_;
};

// Macro command for combining multiple commands
class MacroCommand : public RobotCommand {
public:
    void addCommand(std::shared_ptr<RobotCommand> command) {
        commands_.push_back(command);
    }

    bool execute() override {
        for (const auto& command : commands_) {
            if (!command->execute()) {
                // If any command fails, undo all executed commands
                while (!executed_commands_.empty()) {
                    executed_commands_.back()->undo();
                    executed_commands_.pop_back();
                }
                return false;
            }
            executed_commands_.push_back(command);
        }
        return true;
    }

    bool undo() override {
        // Undo commands in reverse order
        for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
            if (!(*it)->undo()) {
                return false;
            }
        }
        return true;
    }

    std::string getName() const override {
        std::string name = "MacroCommand [";
        for (const auto& cmd : commands_) {
            name += cmd->getName() + ", ";
        }
        if (!commands_.empty()) {
            name = name.substr(0, name.length() - 2);  // Remove last ", "
        }
        name += "]";
        return name;
    }

private:
    std::vector<std::shared_ptr<RobotCommand>> commands_;
    std::vector<std::shared_ptr<RobotCommand>> executed_commands_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Command Pattern trong một ROS2 node:

```cpp
class RobotTaskNode : public rclcpp::Node {
public:
    RobotTaskNode() : Node("robot_task_node") {
        command_invoker_ = std::make_unique<RobotCommandInvoker>();

        // Create service servers
        execute_task_server_ = create_service<std_srvs::srv::Trigger>(
            "execute_task",
            std::bind(&RobotTaskNode::handleExecuteTask, this,
                std::placeholders::_1, std::placeholders::_2));

        undo_task_server_ = create_service<std_srvs::srv::Trigger>(
            "undo_task",
            std::bind(&RobotTaskNode::handleUndoTask, this,
                std::placeholders::_1, std::placeholders::_2));

        // Example of creating and executing a macro command
        auto macro = std::make_shared<MacroCommand>();
        
        // Move to object
        geometry_msgs::msg::Pose pickup_pose;
        pickup_pose.position.x = 1.0;
        pickup_pose.position.y = 0.5;
        pickup_pose.orientation.w = 1.0;
        macro->addCommand(std::make_shared<MoveCommand>(
            shared_from_this(), pickup_pose));

        // Open gripper
        macro->addCommand(std::make_shared<GripperCommand>(
            shared_from_this(), false));

        // Move down to grasp
        geometry_msgs::msg::Pose grasp_pose = pickup_pose;
        grasp_pose.position.z -= 0.2;
        macro->addCommand(std::make_shared<MoveCommand>(
            shared_from_this(), grasp_pose));

        // Close gripper
        macro->addCommand(std::make_shared<GripperCommand>(
            shared_from_this(), true));

        // Store the macro command for later execution
        pickup_task_ = macro;
    }

private:
    void handleExecuteTask(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (command_invoker_->executeCommand(pickup_task_)) {
            response->success = true;
            response->message = "Task executed successfully";
        } else {
            response->success = false;
            response->message = "Task execution failed";
        }
    }

    void handleUndoTask(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        if (command_invoker_->undoLastCommand()) {
            response->success = true;
            response->message = "Task undone successfully";
        } else {
            response->success = false;
            response->message = "Failed to undo task";
        }
    }

    std::unique_ptr<RobotCommandInvoker> command_invoker_;
    std::shared_ptr<RobotCommand> pickup_task_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_task_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_task_server_;
};
```

## 6. Lợi ích

1. **Decoupling**:
   - Tách biệt sender và receiver
   - Dễ dàng thêm commands mới
   - Giảm phụ thuộc giữa components

2. **Flexibility**:
   - Parameterize objects với operations
   - Compose commands phức tạp
   - Queue và schedule commands

3. **Extensibility**:
   - Support undo/redo
   - Logging và monitoring
   - Transaction-like behavior

## 7. Khi nào sử dụng

- Cần parameterize objects với operations
- Cần queue, schedule hoặc execute commands remotely
- Cần support undo/redo
- Cần build composite commands
- Muốn tách biệt command execution từ command implementation
- Cần transaction-like behavior

## 8. Lưu ý

1. Thiết kế:
   - Xác định rõ command interface
   - Cân nhắc undo/redo requirements
   - Plan for command composition
   - Consider command history management

2. Implementation:
   - Thread safety
   - Error handling
   - Resource management
   - Command validation

3. Trong ROS2:
   - Action/Service timeouts
   - State management
   - Error recovery
   - Resource cleanup 