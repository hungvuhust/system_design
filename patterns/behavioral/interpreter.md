# Interpreter Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Interpreter Pattern là một mẫu thiết kế thuộc nhóm Behavioral Pattern, dùng để định nghĩa ngữ pháp cho một ngôn ngữ và cung cấp một interpreter để diễn giải các câu trong ngôn ngữ đó. Trong ROS2 và robotics, pattern này thường được sử dụng cho:
- Task specification languages
- Robot command languages
- Configuration parsers
- Motion planning DSLs
- Behavior tree interpreters

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Định nghĩa ngôn ngữ cho robot tasks
- Parse và thực thi các command sequences
- Interpret các behavior specifications
- Xử lý các custom configuration formats
- Translate high-level commands thành low-level actions

## 3. Giải pháp

Interpreter Pattern giải quyết các vấn đề trên bằng cách:
1. Định nghĩa grammar cho ngôn ngữ
2. Tạo abstract syntax tree (AST)
3. Implement interpreter cho mỗi rule
4. Parse và execute các expressions

## 4. Ví dụ thực tế: Robot Task Language

```cpp
// Abstract Expression
class TaskExpression {
public:
    virtual ~TaskExpression() = default;
    virtual bool interpret(std::shared_ptr<rclcpp::Node> node) = 0;
    virtual std::string toString() const = 0;
};

// Terminal Expression - Move Command
class MoveCommand : public TaskExpression {
public:
    MoveCommand(double x, double y, double theta)
        : x_(x), y_(y), theta_(theta) {}

    bool interpret(std::shared_ptr<rclcpp::Node> node) override {
        RCLCPP_INFO(node->get_logger(),
            "Executing move command: x=%.2f, y=%.2f, theta=%.2f",
            x_, y_, theta_);

        // Create action client
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node, "navigate_to_pose");

        // Wait for action server
        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Navigation action server not available");
            return false;
        }

        // Create goal
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.pose.position.x = x_;
        goal.pose.pose.position.y = y_;
        
        // Convert theta to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        goal.pose.pose.orientation = tf2::toMsg(q);

        // Send goal and wait for result
        auto future = action_client->async_send_goal(goal);
        if (rclcpp::spin_until_future_complete(node, future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send navigation goal");
            return false;
        }

        return true;
    }

    std::string toString() const override {
        std::stringstream ss;
        ss << "MOVE(" << x_ << ", " << y_ << ", " << theta_ << ")";
        return ss.str();
    }

private:
    double x_, y_, theta_;
};

// Terminal Expression - Gripper Command
class GripperCommand : public TaskExpression {
public:
    explicit GripperCommand(bool close) : close_(close) {}

    bool interpret(std::shared_ptr<rclcpp::Node> node) override {
        RCLCPP_INFO(node->get_logger(),
            "Executing gripper command: %s", close_ ? "close" : "open");

        // Create service client
        auto client = node->create_client<std_srvs::srv::SetBool>("gripper_command");

        // Wait for service
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Gripper service not available");
            return false;
        }

        // Send request
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = close_;

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send gripper command");
            return false;
        }

        return future.get()->success;
    }

    std::string toString() const override {
        return close_ ? "GRIP()" : "RELEASE()";
    }

private:
    bool close_;
};

// Non-terminal Expression - Sequence
class TaskSequence : public TaskExpression {
public:
    void addTask(std::shared_ptr<TaskExpression> task) {
        tasks_.push_back(task);
    }

    bool interpret(std::shared_ptr<rclcpp::Node> node) override {
        RCLCPP_INFO(node->get_logger(), "Executing task sequence");
        
        for (const auto& task : tasks_) {
            RCLCPP_INFO(node->get_logger(), "Executing: %s", task->toString().c_str());
            if (!task->interpret(node)) {
                RCLCPP_ERROR(node->get_logger(), "Task failed: %s", task->toString().c_str());
                return false;
            }
        }
        return true;
    }

    std::string toString() const override {
        std::stringstream ss;
        ss << "SEQUENCE(";
        for (size_t i = 0; i < tasks_.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << tasks_[i]->toString();
        }
        ss << ")";
        return ss.str();
    }

private:
    std::vector<std::shared_ptr<TaskExpression>> tasks_;
};

// Non-terminal Expression - Repeat
class RepeatTask : public TaskExpression {
public:
    RepeatTask(std::shared_ptr<TaskExpression> task, int count)
        : task_(task), count_(count) {}

    bool interpret(std::shared_ptr<rclcpp::Node> node) override {
        RCLCPP_INFO(node->get_logger(),
            "Repeating task %d times: %s", count_, task_->toString().c_str());

        for (int i = 0; i < count_; ++i) {
            RCLCPP_INFO(node->get_logger(), "Iteration %d/%d", i + 1, count_);
            if (!task_->interpret(node)) {
                RCLCPP_ERROR(node->get_logger(),
                    "Repeat task failed on iteration %d", i + 1);
                return false;
            }
        }
        return true;
    }

    std::string toString() const override {
        std::stringstream ss;
        ss << "REPEAT(" << count_ << ", " << task_->toString() << ")";
        return ss.str();
    }

private:
    std::shared_ptr<TaskExpression> task_;
    int count_;
};

// Parser for the task language
class TaskParser {
public:
    std::shared_ptr<TaskExpression> parseCommand(const std::string& cmd) {
        std::istringstream iss(cmd);
        std::string token;
        iss >> token;

        if (token == "MOVE") {
            double x, y, theta;
            char dummy;  // For parentheses
            iss >> dummy >> x >> dummy >> y >> dummy >> theta >> dummy;
            return std::make_shared<MoveCommand>(x, y, theta);
        }
        else if (token == "GRIP") {
            return std::make_shared<GripperCommand>(true);
        }
        else if (token == "RELEASE") {
            return std::make_shared<GripperCommand>(false);
        }
        else if (token == "REPEAT") {
            int count;
            std::string subCmd;
            char dummy;
            iss >> dummy >> count >> dummy;
            std::getline(iss, subCmd, ')');
            return std::make_shared<RepeatTask>(parseCommand(subCmd), count);
        }
        
        throw std::runtime_error("Unknown command: " + token);
    }

    std::shared_ptr<TaskSequence> parseSequence(const std::vector<std::string>& cmds) {
        auto sequence = std::make_shared<TaskSequence>();
        for (const auto& cmd : cmds) {
            sequence->addTask(parseCommand(cmd));
        }
        return sequence;
    }
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Interpreter Pattern trong một ROS2 node:

```cpp
class TaskInterpreterNode : public rclcpp::Node {
public:
    TaskInterpreterNode() : Node("task_interpreter") {
        // Create service to execute task sequences
        execute_service_ = create_service<custom_msgs::srv::ExecuteTaskSequence>(
            "execute_task_sequence",
            std::bind(&TaskInterpreterNode::handleExecuteRequest, this,
                     std::placeholders::_1, std::placeholders::_2));

        parser_ = std::make_unique<TaskParser>();
        RCLCPP_INFO(get_logger(), "Task interpreter node initialized");
    }

private:
    void handleExecuteRequest(
        const custom_msgs::srv::ExecuteTaskSequence::Request::SharedPtr request,
        custom_msgs::srv::ExecuteTaskSequence::Response::SharedPtr response) {
        
        try {
            // Parse task sequence
            auto sequence = parser_->parseSequence(request->commands);
            RCLCPP_INFO(get_logger(), "Parsed sequence: %s", sequence->toString().c_str());

            // Execute sequence
            response->success = sequence->interpret(shared_from_this());
            if (response->success) {
                response->message = "Task sequence completed successfully";
            } else {
                response->message = "Task sequence failed";
            }
        }
        catch (const std::exception& e) {
            response->success = false;
            response->message = "Parse error: " + std::string(e.what());
            RCLCPP_ERROR(get_logger(), "Parse error: %s", e.what());
        }
    }

    std::unique_ptr<TaskParser> parser_;
    rclcpp::Service<custom_msgs::srv::ExecuteTaskSequence>::SharedPtr execute_service_;
};

// Example usage
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskInterpreterNode>();

    // Example task sequence
    std::vector<std::string> commands = {
        "MOVE(1.0, 2.0, 0.0)",
        "GRIP()",
        "MOVE(0.0, 0.0, 3.14)",
        "RELEASE()",
        "REPEAT(2, MOVE(1.0, 0.0, 0.0))"
    };

    // Create request
    auto client = node->create_client<custom_msgs::srv::ExecuteTaskSequence>(
        "execute_task_sequence");
    auto request = std::make_shared<custom_msgs::srv::ExecuteTaskSequence::Request>();
    request->commands = commands;

    // Send request
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        RCLCPP_INFO(node->get_logger(),
            "Task execution %s: %s",
            result->success ? "succeeded" : "failed",
            result->message.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
```

## 6. Lợi ích

1. **Flexibility**:
   - Dễ dàng thêm expressions mới
   - Customize behavior dễ dàng
   - Mở rộng ngôn ngữ linh hoạt

2. **Abstraction**:
   - High-level task specification
   - Tách biệt grammar và execution
   - Clean separation of concerns

3. **Reusability**:
   - Tái sử dụng expressions
   - Combine expressions
   - Build complex behaviors

## 7. Khi nào sử dụng

- Cần định nghĩa domain-specific language
- Xử lý task specifications phức tạp
- Parse structured commands
- Build behavior trees
- Implement custom configuration formats

## 8. Lưu ý

1. Thiết kế:
   - Keep grammar simple
   - Define clear semantics
   - Handle errors gracefully
   - Consider extensibility

2. Performance:
   - Optimize parsing
   - Cache parsed expressions
   - Avoid deep nesting
   - Monitor memory usage

3. Trong ROS2:
   - Handle timeouts
   - Consider action preemption
   - Implement proper error recovery
   - Follow ROS2 conventions 