# Memento Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Memento Pattern là một behavioral pattern cho phép lưu trữ và khôi phục trạng thái của một object mà không vi phạm encapsulation. Trong ROS2 và robotics, pattern này thường được sử dụng cho:

- Robot state recovery
- Mission checkpointing
- Configuration management
- Trajectory rollback
- Error recovery
- System snapshots

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Lưu trữ trạng thái robot
- Khôi phục sau lỗi
- Undo/redo operations
- Checkpoint trong nhiệm vụ phức tạp
- Backup configurations
- Rollback khi gặp lỗi

## 3. Giải pháp

Memento Pattern giải quyết các vấn đề trên bằng cách:
1. Tạo snapshot của object state
2. Lưu trữ snapshot an toàn
3. Khôi phục state khi cần
4. Maintain encapsulation

## 4. Ví dụ thực tế: Robot Mission Checkpointing System

```cpp
// State class to be saved
struct RobotState {
    geometry_msgs::msg::Pose pose;
    std::vector<double> joint_positions;
    bool gripper_closed;
    std::string current_task;
    std::map<std::string, double> parameters;
    rclcpp::Time timestamp;

    // Serialization support
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar & pose;
        ar & joint_positions;
        ar & gripper_closed;
        ar & current_task;
        ar & parameters;
        ar & timestamp;
    }
};

// Memento class
class RobotStateMemento {
public:
    explicit RobotStateMemento(const RobotState& state)
        : state_(state) {}

    RobotState getState() const { return state_; }

private:
    RobotState state_;
    friend class RobotMissionCaretaker;  // Allow caretaker to access state
};

// Originator class
class RobotMissionExecutor {
public:
    explicit RobotMissionExecutor(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , logger_(node->get_logger()) {
        // Initialize subscribers
        pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "robot_pose", 10,
            std::bind(&RobotMissionExecutor::poseCallback, this, std::placeholders::_1));

        joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&RobotMissionExecutor::jointStateCallback, this, std::placeholders::_1));

        // Initialize service clients
        gripper_client_ = node->create_client<control_msgs::srv::GripperCommand>(
            "gripper_command");
    }

    // Create a memento
    std::shared_ptr<RobotStateMemento> createMemento() {
        RobotState current_state;
        current_state.pose = current_pose_;
        current_state.joint_positions = current_joints_;
        current_state.gripper_closed = gripper_closed_;
        current_state.current_task = current_task_;
        current_state.parameters = parameters_;
        current_state.timestamp = node_->now();

        return std::make_shared<RobotStateMemento>(current_state);
    }

    // Restore from a memento
    void restoreFromMemento(const std::shared_ptr<RobotStateMemento>& memento) {
        if (!memento) {
            RCLCPP_ERROR(logger_, "Cannot restore from null memento");
            return;
        }

        auto state = memento->getState();
        RCLCPP_INFO(logger_, "Restoring state from timestamp: %f",
            state.timestamp.seconds());

        // Restore pose
        geometry_msgs::msg::PoseStamped pose_cmd;
        pose_cmd.header.frame_id = "map";
        pose_cmd.header.stamp = node_->now();
        pose_cmd.pose = state.pose;
        moveToPose(pose_cmd);

        // Restore joint positions
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = state.joint_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        traj.points.push_back(point);
        moveJoints(traj);

        // Restore gripper state
        setGripper(state.gripper_closed);

        // Restore other state
        current_task_ = state.current_task;
        parameters_ = state.parameters;

        RCLCPP_INFO(logger_, "State restored successfully");
    }

    // Save state to file
    bool saveToFile(const std::string& filename) {
        try {
            auto memento = createMemento();
            std::ofstream ofs(filename, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << memento->getState();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to save state: %s", e.what());
            return false;
        }
    }

    // Load state from file
    bool loadFromFile(const std::string& filename) {
        try {
            RobotState state;
            std::ifstream ifs(filename, std::ios::binary);
            boost::archive::binary_iarchive ia(ifs);
            ia >> state;
            auto memento = std::make_shared<RobotStateMemento>(state);
            restoreFromMemento(memento);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to load state: %s", e.what());
            return false;
        }
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joints_ = msg->position;
        joint_names_ = msg->name;
    }

    void moveToPose(const geometry_msgs::msg::PoseStamped& pose) {
        // Implement move to pose using navigation or motion planning
        RCLCPP_INFO(logger_, "Moving to pose: x=%.2f, y=%.2f",
            pose.pose.position.x, pose.pose.position.y);
    }

    void moveJoints(const trajectory_msgs::msg::JointTrajectory& traj) {
        // Implement joint trajectory execution
        RCLCPP_INFO(logger_, "Moving joints to target positions");
    }

    void setGripper(bool close) {
        auto request = std::make_shared<control_msgs::srv::GripperCommand::Request>();
        request->command.position = close ? 0.0 : 0.1;
        request->command.max_effort = 50.0;

        auto result = gripper_client_->async_send_request(request);
        gripper_closed_ = close;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<control_msgs::srv::GripperCommand>::SharedPtr gripper_client_;

    geometry_msgs::msg::Pose current_pose_;
    std::vector<double> current_joints_;
    std::vector<std::string> joint_names_;
    bool gripper_closed_{false};
    std::string current_task_;
    std::map<std::string, double> parameters_;
};

// Caretaker class
class RobotMissionCaretaker {
public:
    void addCheckpoint(const std::string& name,
                      std::shared_ptr<RobotStateMemento> memento) {
        checkpoints_[name] = memento;
    }

    std::shared_ptr<RobotStateMemento> getCheckpoint(const std::string& name) {
        auto it = checkpoints_.find(name);
        if (it != checkpoints_.end()) {
            return it->second;
        }
        return nullptr;
    }

    void removeCheckpoint(const std::string& name) {
        checkpoints_.erase(name);
    }

    std::vector<std::string> listCheckpoints() const {
        std::vector<std::string> names;
        for (const auto& [name, _] : checkpoints_) {
            names.push_back(name);
        }
        return names;
    }

private:
    std::map<std::string, std::shared_ptr<RobotStateMemento>> checkpoints_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Memento Pattern trong một ROS2 node:

```cpp
class RobotMissionNode : public rclcpp::Node {
public:
    RobotMissionNode() : Node("robot_mission_node") {
        // Create executor and caretaker
        executor_ = std::make_shared<RobotMissionExecutor>(shared_from_this());
        caretaker_ = std::make_shared<RobotMissionCaretaker>();

        // Create services
        checkpoint_service_ = create_service<std_srvs::srv::SetBool>(
            "create_checkpoint",
            std::bind(&RobotMissionNode::handleCreateCheckpoint, this,
                std::placeholders::_1, std::placeholders::_2));

        restore_service_ = create_service<std_srvs::srv::Trigger>(
            "restore_last_checkpoint",
            std::bind(&RobotMissionNode::handleRestore, this,
                std::placeholders::_1, std::placeholders::_2));

        // Create timer for periodic checkpoints
        auto checkpoint_interval = declare_parameter("checkpoint_interval", 300.0);
        checkpoint_timer_ = create_wall_timer(
            std::chrono::duration<double>(checkpoint_interval),
            std::bind(&RobotMissionNode::createPeriodicCheckpoint, this));

        RCLCPP_INFO(get_logger(), "Robot mission node initialized");
    }

private:
    void handleCreateCheckpoint(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        try {
            auto memento = executor_->createMemento();
            std::string checkpoint_name = "checkpoint_" +
                std::to_string(rclcpp::Clock().now().seconds());
            caretaker_->addCheckpoint(checkpoint_name, memento);

            response->success = true;
            response->message = "Created checkpoint: " + checkpoint_name;
            RCLCPP_INFO(get_logger(), response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to create checkpoint: " + std::string(e.what());
            RCLCPP_ERROR(get_logger(), response->message.c_str());
        }
    }

    void handleRestore(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        auto checkpoints = caretaker_->listCheckpoints();
        if (checkpoints.empty()) {
            response->success = false;
            response->message = "No checkpoints available";
            return;
        }

        // Get latest checkpoint
        auto latest = checkpoints.back();
        auto memento = caretaker_->getCheckpoint(latest);
        
        try {
            executor_->restoreFromMemento(memento);
            response->success = true;
            response->message = "Restored from checkpoint: " + latest;
            RCLCPP_INFO(get_logger(), response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to restore: " + std::string(e.what());
            RCLCPP_ERROR(get_logger(), response->message.c_str());
        }
    }

    void createPeriodicCheckpoint() {
        auto memento = executor_->createMemento();
        std::string checkpoint_name = "periodic_" +
            std::to_string(rclcpp::Clock().now().seconds());
        caretaker_->addCheckpoint(checkpoint_name, memento);
        
        // Keep only last 5 periodic checkpoints
        auto checkpoints = caretaker_->listCheckpoints();
        while (checkpoints.size() > 5) {
            caretaker_->removeCheckpoint(checkpoints.front());
            checkpoints.erase(checkpoints.begin());
        }

        RCLCPP_INFO(get_logger(), "Created periodic checkpoint: %s",
            checkpoint_name.c_str());
    }

    std::shared_ptr<RobotMissionExecutor> executor_;
    std::shared_ptr<RobotMissionCaretaker> caretaker_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr checkpoint_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_service_;
    rclcpp::TimerBase::SharedPtr checkpoint_timer_;
};
```

## 6. Lợi ích

1. **State Management**:
   - Lưu trữ state an toàn
   - Khôi phục state dễ dàng
   - Maintain encapsulation

2. **Error Recovery**:
   - Rollback khi lỗi
   - Checkpoint system
   - Safe state restoration

3. **Flexibility**:
   - Multiple save points
   - Selective state restore
   - Persistent storage

## 7. Khi nào sử dụng

- Cần lưu trữ system state
- Implement undo/redo
- Cần checkpoint mechanism
- Error recovery scenarios
- State rollback requirements
- Configuration management

## 8. Lưu ý

1. Thiết kế:
   - Memory usage
   - State consistency
   - Serialization format
   - Storage strategy

2. Implementation:
   - Thread safety
   - Resource management
   - Error handling
   - Performance impact

3. Trong ROS2:
   - Message serialization
   - Node lifecycle
   - Resource cleanup
   - State validation 