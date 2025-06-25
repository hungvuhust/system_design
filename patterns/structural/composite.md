# Composite Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Composite Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, cho phép tổ chức các đối tượng thành cấu trúc cây phân cấp. Pattern này đối xử với từng đối tượng riêng lẻ và nhóm các đối tượng theo cùng một cách.

Trong ROS2 và robotics, Composite Pattern thường được sử dụng để:
- Xây dựng hệ thống điều khiển phân cấp cho robot
- Quản lý các task và subtask trong mission planning
- Tổ chức các sensor và actuator trong robot
- Xây dựng behavior trees cho robot navigation

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần quản lý nhiều thành phần có quan hệ phân cấp, ví dụ:
- Một mission có nhiều task, mỗi task có nhiều subtask
- Một robot có nhiều joint, mỗi joint có nhiều motor và sensor
- Một navigation system có nhiều behavior, mỗi behavior có nhiều action

Việc quản lý riêng lẻ từng thành phần sẽ dẫn đến:
- Code phức tạp và khó bảo trì
- Khó mở rộng và thêm tính năng mới
- Khó tái sử dụng code

## 3. Giải pháp

Composite Pattern giải quyết vấn đề bằng cách:
1. Tạo interface chung cho tất cả thành phần
2. Tổ chức thành phần theo cấu trúc cây
3. Xử lý thống nhất giữa leaf nodes và composite nodes

## 4. Ví dụ thực tế: Robot Task Management

```cpp
// Component interface
class RobotTask {
public:
    virtual ~RobotTask() = default;
    virtual void execute() = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
    virtual bool isDone() const = 0;
    virtual void addSubtask(std::shared_ptr<RobotTask> task) {
        throw std::runtime_error("Cannot add subtask to leaf task");
    }
    virtual void removeSubtask(std::shared_ptr<RobotTask> task) {
        throw std::runtime_error("Cannot remove subtask from leaf task");
    }
};

// Leaf task: Move to position
class MoveToPositionTask : public RobotTask {
public:
    MoveToPositionTask(const geometry_msgs::msg::Pose& target)
        : target_pose_(target), is_done_(false) {}

    void execute() override {
        RCLCPP_INFO(logger_, "Moving to position: x=%.2f, y=%.2f",
                   target_pose_.position.x, target_pose_.position.y);
        // Implement movement logic
        is_done_ = true;
    }

    void pause() override {
        RCLCPP_INFO(logger_, "Pausing movement");
        // Implement pause logic
    }

    void resume() override {
        RCLCPP_INFO(logger_, "Resuming movement");
        // Implement resume logic
    }

    bool isDone() const override {
        return is_done_;
    }

private:
    geometry_msgs::msg::Pose target_pose_;
    bool is_done_;
    rclcpp::Logger logger_ = rclcpp::get_logger("MoveToPositionTask");
};

// Leaf task: Grab object
class GrabObjectTask : public RobotTask {
public:
    GrabObjectTask(const std::string& object_id)
        : object_id_(object_id), is_done_(false) {}

    void execute() override {
        RCLCPP_INFO(logger_, "Grabbing object: %s", object_id_.c_str());
        // Implement grabbing logic
        is_done_ = true;
    }

    void pause() override {
        RCLCPP_INFO(logger_, "Pausing grab operation");
        // Implement pause logic
    }

    void resume() override {
        RCLCPP_INFO(logger_, "Resuming grab operation");
        // Implement resume logic
    }

    bool isDone() const override {
        return is_done_;
    }

private:
    std::string object_id_;
    bool is_done_;
    rclcpp::Logger logger_ = rclcpp::get_logger("GrabObjectTask");
};

// Composite: Task sequence
class TaskSequence : public RobotTask {
public:
    void execute() override {
        RCLCPP_INFO(logger_, "Executing task sequence");
        for (auto& task : subtasks_) {
            if (!task->isDone()) {
                task->execute();
            }
        }
    }

    void pause() override {
        RCLCPP_INFO(logger_, "Pausing task sequence");
        for (auto& task : subtasks_) {
            task->pause();
        }
    }

    void resume() override {
        RCLCPP_INFO(logger_, "Resuming task sequence");
        for (auto& task : subtasks_) {
            if (!task->isDone()) {
                task->resume();
            }
        }
    }

    bool isDone() const override {
        return std::all_of(subtasks_.begin(), subtasks_.end(),
                          [](const auto& task) { return task->isDone(); });
    }

    void addSubtask(std::shared_ptr<RobotTask> task) override {
        subtasks_.push_back(task);
    }

    void removeSubtask(std::shared_ptr<RobotTask> task) override {
        subtasks_.erase(
            std::remove(subtasks_.begin(), subtasks_.end(), task),
            subtasks_.end());
    }

private:
    std::vector<std::shared_ptr<RobotTask>> subtasks_;
    rclcpp::Logger logger_ = rclcpp::get_logger("TaskSequence");
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Composite Pattern trong một ROS2 node:

```cpp
class RobotTaskExecutor : public rclcpp::Node {
public:
    RobotTaskExecutor() : Node("robot_task_executor") {
        // Khởi tạo task sequence
        auto pick_and_place = std::make_shared<TaskSequence>();

        // Tạo subtasks
        auto move_to_object = std::make_shared<MoveToPositionTask>(
            createPose(1.0, 1.0, 0.0));
        auto grab_object = std::make_shared<GrabObjectTask>("box_1");
        auto move_to_target = std::make_shared<MoveToPositionTask>(
            createPose(2.0, 2.0, 0.0));

        // Thêm subtasks vào sequence
        pick_and_place->addSubtask(move_to_object);
        pick_and_place->addSubtask(grab_object);
        pick_and_place->addSubtask(move_to_target);

        // Thực thi task sequence
        pick_and_place->execute();
    }

private:
    geometry_msgs::msg::Pose createPose(double x, double y, double z) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    }
};
```

## 6. Ứng dụng trong Behavior Trees

Composite Pattern là nền tảng của Behavior Trees, một công cụ phổ biến trong robot navigation:

```cpp
// Component interface
class BehaviorNode {
public:
    virtual ~BehaviorNode() = default;
    virtual BehaviorStatus tick() = 0;
    virtual void reset() = 0;
};

// Leaf node: Action
class MoveAction : public BehaviorNode {
public:
    BehaviorStatus tick() override {
        // Implement movement logic
        return BehaviorStatus::SUCCESS;
    }

    void reset() override {
        // Reset internal state
    }
};

// Composite node: Sequence
class SequenceNode : public BehaviorNode {
public:
    BehaviorStatus tick() override {
        for (auto& child : children_) {
            auto status = child->tick();
            if (status != BehaviorStatus::SUCCESS) {
                return status;
            }
        }
        return BehaviorStatus::SUCCESS;
    }

    void reset() override {
        for (auto& child : children_) {
            child->reset();
        }
    }

    void addChild(std::shared_ptr<BehaviorNode> child) {
        children_.push_back(child);
    }

private:
    std::vector<std::shared_ptr<BehaviorNode>> children_;
};
```

## 7. Lợi ích

1. **Tính linh hoạt**: Dễ dàng thêm/xóa/thay đổi các thành phần
2. **Code sạch**: Interface thống nhất cho mọi thành phần
3. **Tái sử dụng**: Có thể tái sử dụng các thành phần trong nhiều context
4. **Mở rộng**: Dễ dàng thêm loại task/behavior mới

## 8. Khi nào sử dụng

- Khi cần tổ chức dữ liệu theo cấu trúc cây
- Khi muốn client xử lý đồng nhất các đối tượng đơn và nhóm
- Khi cần xây dựng hệ thống task/behavior phức tạp
- Khi cần quản lý các thành phần có quan hệ cha-con

## 9. Lưu ý

1. Không nên sử dụng Composite Pattern khi:
   - Cấu trúc dữ liệu đơn giản, không có quan hệ phân cấp
   - Không cần xử lý đồng nhất giữa đối tượng đơn và nhóm

2. Cần chú ý:
   - Xử lý lỗi khi thao tác với leaf nodes
   - Quản lý memory khi sử dụng con trỏ
   - Đảm bảo thread safety trong môi trường đa luồng

3. Trong ROS2:
   - Kết hợp với các pattern khác như Observer để theo dõi trạng thái
   - Sử dụng smart pointers để quản lý memory
   - Tận dụng ROS2 logging system để debug
