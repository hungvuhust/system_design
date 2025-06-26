# Facade Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Facade Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, cung cấp một interface đơn giản cho một hệ thống phức tạp. Pattern này tạo ra một lớp facade đóng vai trò như một "mặt tiền" đơn giản, che giấu sự phức tạp của hệ thống bên dưới.

Trong ROS2 và robotics, Facade Pattern thường được sử dụng để:

- Đơn giản hóa việc tương tác với robot
- Gom nhóm nhiều ROS2 service/action thành một interface thống nhất
- Tạo API cao cấp cho hệ thống phức tạp
- Đóng gói các subsystem thành một interface dễ sử dụng

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các hệ thống phức tạp với nhiều thành phần, ví dụ:

- Robot arm với nhiều joint controllers
- Navigation system với planner, controller, localizer
- Perception system với nhiều sensors và algorithms
- Multi-robot system với nhiều robot và coordinators

Việc làm việc trực tiếp với các thành phần này sẽ dẫn đến:

- Code phức tạp và khó bảo trì
- Coupling chặt giữa các thành phần
- Khó thay đổi implementation
- Learning curve cao cho người mới

## 3. Giải pháp

Facade Pattern giải quyết vấn đề bằng cách:

1. Tạo một interface đơn giản cho hệ thống phức tạp
2. Che giấu chi tiết implementation
3. Giảm coupling giữa client và subsystems
4. Cung cấp entry point duy nhất cho hệ thống

## 4. Ví dụ thực tế: Robot Arm Control

<style>
.code-block {
    white-space: pre-wrap;       /* CSS 3 */
    white-space: -moz-pre-wrap;  /* Mozilla */
    white-space: -pre-wrap;      /* Opera 4-6 */
    white-space: -o-pre-wrap;    /* Opera 7 */
    word-wrap: break-word;       /* Internet Explorer 5.5+ */
}
</style>

```cpp
// Subsystem classes
class JointController {
public:
    void setPosition(int joint_id, double position) {
        RCLCPP_INFO(logger_, "Setting joint %d to position %f", joint_id, position);
        // Implementation
    }

    void setVelocity(int joint_id, double velocity) {
        RCLCPP_INFO(logger_, "Setting joint %d velocity to %f", joint_id, velocity);
        // Implementation
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("JointController");
};

class GripperController {
public:
    void open(double width) {
        RCLCPP_INFO(logger_, "Opening gripper to width %f", width);
        // Implementation
    }

    void close(double force) {
        RCLCPP_INFO(logger_, "Closing gripper with force %f", force);
        // Implementation
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("GripperController");
};

class CollisionChecker {
public:
    bool checkCollision(const std::vector<double>& joint_positions) {
        RCLCPP_INFO(logger_, "Checking collision for joint positions");
        // Implementation
        return false;
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("CollisionChecker");
};

class TrajectoryPlanner {
public:
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start,
        const std::vector<double>& goal
    ) {
        RCLCPP_INFO(logger_, "Planning path from start to goal");
        // Implementation
        return {start, goal}; // Simplified
    }

private:
    rclcpp::Logger logger_ = rclcpp::get_logger("TrajectoryPlanner");
};

// Facade
class RobotArmFacade {
public:
    RobotArmFacade()
        : node_(std::make_shared<rclcpp::Node>("robot_arm_facade")) 
    {
        // Initialize subsystems
        joint_controller_ = std::make_unique<JointController>();
        gripper_controller_ = std::make_unique<GripperController>();
        collision_checker_ = std::make_unique<CollisionChecker>();
        trajectory_planner_ = std::make_unique<TrajectoryPlanner>();
    }

    // High-level methods
    bool pickObject(const geometry_msgs::msg::Pose& object_pose) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Picking object at pose: x=%f, y=%f, z=%f",
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z
        );

        try {
            // 1. Plan path to pre-grasp pose
            auto pre_grasp_joints = computeInverseKinematics(object_pose);
            if (pre_grasp_joints.empty()) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Failed to compute IK for pre-grasp pose"
                );
                return false;
            }

            // 2. Check collision
            if (collision_checker_->checkCollision(pre_grasp_joints)) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Path to object is blocked"
                );
                return false;
            }

            // 3. Plan and execute trajectory
            auto path = trajectory_planner_->planPath(
                getCurrentJointPositions(),
                pre_grasp_joints
            );
            executeTrajectory(path);

            // 4. Open gripper
            gripper_controller_->open(0.08); // 8cm opening

            // 5. Move to grasp pose
            auto grasp_joints = computeInverseKinematics(object_pose);
            executeTrajectory({grasp_joints});

            // 6. Close gripper
            gripper_controller_->close(10.0); // 10N force

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Pick operation failed: %s",
                e.what()
            );
            return false;
        }
    }

    bool placeObject(const geometry_msgs::msg::Pose& place_pose) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Placing object at pose: x=%f, y=%f, z=%f",
            place_pose.position.x,
            place_pose.position.y,
            place_pose.position.z
        );

        try {
            // 1. Plan path to place pose
            auto place_joints = computeInverseKinematics(place_pose);
            if (place_joints.empty()) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Failed to compute IK for place pose"
                );
                return false;
            }

            // 2. Check collision
            if (collision_checker_->checkCollision(place_joints)) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Path to place pose is blocked"
                );
                return false;
            }

            // 3. Plan and execute trajectory
            auto path = trajectory_planner_->planPath(
                getCurrentJointPositions(),
                place_joints
            );
            executeTrajectory(path);

            // 4. Open gripper to release object
            gripper_controller_->open(0.08);

            // 5. Move back to home position
            moveToHome();

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Place operation failed: %s",
                e.what()
            );
            return false;
        }
    }

    void moveToHome() {
        RCLCPP_INFO(node_->get_logger(), "Moving to home position");
        std::vector<double> home_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
        auto path = trajectory_planner_->planPath(
            getCurrentJointPositions(),
            home_position
        );
        executeTrajectory(path);
    }

private:
    std::vector<double> computeInverseKinematics(const geometry_msgs::msg::Pose& pose) {
        // Implementation of inverse kinematics
        return std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Simplified
    }

    std::vector<double> getCurrentJointPositions() {
        // Implementation to get current joint positions
        return std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Simplified
    }

    void executeTrajectory(const std::vector<std::vector<double>>& path) {
        for (const auto& point : path) {
            for (size_t i = 0; i < point.size(); ++i) {
                joint_controller_->setPosition(i, point[i]);
            }
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<JointController> joint_controller_;
    std::unique_ptr<GripperController> gripper_controller_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    std::unique_ptr<TrajectoryPlanner> trajectory_planner_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Facade Pattern trong một ROS2 node:

```cpp
class RobotArmNode : public rclcpp::Node {
public:
    RobotArmNode() : Node("robot_arm_node") {
        // Khởi tạo facade
        arm_facade_ = std::make_unique<RobotArmFacade>();

        // Tạo services
        pick_service_ = create_service<custom_msgs::srv::PickObject>(
            "pick_object",
            [this](const std::shared_ptr<custom_msgs::srv::PickObject::Request> request,
                   std::shared_ptr<custom_msgs::srv::PickObject::Response> response) {
                response->success = arm_facade_->pickObject(request->object_pose);
            });

        place_service_ = create_service<custom_msgs::srv::PlaceObject>(
            "place_object",
            [this](const std::shared_ptr<custom_msgs::srv::PlaceObject::Request> request,
                   std::shared_ptr<custom_msgs::srv::PlaceObject::Response> response) {
                response->success = arm_facade_->placeObject(request->place_pose);
            });

        home_service_ = create_service<std_srvs::srv::Trigger>(
            "move_to_home",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                arm_facade_->moveToHome();
                response->success = true;
            });
    }

private:
    std::unique_ptr<RobotArmFacade> arm_facade_;
    rclcpp::Service<custom_msgs::srv::PickObject>::SharedPtr pick_service_;
    rclcpp::Service<custom_msgs::srv::PlaceObject>::SharedPtr place_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
};
```

## 6. Lợi ích

1. **Đơn giản hóa**: Cung cấp interface đơn giản cho hệ thống phức tạp
2. **Loose coupling**: Giảm sự phụ thuộc giữa client và subsystems
3. **Encapsulation**: Che giấu chi tiết implementation
4. **Maintainability**: Dễ dàng thay đổi subsystems mà không ảnh hưởng tới client

## 7. Khi nào sử dụng

- Khi cần interface đơn giản cho hệ thống phức tạp
- Khi muốn tạo entry point cho các subsystem
- Khi cần giảm coupling giữa subsystems
- Khi muốn layer hóa hệ thống

## 8. Lưu ý

1. Thiết kế Facade:
   - Không nên làm Facade quá phức tạp
   - Chỉ expose những chức năng cần thiết
   - Giữ interface đơn giản và rõ ràng

2. Quản lý phụ thuộc:
   - Sử dụng dependency injection khi có thể
   - Tránh tight coupling với subsystems
   - Cân nhắc sử dụng interface cho subsystems

3. Trong ROS2:
   - Sử dụng ROS2 services/actions cho các thao tác dài
   - Xử lý lỗi và timeout phù hợp
   - Cung cấp feedback về trạng thái thao tác 