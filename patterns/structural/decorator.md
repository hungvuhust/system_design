# Decorator Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Decorator Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, cho phép thêm các tính năng mới cho đối tượng một cách linh hoạt mà không làm thay đổi cấu trúc của đối tượng gốc.

Trong ROS2 và robotics, Decorator Pattern thường được sử dụng để:
- Thêm các tính năng xử lý cho sensor data
- Mở rộng chức năng của robot controllers
- Tăng cường khả năng của navigation algorithms
- Thêm logging, monitoring cho các node

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần mở rộng chức năng của một component, ví dụ:
- Thêm bộ lọc nhiễu cho sensor data
- Thêm giới hạn tốc độ cho robot controller
- Thêm chức năng ghi log cho navigation
- Thêm validation cho các message

Việc thêm tính năng bằng cách kế thừa sẽ dẫn đến:
- Code cứng nhắc và khó mở rộng
- Số lượng class tăng nhanh
- Khó kết hợp nhiều tính năng

## 3. Giải pháp

Decorator Pattern giải quyết vấn đề bằng cách:
1. Tạo wrapper class implement cùng interface với component gốc
2. Wrapper class chứa reference đến component gốc
3. Wrapper class có thể thêm hành vi trước/sau khi gọi method của component gốc

## 4. Ví dụ thực tế: Robot Controller

```cpp
// Base interface
class RobotController {
public:
    virtual ~RobotController() = default;
    virtual geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) = 0;
};

// Concrete component
class DifferentialDriveController : public RobotController {
public:
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) override {
        
        geometry_msgs::msg::Twist cmd_vel;
        // Tính toán vận tốc cơ bản
        cmd_vel.linear.x = 0.5;  // m/s
        cmd_vel.angular.z = 0.2; // rad/s
        return cmd_vel;
    }
};

// Base decorator
class ControllerDecorator : public RobotController {
protected:
    std::shared_ptr<RobotController> controller_;

public:
    ControllerDecorator(std::shared_ptr<RobotController> controller)
        : controller_(controller) {}

    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) override {
        return controller_->computeVelocity(current, target);
    }
};

// Concrete decorator: Speed Limiter
class SpeedLimiterDecorator : public ControllerDecorator {
private:
    double max_linear_speed_;
    double max_angular_speed_;

public:
    SpeedLimiterDecorator(
        std::shared_ptr<RobotController> controller,
        double max_linear = 1.0,
        double max_angular = 0.5)
        : ControllerDecorator(controller)
        , max_linear_speed_(max_linear)
        , max_angular_speed_(max_angular) {}

    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) override {
        
        auto cmd_vel = controller_->computeVelocity(current, target);
        
        // Giới hạn tốc độ
        cmd_vel.linear.x = std::min(std::abs(cmd_vel.linear.x), max_linear_speed_) *
                          (cmd_vel.linear.x >= 0 ? 1 : -1);
        cmd_vel.angular.z = std::min(std::abs(cmd_vel.angular.z), max_angular_speed_) *
                           (cmd_vel.angular.z >= 0 ? 1 : -1);
        
        return cmd_vel;
    }
};

// Concrete decorator: Obstacle Avoider
class ObstacleAvoiderDecorator : public ControllerDecorator {
private:
    std::shared_ptr<sensor_msgs::msg::LaserScan> latest_scan_;
    double safe_distance_;

public:
    ObstacleAvoiderDecorator(
        std::shared_ptr<RobotController> controller,
        double safe_distance = 0.5)
        : ControllerDecorator(controller)
        , safe_distance_(safe_distance) {}

    void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        latest_scan_ = scan;
    }

    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) override {
        
        auto cmd_vel = controller_->computeVelocity(current, target);
        
        if (latest_scan_) {
            // Kiểm tra khoảng cách an toàn
            bool obstacle_detected = false;
            for (const auto& range : latest_scan_->ranges) {
                if (range < safe_distance_) {
                    obstacle_detected = true;
                    break;
                }
            }
            
            // Dừng robot nếu phát hiện chướng ngại vật
            if (obstacle_detected) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
        }
        
        return cmd_vel;
    }
};

// Concrete decorator: Logger
class LoggerDecorator : public ControllerDecorator {
private:
    rclcpp::Logger logger_;

public:
    LoggerDecorator(
        std::shared_ptr<RobotController> controller,
        const rclcpp::Logger& logger)
        : ControllerDecorator(controller)
        , logger_(logger) {}

    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::Pose& current,
        const geometry_msgs::msg::Pose& target) override {
        
        RCLCPP_INFO(logger_, "Computing velocity command");
        RCLCPP_DEBUG(logger_, "Current pose: x=%.2f, y=%.2f",
                    current.position.x, current.position.y);
        RCLCPP_DEBUG(logger_, "Target pose: x=%.2f, y=%.2f",
                    target.position.x, target.position.y);
        
        auto start_time = std::chrono::steady_clock::now();
        auto cmd_vel = controller_->computeVelocity(current, target);
        auto end_time = std::chrono::steady_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        
        RCLCPP_INFO(logger_, "Velocity computed in %ld ms: linear=%.2f, angular=%.2f",
                   duration, cmd_vel.linear.x, cmd_vel.angular.z);
        
        return cmd_vel;
    }
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Decorator Pattern trong một ROS2 node:

```cpp
class DecoratedControllerNode : public rclcpp::Node {
public:
    DecoratedControllerNode()
        : Node("decorated_controller") {
        
        // Khởi tạo base controller
        auto base_controller = std::make_shared<DifferentialDriveController>();
        
        // Thêm các decorator
        auto speed_limited = std::make_shared<SpeedLimiterDecorator>(
            base_controller, 0.8, 0.4);
        
        auto obstacle_aware = std::make_shared<ObstacleAvoiderDecorator>(
            speed_limited, 0.3);
        
        controller_ = std::make_shared<LoggerDecorator>(
            obstacle_aware, get_logger());
        
        // Subscribers
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10,
            std::bind(&DecoratedControllerNode::poseCallback, this, std::placeholders::_1));
        
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&DecoratedControllerNode::targetCallback, this, std::placeholders::_1));
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this, obstacle_aware](const sensor_msgs::msg::LaserScan::SharedPtr scan) {
                obstacle_aware->updateScan(scan);
            });
        
        // Publisher
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
        publishVelocity();
    }
    
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_pose_ = msg->pose;
        publishVelocity();
    }
    
    void publishVelocity() {
        if (current_pose_ && target_pose_) {
            auto cmd_vel = controller_->computeVelocity(*current_pose_, *target_pose_);
            cmd_vel_pub_->publish(cmd_vel);
        }
    }

    std::shared_ptr<RobotController> controller_;
    std::optional<geometry_msgs::msg::Pose> current_pose_;
    std::optional<geometry_msgs::msg::Pose> target_pose_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};
```

## 6. Lợi ích

1. **Tính linh hoạt**: Dễ dàng thêm/xóa tính năng trong runtime
2. **Single Responsibility**: Mỗi decorator chỉ xử lý một chức năng
3. **Open/Closed**: Mở rộng chức năng mà không sửa code hiện có
4. **Tái sử dụng**: Có thể kết hợp các decorator theo nhiều cách khác nhau

## 7. Khi nào sử dụng

- Khi cần thêm tính năng cho đối tượng mà không muốn thay đổi code gốc
- Khi muốn tính năng có thể được thêm/xóa trong runtime
- Khi kế thừa không phải là giải pháp tốt
- Khi cần kết hợp nhiều tính năng một cách linh hoạt

## 8. Lưu ý

1. Thứ tự decorator có thể ảnh hưởng đến kết quả:
   - SpeedLimiter -> ObstacleAvoider khác với ObstacleAvoider -> SpeedLimiter
   - Cần cân nhắc thứ tự hợp lý dựa trên logic nghiệp vụ

2. Quản lý phụ thuộc:
   - Sử dụng dependency injection để inject các decorator
   - Tránh hardcode thứ tự decorator

3. Trong ROS2:
   - Sử dụng shared_ptr để quản lý memory
   - Cẩn thận với thread safety khi decorator truy cập shared resources
   - Xử lý ngoại lệ phù hợp để tránh crash node
