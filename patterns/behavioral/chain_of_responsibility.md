# CHAIN OF RESPONSIBILITY PATTERN 

## 1. Giới thiệu

Chain of Responsibility là một behavioral pattern cho phép chuyển yêu cầu dọc theo một chuỗi các handlers. Khi nhận yêu cầu, mỗi handler quyết định xử lý nó hoặc chuyển tiếp cho handler tiếp theo. Trong ROS2 và robotics, pattern này thường được sử dụng cho:

- Xử lý sensor data
- Robot behavior selection
- Error handling
- Command validation
- Task scheduling
- Safety checks

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:

- Xử lý dữ liệu theo nhiều bước
- Kiểm tra nhiều điều kiện an toàn
- Lựa chọn behavior phù hợp
- Validate commands theo nhiều tiêu chí
- Handle errors ở nhiều levels

## 3. Giải pháp

Chain of Responsibility giải quyết các vấn đề trên bằng cách:

1. Tách logic xử lý thành các handlers riêng biệt
2. Sắp xếp handlers thành một chuỗi
3. Cho phép handlers quyết định xử lý hoặc chuyển tiếp
4. Dễ dàng thêm/bớt/sắp xếp lại handlers

## 4. Ví dụ thực tế: Robot Command Processing System

```cpp
// Abstract base class cho command handlers
class CommandHandler {
public:
    virtual ~CommandHandler() = default;

    void setNext(std::shared_ptr<CommandHandler> next) {
        next_ = next;
    }

    // Template method để xử lý command
    virtual bool handle(const geometry_msgs::msg::Twist& cmd) {
        if (canHandle(cmd)) {
            return handleCommand(cmd);
        } else if (next_) {
            return next_->handle(cmd);
        }
        return false;
    }

protected:
    virtual bool canHandle(const geometry_msgs::msg::Twist& cmd) = 0;
    virtual bool handleCommand(const geometry_msgs::msg::Twist& cmd) = 0;
    std::shared_ptr<CommandHandler> next_;
};

// Safety check handler
class SafetyHandler : public CommandHandler {
public:
    explicit SafetyHandler(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , laser_sub_(node->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&SafetyHandler::laserCallback, this, std::placeholders::_1)))
    {
        min_distance_ = node->declare_parameter("safety.min_distance", 0.5);
    }

protected:
    bool canHandle(const geometry_msgs::msg::Twist& cmd) override {
        return true;  // Safety handler should always check
    }

    bool handleCommand(const geometry_msgs::msg::Twist& cmd) override {
        if (!isEnvironmentSafe()) {
            RCLCPP_WARN(node_->get_logger(), "Environment not safe for movement");
            return false;
        }

        if (!isCommandSafe(cmd)) {
            RCLCPP_WARN(node_->get_logger(), "Command exceeds safety limits");
            return false;
        }

        return next_ ? next_->handle(cmd) : true;
    }

private:
    bool isEnvironmentSafe() {
        std::lock_guard<std::mutex> lock(laser_mutex_);
        return min_laser_distance_ > min_distance_;
    }

    bool isCommandSafe(const geometry_msgs::msg::Twist& cmd) {
        // Check velocity limits
        const double MAX_LINEAR_VEL = 1.0;
        const double MAX_ANGULAR_VEL = 1.5;

        return std::abs(cmd.linear.x) <= MAX_LINEAR_VEL &&
               std::abs(cmd.angular.z) <= MAX_ANGULAR_VEL;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(laser_mutex_);
        min_laser_distance_ = *std::min_element(
            msg->ranges.begin(), msg->ranges.end());
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::mutex laser_mutex_;
    double min_laser_distance_{std::numeric_limits<double>::max()};
    double min_distance_;
};

// Battery level handler
class BatteryHandler : public CommandHandler {
public:
    explicit BatteryHandler(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , battery_sub_(node->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&BatteryHandler::batteryCallback, this, std::placeholders::_1)))
    {
        min_battery_ = node->declare_parameter("battery.min_level", 0.15);
    }

protected:
    bool canHandle(const geometry_msgs::msg::Twist& cmd) override {
        return true;  // Always check battery
    }

    bool handleCommand(const geometry_msgs::msg::Twist& cmd) override {
        if (!isBatteryOk()) {
            RCLCPP_WARN(node_->get_logger(), "Battery level too low for movement");
            return false;
        }

        return next_ ? next_->handle(cmd) : true;
    }

private:
    bool isBatteryOk() {
        std::lock_guard<std::mutex> lock(battery_mutex_);
        return battery_level_ > min_battery_;
    }

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(battery_mutex_);
        battery_level_ = msg->percentage;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    std::mutex battery_mutex_;
    double battery_level_{1.0};
    double min_battery_;
};

// Obstacle avoidance handler
class ObstacleHandler : public CommandHandler {
public:
    explicit ObstacleHandler(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , costmap_sub_(node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap", 10,
            std::bind(&ObstacleHandler::costmapCallback, this, std::placeholders::_1)))
    {
        robot_radius_ = node->declare_parameter("robot.radius", 0.3);
    }

protected:
    bool canHandle(const geometry_msgs::msg::Twist& cmd) override {
        // Only check for non-zero velocities
        return std::abs(cmd.linear.x) > 0.01 || 
               std::abs(cmd.linear.y) > 0.01 || 
               std::abs(cmd.angular.z) > 0.01;
    }

    bool handleCommand(const geometry_msgs::msg::Twist& cmd) override {
        if (!isPathClear(cmd)) {
            RCLCPP_WARN(node_->get_logger(), "Obstacle detected in movement direction");
            return false;
        }

        return next_ ? next_->handle(cmd) : true;
    }

private:
    bool isPathClear(const geometry_msgs::msg::Twist& cmd) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        
        // Simple collision checking in movement direction
        double prediction_time = 1.0;  // Look ahead 1 second
        double pred_x = cmd.linear.x * prediction_time;
        double pred_y = cmd.linear.y * prediction_time;
        
        // Convert to costmap cells
        int cell_x = static_cast<int>((pred_x + costmap_origin_x_) / costmap_resolution_);
        int cell_y = static_cast<int>((pred_y + costmap_origin_y_) / costmap_resolution_);
        
        // Check cells in robot radius
        int radius_cells = static_cast<int>(robot_radius_ / costmap_resolution_);
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                int check_x = cell_x + dx;
                int check_y = cell_y + dy;
                
                if (check_x >= 0 && check_x < costmap_width_ &&
                    check_y >= 0 && check_y < costmap_height_) {
                    if (costmap_[check_y * costmap_width_ + check_x] > 50) {
                        return false;  // Obstacle detected
                    }
                }
            }
        }
        
        return true;
    }

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_ = msg->data;
        costmap_width_ = msg->info.width;
        costmap_height_ = msg->info.height;
        costmap_resolution_ = msg->info.resolution;
        costmap_origin_x_ = msg->info.origin.position.x;
        costmap_origin_y_ = msg->info.origin.position.y;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    std::mutex costmap_mutex_;
    std::vector<int8_t> costmap_;
    int costmap_width_{0};
    int costmap_height_{0};
    double costmap_resolution_{0.05};
    double costmap_origin_x_{0.0};
    double costmap_origin_y_{0.0};
    double robot_radius_;
};

// Motor temperature handler
class TemperatureHandler : public CommandHandler {
public:
    explicit TemperatureHandler(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , temp_sub_(node->create_subscription<sensor_msgs::msg::Temperature>(
            "motor_temperature", 10,
            std::bind(&TemperatureHandler::temperatureCallback, this, std::placeholders::_1)))
    {
        max_temp_ = node->declare_parameter("motor.max_temperature", 60.0);
    }

protected:
    bool canHandle(const geometry_msgs::msg::Twist& cmd) override {
        return std::abs(cmd.linear.x) > 0.1 || std::abs(cmd.angular.z) > 0.1;
    }

    bool handleCommand(const geometry_msgs::msg::Twist& cmd) override {
        if (!isTemperatureOk()) {
            RCLCPP_WARN(node_->get_logger(), "Motor temperature too high");
            return false;
        }

        return next_ ? next_->handle(cmd) : true;
    }

private:
    bool isTemperatureOk() {
        std::lock_guard<std::mutex> lock(temp_mutex_);
        return current_temp_ <= max_temp_;
    }

    void temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(temp_mutex_);
        current_temp_ = msg->temperature;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
    std::mutex temp_mutex_;
    double current_temp_{20.0};
    double max_temp_;
};

```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Chain of Responsibility trong một ROS2 node:

```cpp
class SafeCommandNode : public rclcpp::Node {
public:
    SafeCommandNode() : Node("safe_command_node") {
        // Khởi tạo chain of handlers
        auto safety_handler = std::make_shared<SafetyHandler>(shared_from_this());
        auto battery_handler = std::make_shared<BatteryHandler>(shared_from_this());
        auto obstacle_handler = std::make_shared<ObstacleHandler>(shared_from_this());
        auto temp_handler = std::make_shared<TemperatureHandler>(shared_from_this());

        // Thiết lập chain
        safety_handler->setNext(battery_handler);
        battery_handler->setNext(obstacle_handler);
        obstacle_handler->setNext(temp_handler);

        command_handler_ = safety_handler;

        // Subscribe to command topic
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_raw", 10,
            std::bind(&SafeCommandNode::commandCallback, this, std::placeholders::_1));

        // Publisher for safe commands
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (command_handler_->handle(*msg)) {
            // Command passed all checks
            cmd_pub_->publish(*msg);
        } else {
            // Command failed checks, publish zero velocity
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);
        }
    }

    std::shared_ptr<CommandHandler> command_handler_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
```

## 6. Lợi ích

1. **Decoupling**:
   - Tách biệt sender và receiver
   - Giảm phụ thuộc giữa các components
   - Dễ dàng thay đổi xử lý logic

2. **Flexibility**:
   - Thêm/bớt handlers dễ dàng
   - Thay đổi thứ tự xử lý
   - Dynamic chain configuration

3. **Single Responsibility**:
   - Mỗi handler tập trung vào một nhiệm vụ
   - Code dễ maintain
   - Dễ test và debug

## 7. Khi nào sử dụng

- Cần xử lý request theo nhiều cách khác nhau
- Thứ tự xử lý quan trọng
- Muốn tách biệt các bước xử lý
- Cần flexibility trong việc thêm/bớt handlers
- Có nhiều điều kiện check cần thực hiện

## 8. Lưu ý

1. Thiết kế:

   - Xác định rõ trách nhiệm của mỗi handler
   - Cân nhắc thứ tự xử lý
   - Handle edge cases
   - Tránh chains quá dài

2. Implementation:

   - Thread safety
   - Error handling
   - Performance considerations
   - Memory management

3. Trong ROS2:

   - Message passing overhead
   - Timing considerations
   - Resource management
   - Node lifecycle handling 