# Visitor Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Visitor Pattern là một behavioral pattern cho phép thêm các operations mới vào một object structure mà không cần thay đổi các classes của structure đó. Trong ROS2 và robotics, pattern này đặc biệt hữu ích cho:

- Sensor data processing
- Robot diagnostics
- Configuration validation
- Data collection và analysis
- Robot behavior customization
- Plugin-based functionality

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống:
- Xử lý nhiều loại sensor data khác nhau
- Thực hiện các operations khác nhau trên robot components
- Thu thập metrics và diagnostics
- Validate configurations
- Customize robot behaviors
- Extend functionality without modify code

## 3. Giải pháp

Visitor Pattern giải quyết các vấn đề trên bằng cách:
1. Tách operations khỏi object structure
2. Support double dispatch
3. Enable easy extension
4. Maintain clean separation of concerns

## 4. Ví dụ thực tế: Robot Diagnostics System

```cpp
// Forward declarations
class JointComponent;
class SensorComponent;
class BatteryComponent;
class MotorComponent;

// Visitor interface
class ComponentVisitor {
public:
    virtual ~ComponentVisitor() = default;
    virtual void visitJoint(JointComponent* joint) = 0;
    virtual void visitSensor(SensorComponent* sensor) = 0;
    virtual void visitBattery(BatteryComponent* battery) = 0;
    virtual void visitMotor(MotorComponent* motor) = 0;
};

// Component interface
class RobotComponent {
public:
    virtual ~RobotComponent() = default;
    virtual void accept(ComponentVisitor* visitor) = 0;
    virtual std::string getName() const = 0;
};

// Concrete components
class JointComponent : public RobotComponent {
public:
    explicit JointComponent(const std::string& name)
        : name_(name), position_(0.0), velocity_(0.0), effort_(0.0) {}

    void accept(ComponentVisitor* visitor) override {
        visitor->visitJoint(this);
    }

    std::string getName() const override { return name_; }

    // Joint-specific data
    double getPosition() const { return position_; }
    double getVelocity() const { return velocity_; }
    double getEffort() const { return effort_; }
    double getTemperature() const { return temperature_; }

private:
    std::string name_;
    double position_;
    double velocity_;
    double effort_;
    double temperature_{25.0};
};

class SensorComponent : public RobotComponent {
public:
    explicit SensorComponent(const std::string& name)
        : name_(name), is_calibrated_(false) {}

    void accept(ComponentVisitor* visitor) override {
        visitor->visitSensor(this);
    }

    std::string getName() const override { return name_; }

    // Sensor-specific data
    bool isCalibrated() const { return is_calibrated_; }
    double getUpdateRate() const { return update_rate_; }
    std::string getStatus() const { return status_; }

private:
    std::string name_;
    bool is_calibrated_;
    double update_rate_{30.0};
    std::string status_{"OK"};
};

class BatteryComponent : public RobotComponent {
public:
    explicit BatteryComponent(const std::string& name)
        : name_(name), charge_level_(1.0) {}

    void accept(ComponentVisitor* visitor) override {
        visitor->visitBattery(this);
    }

    std::string getName() const override { return name_; }

    // Battery-specific data
    double getChargeLevel() const { return charge_level_; }
    double getVoltage() const { return voltage_; }
    double getCurrent() const { return current_; }
    double getTemperature() const { return temperature_; }

private:
    std::string name_;
    double charge_level_;
    double voltage_{12.0};
    double current_{1.0};
    double temperature_{30.0};
};

class MotorComponent : public RobotComponent {
public:
    explicit MotorComponent(const std::string& name)
        : name_(name), speed_(0.0) {}

    void accept(ComponentVisitor* visitor) override {
        visitor->visitMotor(this);
    }

    std::string getName() const override { return name_; }

    // Motor-specific data
    double getSpeed() const { return speed_; }
    double getTorque() const { return torque_; }
    double getTemperature() const { return temperature_; }

private:
    std::string name_;
    double speed_;
    double torque_{0.0};
    double temperature_{40.0};
};

// Concrete visitors
class DiagnosticsVisitor : public ComponentVisitor {
public:
    explicit DiagnosticsVisitor(rclcpp::Logger logger)
        : logger_(logger) {}

    void visitJoint(JointComponent* joint) override {
        RCLCPP_INFO(logger_, "Joint [%s] Diagnostics:", joint->getName().c_str());
        RCLCPP_INFO(logger_, "  Position: %.2f", joint->getPosition());
        RCLCPP_INFO(logger_, "  Velocity: %.2f", joint->getVelocity());
        RCLCPP_INFO(logger_, "  Temperature: %.2f°C", joint->getTemperature());

        if (joint->getTemperature() > 50.0) {
            RCLCPP_WARN(logger_, "  High temperature warning!");
        }
    }

    void visitSensor(SensorComponent* sensor) override {
        RCLCPP_INFO(logger_, "Sensor [%s] Diagnostics:", sensor->getName().c_str());
        RCLCPP_INFO(logger_, "  Calibrated: %s",
            sensor->isCalibrated() ? "Yes" : "No");
        RCLCPP_INFO(logger_, "  Update Rate: %.2f Hz", sensor->getUpdateRate());
        RCLCPP_INFO(logger_, "  Status: %s", sensor->getStatus().c_str());

        if (!sensor->isCalibrated()) {
            RCLCPP_WARN(logger_, "  Sensor needs calibration!");
        }
    }

    void visitBattery(BatteryComponent* battery) override {
        RCLCPP_INFO(logger_, "Battery [%s] Diagnostics:", battery->getName().c_str());
        RCLCPP_INFO(logger_, "  Charge Level: %.2f%%",
            battery->getChargeLevel() * 100.0);
        RCLCPP_INFO(logger_, "  Voltage: %.2fV", battery->getVoltage());
        RCLCPP_INFO(logger_, "  Current: %.2fA", battery->getCurrent());
        RCLCPP_INFO(logger_, "  Temperature: %.2f°C", battery->getTemperature());

        if (battery->getChargeLevel() < 0.2) {
            RCLCPP_WARN(logger_, "  Low battery warning!");
        }
    }

    void visitMotor(MotorComponent* motor) override {
        RCLCPP_INFO(logger_, "Motor [%s] Diagnostics:", motor->getName().c_str());
        RCLCPP_INFO(logger_, "  Speed: %.2f RPM", motor->getSpeed());
        RCLCPP_INFO(logger_, "  Torque: %.2f Nm", motor->getTorque());
        RCLCPP_INFO(logger_, "  Temperature: %.2f°C", motor->getTemperature());

        if (motor->getTemperature() > 60.0) {
            RCLCPP_WARN(logger_, "  High temperature warning!");
        }
    }

private:
    rclcpp::Logger logger_;
};

class MetricsVisitor : public ComponentVisitor {
public:
    explicit MetricsVisitor(std::shared_ptr<rclcpp::Node> node)
        : node_(node) {
        // Initialize publishers
        metrics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "robot_metrics", 10);
    }

    void visitJoint(JointComponent* joint) override {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "joint_" + joint->getName();
        status.hardware_id = joint->getName();
        
        addKeyValue(status, "position", joint->getPosition());
        addKeyValue(status, "velocity", joint->getVelocity());
        addKeyValue(status, "temperature", joint->getTemperature());
        
        publishMetrics(status);
    }

    void visitSensor(SensorComponent* sensor) override {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "sensor_" + sensor->getName();
        status.hardware_id = sensor->getName();
        
        addKeyValue(status, "calibrated", sensor->isCalibrated());
        addKeyValue(status, "update_rate", sensor->getUpdateRate());
        addKeyValue(status, "status", sensor->getStatus());
        
        publishMetrics(status);
    }

    void visitBattery(BatteryComponent* battery) override {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "battery_" + battery->getName();
        status.hardware_id = battery->getName();
        
        addKeyValue(status, "charge_level", battery->getChargeLevel() * 100.0);
        addKeyValue(status, "voltage", battery->getVoltage());
        addKeyValue(status, "current", battery->getCurrent());
        addKeyValue(status, "temperature", battery->getTemperature());
        
        publishMetrics(status);
    }

    void visitMotor(MotorComponent* motor) override {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "motor_" + motor->getName();
        status.hardware_id = motor->getName();
        
        addKeyValue(status, "speed", motor->getSpeed());
        addKeyValue(status, "torque", motor->getTorque());
        addKeyValue(status, "temperature", motor->getTemperature());
        
        publishMetrics(status);
    }

private:
    void addKeyValue(diagnostic_msgs::msg::DiagnosticStatus& status,
                    const std::string& key, double value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = std::to_string(value);
        status.values.push_back(kv);
    }

    void addKeyValue(diagnostic_msgs::msg::DiagnosticStatus& status,
                    const std::string& key, bool value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value ? "true" : "false";
        status.values.push_back(kv);
    }

    void addKeyValue(diagnostic_msgs::msg::DiagnosticStatus& status,
                    const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    }

    void publishMetrics(const diagnostic_msgs::msg::DiagnosticStatus& status) {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = node_->now();
        array.status.push_back(status);
        metrics_pub_->publish(array);
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr metrics_pub_;
};

```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Visitor Pattern trong một ROS2 node:

```cpp
class RobotDiagnosticsNode : public rclcpp::Node {
public:
    RobotDiagnosticsNode()
        : Node("robot_diagnostics_node") {
        // Initialize components
        components_.push_back(std::make_unique<JointComponent>("shoulder"));
        components_.push_back(std::make_unique<JointComponent>("elbow"));
        components_.push_back(std::make_unique<SensorComponent>("lidar"));
        components_.push_back(std::make_unique<BatteryComponent>("main_battery"));
        components_.push_back(std::make_unique<MotorComponent>("base_motor"));

        // Initialize visitors
        diagnostics_visitor_ = std::make_unique<DiagnosticsVisitor>(get_logger());
        metrics_visitor_ = std::make_unique<MetricsVisitor>(shared_from_this());

        // Create timer for periodic diagnostics
        diagnostics_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotDiagnosticsNode::runDiagnostics, this));

        // Create timer for periodic metrics
        metrics_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotDiagnosticsNode::publishMetrics, this));

        RCLCPP_INFO(get_logger(), "Robot diagnostics node initialized");
    }

private:
    void runDiagnostics() {
        RCLCPP_INFO(get_logger(), "Running diagnostics...");
        for (const auto& component : components_) {
            component->accept(diagnostics_visitor_.get());
        }
    }

    void publishMetrics() {
        for (const auto& component : components_) {
            component->accept(metrics_visitor_.get());
        }
    }

    std::vector<std::unique_ptr<RobotComponent>> components_;
    std::unique_ptr<DiagnosticsVisitor> diagnostics_visitor_;
    std::unique_ptr<MetricsVisitor> metrics_visitor_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDiagnosticsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## 6. Lợi ích

1. **Separation of Concerns**:
   - Tách operations khỏi data structures
   - Dễ dàng thêm operations mới
   - Clean code organization

2. **Extensibility**:
   - Thêm visitors mới không cần sửa components
   - Support plugin architecture
   - Flexible functionality

3. **Type Safety**:
   - Strong type checking
   - Clear interface
   - Compile-time safety

## 7. Khi nào sử dụng

- Complex object structures
- Multiple operations trên objects
- Operation types có thể mở rộng
- Clean separation of concerns
- Plugin-based architecture
- Diagnostic và monitoring systems

## 8. Lưu ý

1. Thiết kế:
   - Visitor interface design
   - Component hierarchy
   - Operation granularity
   - Extension points

2. Implementation:
   - Double dispatch overhead
   - Type safety
   - Memory management
   - Performance considerations

3. Trong ROS2:
   - Message design
   - Service integration
   - Component lifecycle
   - Resource management 