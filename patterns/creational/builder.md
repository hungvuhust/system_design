# Builder Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Builder Pattern là một design pattern thuộc nhóm Creational Pattern, giúp xây dựng các đối tượng phức tạp theo từng bước. Pattern này đặc biệt hữu ích khi cần tạo ra một đối tượng với nhiều tham số cấu hình và các thành phần con.

Trong robotics và ROS2, Builder Pattern thường được sử dụng để:
- Khởi tạo robot với nhiều cấu hình khác nhau
- Xây dựng các message phức tạp
- Tạo các mission plan cho robot

## 2. Vấn đề

Giả sử chúng ta cần xây dựng một robot delivery với nhiều cấu hình khác nhau:
- Loại động cơ (bánh, xích, chân)
- Hệ thống cảm biến (camera, lidar, IMU)
- Hệ thống điều khiển (autonomous, semi-autonomous, manual)
- Dung lượng pin
- Kích thước khung gầm

Việc sử dụng constructor với nhiều tham số sẽ dẫn đến:
- Code khó đọc và dễ nhầm lẫn thứ tự tham số
- Khó mở rộng khi thêm tính năng mới
- Khó tái sử dụng code cho các cấu hình khác nhau

## 3. Giải pháp

Builder Pattern giải quyết vấn đề bằng cách:
1. Tách quá trình xây dựng đối tượng thành nhiều bước nhỏ
2. Cho phép tạo các cấu hình khác nhau của cùng một đối tượng
3. Che giấu chi tiết cài đặt phức tạp

## 4. Ví dụ thực tế

Dưới đây là ví dụ về việc sử dụng Builder Pattern để xây dựng một robot delivery:

```cpp
// Robot class - Product
class DeliveryRobot {
public:
    void setLocomotion(const std::string& type) { locomotion_type_ = type; }
    void setSensors(const std::vector<std::string>& sensors) { sensors_ = sensors; }
    void setControlSystem(const std::string& system) { control_system_ = system; }
    void setBatteryCapacity(double capacity) { battery_capacity_ = capacity; }
    void setDimensions(double length, double width, double height) {
        dimensions_ = {length, width, height};
    }

private:
    std::string locomotion_type_;
    std::vector<std::string> sensors_;
    std::string control_system_;
    double battery_capacity_;
    std::vector<double> dimensions_;
};

// Abstract Builder
class DeliveryRobotBuilder {
public:
    virtual void buildLocomotion() = 0;
    virtual void buildSensors() = 0;
    virtual void buildControlSystem() = 0;
    virtual void buildBattery() = 0;
    virtual void buildDimensions() = 0;
    virtual DeliveryRobot* getResult() = 0;
};

// Concrete Builder cho Indoor Delivery Robot
class IndoorDeliveryRobotBuilder : public DeliveryRobotBuilder {
public:
    IndoorDeliveryRobotBuilder() {
        robot_ = new DeliveryRobot();
    }

    void buildLocomotion() override {
        robot_->setLocomotion("differential_drive");
    }

    void buildSensors() override {
        robot_->setSensors({"rgb_camera", "depth_camera", "imu"});
    }

    void buildControlSystem() override {
        robot_->setControlSystem("autonomous");
    }

    void buildBattery() override {
        robot_->setBatteryCapacity(24.0); // 24V
    }

    void buildDimensions() override {
        robot_->setDimensions(0.8, 0.6, 1.2); // meters
    }

    DeliveryRobot* getResult() override {
        return robot_;
    }

private:
    DeliveryRobot* robot_;
};

// Director class
class RobotEngineer {
public:
    void setBuilder(DeliveryRobotBuilder* builder) {
        builder_ = builder;
    }

    DeliveryRobot* constructRobot() {
        builder_->buildLocomotion();
        builder_->buildSensors();
        builder_->buildControlSystem();
        builder_->buildBattery();
        builder_->buildDimensions();
        return builder_->getResult();
    }

private:
    DeliveryRobotBuilder* builder_;
};
```

## 5. Sử dụng trong ROS2

Trong ROS2, chúng ta có thể sử dụng Builder Pattern để khởi tạo node và các thành phần:

```cpp
// ROS2 Node Builder
class DeliveryRobotNodeBuilder {
public:
    DeliveryRobotNodeBuilder(const std::string& node_name)
        : node_(std::make_shared<rclcpp::Node>(node_name)) {}

    DeliveryRobotNodeBuilder& addPublisher(
        const std::string& topic,
        const std::string& msg_type,
        const rclcpp::QoS& qos) {
        // Thêm publisher
        return *this;
    }

    DeliveryRobotNodeBuilder& addSubscriber(
        const std::string& topic,
        const std::string& msg_type,
        const rclcpp::QoS& qos) {
        // Thêm subscriber
        return *this;
    }

    DeliveryRobotNodeBuilder& addParameter(
        const std::string& name,
        const rclcpp::ParameterValue& default_value) {
        node_->declare_parameter(name, default_value);
        return *this;
    }

    std::shared_ptr<rclcpp::Node> build() {
        return node_;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
};

// Sử dụng
auto node = DeliveryRobotNodeBuilder("delivery_robot")
    .addPublisher("/cmd_vel", "geometry_msgs/msg/Twist", 10)
    .addSubscriber("/scan", "sensor_msgs/msg/LaserScan", 10)
    .addParameter("max_speed", 1.0)
    .addParameter("min_obstacle_distance", 0.5)
    .build();
```

## 6. Lợi ích

1. **Tính linh hoạt**: Dễ dàng tạo các biến thể khác nhau của robot
2. **Code sạch**: Tránh constructor với nhiều tham số
3. **Tái sử dụng**: Có thể tái sử dụng code cho các loại robot khác nhau
4. **Dễ mở rộng**: Thêm tính năng mới không ảnh hưởng code hiện tại

## 7. Khi nào sử dụng

- Khi cần tạo đối tượng phức tạp với nhiều tham số
- Khi muốn tạo nhiều biến thể của cùng một đối tượng
- Khi cần kiểm soát quá trình khởi tạo đối tượng
- Khi muốn code dễ đọc và bảo trì

## 8. Lưu ý

1. Không nên sử dụng Builder Pattern cho các đối tượng đơn giản
2. Cần cân nhắc giữa tính linh hoạt và độ phức tạp của code
3. Trong ROS2, nên kết hợp với các pattern khác như Factory hoặc Singleton khi cần thiết
