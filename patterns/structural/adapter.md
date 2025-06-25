# Adapter Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Adapter Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, cho phép các interface không tương thích có thể làm việc cùng nhau. Pattern này đặc biệt hữu ích trong robotics khi:
- Tích hợp các sensors/actuators khác nhau
- Chuyển đổi giữa các message types
- Kết nối các hệ thống legacy
- Tương thích với nhiều robot platforms
- Chuyển đổi dữ liệu giữa các formats

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống sau:
- Cần sử dụng sensors từ nhiều nhà sản xuất khác nhau
- Muốn tích hợp code cũ vào hệ thống ROS2 mới
- Cần chuyển đổi giữa các coordinate frames
- Muốn sử dụng lại code cho nhiều loại robot
- Cần tương thích với nhiều protocols khác nhau

## 3. Giải pháp

Adapter Pattern giải quyết các vấn đề trên bằng cách:
1. Định nghĩa interface mong muốn (target interface)
2. Tạo adapter class implement target interface
3. Adapter wrap adaptee (class cần được điều chỉnh)
4. Chuyển đổi các calls giữa target và adaptee

## 4. Ví dụ thực tế: Sensor Integration

```cpp
// Target interface - Chuẩn sensor interface cho hệ thống
class IDistanceSensor {
public:
    virtual ~IDistanceSensor() = default;
    virtual double getDistanceMeters() = 0;
    virtual bool isInRange() = 0;
    virtual std::string getSensorInfo() = 0;
};

// Legacy sensor class (Adaptee) - giả sử đây là driver cũ
class LegacyInfraredSensor {
public:
    // Returns distance in inches
    float getDistance() const {
        // Simulate reading from sensor
        return 39.37f;  // 1 meter in inches
    }

    // Returns raw voltage (0-5V)
    float getRawVoltage() const {
        return 2.5f;
    }

    // Returns true if voltage is in valid range
    bool isValid() const {
        float voltage = getRawVoltage();
        return voltage >= 0.5f && voltage <= 4.5f;
    }
};

// Modern sensor class (Another Adaptee)
class ModernUltrasonicSensor {
public:
    // Returns distance in centimeters
    double getMeasurement() const {
        // Simulate reading from sensor
        return 100.0;  // 1 meter in cm
    }

    // Returns status code (0 = OK, 1 = Out of range, 2 = Error)
    int getStatus() const {
        return 0;
    }

    // Returns sensor model and firmware version
    std::string getVersion() const {
        return "Ultrasonic-V2 (Firmware: 1.2.3)";
    }
};

// Adapter for Legacy Infrared Sensor
class InfraredSensorAdapter : public IDistanceSensor {
public:
    explicit InfraredSensorAdapter(std::shared_ptr<LegacyInfraredSensor> sensor)
        : sensor_(sensor) {
        RCLCPP_INFO(logger_, "Initializing Infrared Sensor Adapter");
    }

    double getDistanceMeters() override {
        // Convert inches to meters
        return sensor_->getDistance() * 0.0254;
    }

    bool isInRange() override {
        return sensor_->isValid();
    }

    std::string getSensorInfo() override {
        std::stringstream ss;
        ss << "Legacy Infrared Sensor\n"
           << "Raw Voltage: " << sensor_->getRawVoltage() << "V\n"
           << "Valid: " << (sensor_->isValid() ? "Yes" : "No");
        return ss.str();
    }

private:
    std::shared_ptr<LegacyInfraredSensor> sensor_;
    rclcpp::Logger logger_ = rclcpp::get_logger("InfraredSensorAdapter");
};

// Adapter for Modern Ultrasonic Sensor
class UltrasonicSensorAdapter : public IDistanceSensor {
public:
    explicit UltrasonicSensorAdapter(std::shared_ptr<ModernUltrasonicSensor> sensor)
        : sensor_(sensor) {
        RCLCPP_INFO(logger_, "Initializing Ultrasonic Sensor Adapter");
    }

    double getDistanceMeters() override {
        // Convert centimeters to meters
        return sensor_->getMeasurement() / 100.0;
    }

    bool isInRange() override {
        return sensor_->getStatus() == 0;
    }

    std::string getSensorInfo() override {
        std::stringstream ss;
        ss << "Modern Ultrasonic Sensor\n"
           << "Version: " << sensor_->getVersion() << "\n"
           << "Status: " << sensor_->getStatus();
        return ss.str();
    }

private:
    std::shared_ptr<ModernUltrasonicSensor> sensor_;
    rclcpp::Logger logger_ = rclcpp::get_logger("UltrasonicSensorAdapter");
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Adapter Pattern trong một ROS2 node:

```cpp
class DistanceSensorNode : public rclcpp::Node {
public:
    DistanceSensorNode() : Node("distance_sensor_node") {
        // Khởi tạo các sensors
        infrared_sensor_ = std::make_shared<LegacyInfraredSensor>();
        ultrasonic_sensor_ = std::make_shared<ModernUltrasonicSensor>();

        // Tạo adapters
        infrared_adapter_ = std::make_shared<InfraredSensorAdapter>(infrared_sensor_);
        ultrasonic_adapter_ = std::make_shared<UltrasonicSensorAdapter>(ultrasonic_sensor_);

        // Tạo publishers
        infrared_publisher_ = create_publisher<sensor_msgs::msg::Range>(
            "infrared_distance", 10);
        ultrasonic_publisher_ = create_publisher<sensor_msgs::msg::Range>(
            "ultrasonic_distance", 10);

        // Tạo service để lấy sensor info
        sensor_info_service_ = create_service<custom_msgs::srv::GetSensorInfo>(
            "get_sensor_info",
            std::bind(&DistanceSensorNode::handleSensorInfoRequest, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Tạo timer để publish measurements
        measurement_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DistanceSensorNode::publishMeasurements, this));
    }

private:
    void publishMeasurements() {
        publishSensorData(infrared_adapter_, infrared_publisher_, "infrared_frame");
        publishSensorData(ultrasonic_adapter_, ultrasonic_publisher_, "ultrasonic_frame");
    }

    void publishSensorData(
        std::shared_ptr<IDistanceSensor> sensor,
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher,
        const std::string& frame_id) {
        
        auto msg = sensor_msgs::msg::Range();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id;
        msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        msg.field_of_view = 0.5;  // 30 degrees in radians
        msg.min_range = 0.1;      // 10cm
        msg.max_range = 4.0;      // 4m
        
        if (sensor->isInRange()) {
            msg.range = static_cast<float>(sensor->getDistanceMeters());
        } else {
            msg.range = std::numeric_limits<float>::quiet_NaN();
        }
        
        publisher->publish(msg);
    }

    void handleSensorInfoRequest(
        const custom_msgs::srv::GetSensorInfo::Request::SharedPtr request,
        custom_msgs::srv::GetSensorInfo::Response::SharedPtr response) {
        
        if (request->sensor_type == "infrared") {
            response->info = infrared_adapter_->getSensorInfo();
        } else if (request->sensor_type == "ultrasonic") {
            response->info = ultrasonic_adapter_->getSensorInfo();
        } else {
            response->info = "Unknown sensor type";
        }
    }

    // Sensors
    std::shared_ptr<LegacyInfraredSensor> infrared_sensor_;
    std::shared_ptr<ModernUltrasonicSensor> ultrasonic_sensor_;

    // Adapters
    std::shared_ptr<InfraredSensorAdapter> infrared_adapter_;
    std::shared_ptr<UltrasonicSensorAdapter> ultrasonic_adapter_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr infrared_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_publisher_;

    // Service
    rclcpp::Service<custom_msgs::srv::GetSensorInfo>::SharedPtr sensor_info_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr measurement_timer_;
};
```

## 6. Lợi ích

1. **Tái sử dụng code**:
   - Sử dụng được legacy code
   - Tích hợp các thư viện bên ngoài
   - Không cần sửa code gốc

2. **Tính linh hoạt**:
   - Dễ dàng thêm sensors mới
   - Chuyển đổi giữa các formats
   - Tương thích nhiều platforms

3. **Bảo trì dễ dàng**:
   - Code module hóa
   - Dễ test và debug
   - Thay đổi không ảnh hưởng code khác

4. **Chuẩn hóa interface**:
   - Interface thống nhất
   - Giảm phức tạp
   - Dễ quản lý

## 7. Khi nào sử dụng

- Cần tích hợp các hệ thống không tương thích
- Muốn tái sử dụng legacy code
- Cần chuyển đổi dữ liệu giữa các formats
- Muốn chuẩn hóa interface cho nhiều components
- Tích hợp third-party libraries

## 8. Lưu ý

1. Thiết kế Adapter:
   - Giữ adapter đơn giản
   - Chỉ chuyển đổi những gì cần thiết
   - Xử lý edge cases cẩn thận

2. Hiệu suất:
   - Tránh chuyển đổi không cần thiết
   - Cân nhắc caching nếu chuyển đổi phức tạp
   - Theo dõi overhead

3. Trong ROS2:
   - Đảm bảo thread safety
   - Xử lý transform frames đúng
   - Quản lý lifecycle của adapters
   - Implement proper cleanup 