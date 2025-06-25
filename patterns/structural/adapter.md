# ADAPTER PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Tưởng tượng bạn có một robot đang sử dụng cảm biến laser để đo khoảng cách, nhưng bây giờ bạn muốn thêm một cảm biến siêu âm mới. Vấn đề là:

- Cảm biến laser trả về khoảng cách theo mét
- Cảm biến siêu âm trả về khoảng cách theo inch
- Code hiện tại của robot chỉ làm việc với đơn vị mét

Adapter Pattern giống như một "bộ chuyển đổi" giúp cảm biến siêu âm có thể hoạt động với code hiện tại mà không cần thay đổi code của robot.

#### 2. Định nghĩa chi tiết
Adapter Pattern là một mẫu thiết kế cấu trúc cho phép các objects với interfaces không tương thích có thể làm việc cùng nhau. Pattern này hoạt động như một wrapper, chuyển đổi interface của một class thành interface khác mà client mong đợi.

#### Các thành phần chính:
1. **Target Interface**:
   - Interface mà client sử dụng
   - Trong ROS2: Interface chuẩn cho sensors

2. **Adaptee**:
   - Class cần được adapt
   - Trong ROS2: Sensor mới với interface khác

3. **Adapter**:
   - Class thực hiện việc chuyển đổi
   - Kết nối Target Interface với Adaptee

#### 3. Ví dụ thực tế trong ROS2
Giả sử chúng ta cần tích hợp một cảm biến siêu âm mới vào hệ thống robot hiện có:

```cpp
// 1. Target Interface - Interface chuẩn cho cảm biến khoảng cách
class DistanceSensorInterface {
public:
    virtual ~DistanceSensorInterface() = default;
    
    // Các phương thức chuẩn
    virtual void initialize() = 0;
    virtual double getDistanceInMeters() = 0;
    virtual std::string getSensorType() const = 0;
    virtual double getMaxRange() const = 0;
    virtual double getMinRange() const = 0;
    virtual double getFieldOfView() const = 0;  // radians
};

// 2. Existing Sensor - Cảm biến laser hiện có
class LaserSensor : public DistanceSensorInterface {
public:
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Laser Sensor");
        // Khởi tạo hardware
    }
    
    double getDistanceInMeters() override {
        // Đọc dữ liệu từ laser
        return current_distance_;  // Đã ở đơn vị mét
    }
    
    std::string getSensorType() const override {
        return "LASER";
    }
    
    double getMaxRange() const override {
        return 30.0;  // 30 meters
    }
    
    double getMinRange() const override {
        return 0.1;  // 10 cm
    }
    
    double getFieldOfView() const override {
        return M_PI / 180.0 * 1.0;  // 1 degree
    }

private:
    double current_distance_ = 0.0;
    rclcpp::Logger logger_{rclcpp::get_logger("LaserSensor")};
};

// 3. Adaptee - Cảm biến siêu âm mới (Third-party/Legacy code)
class UltrasonicSensor {
public:
    // Interface khác với chuẩn
    bool connect() {
        // Kết nối với sensor
        return true;
    }
    
    float getRangeInInches() {
        // Đọc khoảng cách theo inch
        return current_range_;
    }
    
    bool isConnected() const {
        return connected_;
    }
    
    float getBeamWidth() const {
        return 15.0f;  // 15 degrees
    }
    
    float getMaxInches() const {
        return 157.48f;  // 4 meters in inches
    }

private:
    float current_range_ = 0.0f;
    bool connected_ = false;
};

// 4. Adapter - Chuyển đổi UltrasonicSensor sang DistanceSensorInterface
class UltrasonicAdapter : public DistanceSensorInterface {
public:
    explicit UltrasonicAdapter(std::shared_ptr<UltrasonicSensor> sensor)
        : ultrasonic_(sensor) {
        if (!ultrasonic_) {
            throw std::runtime_error("Null ultrasonic sensor provided");
        }
    }
    
    void initialize() override {
        RCLCPP_INFO(logger_, "Initializing Ultrasonic Sensor");
        if (!ultrasonic_->connect()) {
            throw std::runtime_error("Failed to connect to ultrasonic sensor");
        }
    }
    
    double getDistanceInMeters() override {
        if (!ultrasonic_->isConnected()) {
            throw std::runtime_error("Ultrasonic sensor not connected");
        }
        
        // Chuyển đổi từ inch sang mét
        return ultrasonic_->getRangeInInches() * 0.0254;
    }
    
    std::string getSensorType() const override {
        return "ULTRASONIC";
    }
    
    double getMaxRange() const override {
        // Chuyển đổi max range từ inch sang mét
        return ultrasonic_->getMaxInches() * 0.0254;
    }
    
    double getMinRange() const override {
        return 0.02;  // 2 cm
    }
    
    double getFieldOfView() const override {
        // Chuyển đổi từ độ sang radian
        return ultrasonic_->getBeamWidth() * M_PI / 180.0;
    }

private:
    std::shared_ptr<UltrasonicSensor> ultrasonic_;
    rclcpp::Logger logger_{rclcpp::get_logger("UltrasonicAdapter")};
};

// 5. Client code trong ROS2 node
class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode() : Node("obstacle_detection") {
        // Khởi tạo các sensors
        setupSensors();
        
        // Tạo publisher cho khoảng cách
        distance_pub_ = create_publisher<sensor_msgs::msg::Range>(
            "obstacle_distance", 10);
            
        // Timer để publish dữ liệu
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleDetectionNode::publishDistances, this));
    }

private:
    void setupSensors() {
        try {
            // Khởi tạo laser sensor
            laser_sensor_ = std::make_shared<LaserSensor>();
            laser_sensor_->initialize();
            
            // Khởi tạo ultrasonic sensor với adapter
            auto ultrasonic = std::make_shared<UltrasonicSensor>();
            ultrasonic_sensor_ = std::make_shared<UltrasonicAdapter>(ultrasonic);
            ultrasonic_sensor_->initialize();
            
            RCLCPP_INFO(get_logger(), "All sensors initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error initializing sensors: %s", e.what());
            throw;
        }
    }
    
    void publishDistances() {
        try {
            // Publish dữ liệu từ cả hai sensor
            publishSensorData(laser_sensor_, "laser");
            publishSensorData(ultrasonic_sensor_, "ultrasonic");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error publishing distances: %s", e.what());
        }
    }
    
    void publishSensorData(
        const std::shared_ptr<DistanceSensorInterface>& sensor,
        const std::string& frame_id) {
        
        auto msg = sensor_msgs::msg::Range();
        msg.header.frame_id = frame_id;
        msg.header.stamp = now();
        msg.radiation_type = 
            (sensor->getSensorType() == "LASER") ? 
            sensor_msgs::msg::Range::INFRARED : 
            sensor_msgs::msg::Range::ULTRASOUND;
        msg.field_of_view = sensor->getFieldOfView();
        msg.min_range = sensor->getMinRange();
        msg.max_range = sensor->getMaxRange();
        msg.range = sensor->getDistanceInMeters();
        
        distance_pub_->publish(msg);
    }
    
    std::shared_ptr<DistanceSensorInterface> laser_sensor_;
    std::shared_ptr<DistanceSensorInterface> ultrasonic_sensor_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distance_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

#### 4. Giải thích chi tiết cách hoạt động
1. **Khởi tạo hệ thống**:
   - Tạo interface chuẩn cho cảm biến khoảng cách
   - Laser sensor implement interface này trực tiếp
   - Ultrasonic sensor được wrap bởi adapter

2. **Chuyển đổi dữ liệu**:
   - Adapter chuyển đổi đơn vị từ inch sang mét
   - Chuyển đổi các thông số khác (FOV, ranges)
   - Xử lý lỗi và exceptions

3. **Publishing dữ liệu**:
   - Node sử dụng interface chung cho cả hai sensor
   - Publish dữ liệu theo định dạng ROS2 standard
   - Xử lý lỗi gracefully

#### 5. Ưu điểm trong ROS2
1. **Tích hợp linh hoạt**:
   - Dễ dàng thêm sensors mới
   - Không cần sửa code hiện có
   - Tái sử dụng code tối đa

2. **Bảo trì dễ dàng**:
   - Tách biệt logic chuyển đổi
   - Dễ thay đổi/cập nhật adapter
   - Code sạch và có tổ chức

3. **Xử lý lỗi tốt**:
   - Kiểm tra null pointers
   - Xử lý connection errors
   - Exception handling

#### 6. Các trường hợp sử dụng trong ROS2
1. **Hardware Integration**:
   - Tích hợp sensors mới
   - Sử dụng drivers cũ
   - Chuyển đổi data formats

2. **Message Adaptation**:
   - Chuyển đổi message types
   - Custom message bridges
   - Protocol adaptation

3. **Legacy System Integration**:
   - Tích hợp code cũ
   - Cập nhật interfaces
   - Backwards compatibility

#### 7. Best Practices trong ROS2
1. **Error Handling**:
```cpp
try {
    double distance = sensor->getDistanceInMeters();
    if (distance < 0) {
        throw std::runtime_error("Invalid distance reading");
    }
} catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Sensor error: %s", e.what());
    // Fallback behavior
}
```

2. **Resource Management**:
```cpp
class SensorAdapter {
    std::unique_ptr<SensorDriver> driver_;
public:
    ~SensorAdapter() {
        if (driver_) {
            driver_->disconnect();
        }
    }
};
```

3. **Thread Safety**:
```cpp
std::mutex sensor_mutex_;
double getDistance() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return sensor_->getDistanceInMeters();
}
```

#### 8. Mở rộng và tùy chỉnh
1. **Two-Way Adapter**:
```cpp
class BidirectionalAdapter : public NewInterface, public OldInterface {
    // Implement both interfaces
    // Convert in both directions
};
```

2. **Multiple Adaptation**:
```cpp
class MultiSensorAdapter : public DistanceSensorInterface {
    std::vector<std::unique_ptr<BaseSensor>> sensors_;
public:
    double getDistanceInMeters() override {
        // Combine data from multiple sensors
    }
};
```

3. **Dynamic Adaptation**:
```cpp
class ConfigurableAdapter : public DistanceSensorInterface {
    std::map<std::string, double> conversion_factors_;
public:
    void updateConversion(const std::string& unit, double factor) {
        conversion_factors_[unit] = factor;
    }
};
```

#### 9. Testing
1. **Mock Objects**:
```cpp
class MockUltrasonicSensor : public UltrasonicSensor {
public:
    MOCK_METHOD(float, getRangeInInches, (), (override));
    MOCK_METHOD(bool, connect, (), (override));
};
```

2. **Unit Tests**:
```cpp
TEST(UltrasonicAdapterTest, ConversionTest) {
    auto mock_sensor = std::make_shared<MockUltrasonicSensor>();
    EXPECT_CALL(*mock_sensor, getRangeInInches())
        .WillOnce(Return(39.37f));  // 1 meter
        
    UltrasonicAdapter adapter(mock_sensor);
    EXPECT_NEAR(adapter.getDistanceInMeters(), 1.0, 0.001);
}
```

3. **Integration Tests**:
```cpp
TEST(SensorSystemTest, FullSystemTest) {
    ObstacleDetectionNode node;
    
    // Test sensor initialization
    EXPECT_NO_THROW(node.setupSensors());
    
    // Test data publishing
    auto messages = std::make_shared<MessageCollector>();
    node.addSubscriber(messages);
    
    // Verify message contents
    EXPECT_TRUE(messages->waitForMessage(1s));
    auto msg = messages->getLastMessage();
    EXPECT_TRUE(msg.range > 0);
}
```

#### 10. Kết luận
Adapter Pattern là một mẫu thiết kế cấu trúc quan trọng trong phát triển phần mềm robotics với ROS2, đặc biệt trong việc tích hợp các thành phần không tương thích. Pattern này mang lại nhiều giá trị thực tiễn:

1. **Tích hợp linh hoạt**:
   - Cho phép sử dụng các sensors và thiết bị mới mà không cần sửa đổi code hiện có
   - Hỗ trợ việc chuyển đổi giữa các định dạng dữ liệu khác nhau
   - Tạo cầu nối giữa các hệ thống legacy và modern

2. **Tối ưu cho robotics**:
   - Dễ dàng thêm các sensors mới vào hệ thống
   - Chuyển đổi mượt mà giữa các đơn vị đo lường
   - Tương thích với nhiều loại hardware interfaces

3. **Bảo trì hiệu quả**:
   - Tách biệt logic chuyển đổi khỏi business logic
   - Dễ dàng cập nhật và thay đổi adapters
   - Code sạch và có cấu trúc rõ ràng

4. **Giá trị thực tế**:
   - Tiết kiệm thời gian và chi phí phát triển
   - Giảm thiểu rủi ro khi tích hợp components mới
   - Tăng khả năng tái sử dụng code

Trong ví dụ về việc tích hợp cảm biến siêu âm mới vào hệ thống robot, chúng ta đã thấy Adapter Pattern giúp giải quyết vấn đề không tương thích một cách thanh lịch và hiệu quả. Pattern này là công cụ thiết yếu cho các nhà phát triển ROS2 trong việc xây dựng hệ thống robotics linh hoạt và dễ mở rộng. 