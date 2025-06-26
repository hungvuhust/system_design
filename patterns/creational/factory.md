# FACTORY PATTERN 

## 1. Giới thiệu đơn giản
Factory Pattern là một mẫu thiết kế khởi tạo cho phép tạo các đối tượng mà không cần chỉ định chính xác lớp của chúng. Trong ROS2, pattern này đặc biệt hữu ích khi:

- Tạo các node plugins khác nhau
- Khởi tạo các sensor drivers
- Tạo các message handlers
- Quản lý các loại controllers khác nhau

## 2. Định nghĩa chi tiết
Factory Pattern định nghĩa một interface để tạo đối tượng nhưng để các lớp con quyết định lớp nào sẽ được khởi tạo. Pattern này cho phép một lớp hoãn việc khởi tạo sang lớp con.

#### Các thành phần chính:
1. **Product Interface**:
   - Interface chung cho tất cả products
   - Định nghĩa các operations chuẩn

2. **Concrete Products**:
   - Các implementations cụ thể
   - Tuân theo product interface

3. **Factory Interface**:
   - Định nghĩa phương thức tạo product
   - Có thể có nhiều factory methods

4. **Concrete Factory**:
   - Implements factory interface
   - Tạo các concrete products

## 3. Ví dụ thực tế trong ROS2
```cpp
// 1. Product Interface
class SensorDriver {
public:
    virtual ~SensorDriver() = default;
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual std::vector<double> getData() = 0;
    virtual std::string getSensorType() const = 0;
    virtual void setParameters(const std::map<std::string, std::string>& params) = 0;
};

// 2. Concrete Products
class LidarDriver : public SensorDriver {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Đọc parameters
            auto params = node_->get_parameters("lidar");
            port_ = params[0].as_string();
            baud_rate_ = params[1].as_int();
            
            // Khởi tạo lidar connection
            return initializeLidarConnection();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize LidarDriver: %s", e.what());
            return false;
        }
    }
    
    bool start() override {
        if (!is_initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "LidarDriver not initialized");
            return false;
        }
        
        try {
            // Bắt đầu scanning
            startScanning();
            is_running_ = true;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to start LidarDriver: %s", e.what());
            return false;
        }
    }
    
    bool stop() override {
        if (!is_running_) {
            return true;
        }
        
        try {
            // Dừng scanning
            stopScanning();
            is_running_ = false;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to stop LidarDriver: %s", e.what());
            return false;
        }
    }
    
    std::vector<double> getData() override {
        if (!is_running_) {
            RCLCPP_WARN(node_->get_logger(), "LidarDriver not running");
            return std::vector<double>();
        }
        
        try {
            // Đọc và xử lý dữ liệu lidar
            return readLidarData();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to read LidarDriver data: %s", e.what());
            return std::vector<double>();
        }
    }
    
    std::string getSensorType() const override {
        return "LIDAR";
    }
    
    void setParameters(const std::map<std::string, std::string>& params) override {
        for (const auto& [key, value] : params) {
            if (key == "port") {
                port_ = value;
            } else if (key == "baud_rate") {
                baud_rate_ = std::stoi(value);
            }
            // Thêm các parameters khác
        }
    }

private:
    bool initializeLidarConnection() {
        // Khởi tạo kết nối với lidar hardware
        return true;
    }
    
    void startScanning() {
        // Bắt đầu quét lidar
    }
    
    void stopScanning() {
        // Dừng quét lidar
    }
    
    std::vector<double> readLidarData() {
        // Đọc và xử lý dữ liệu từ lidar
        return std::vector<double>();
    }
    
    rclcpp::Node::SharedPtr node_;
    std::string port_;
    int baud_rate_;
    bool is_initialized_ = false;
    bool is_running_ = false;
};

class CameraDriver : public SensorDriver {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Đọc camera parameters
            resolution_ = params[0].as_string();
            fps_ = params[1].as_int();
            
            // Khởi tạo camera
            return initializeCamera();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize CameraDriver: %s", e.what());
            return false;
        }
    }
    
    bool start() override {
        if (!is_initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "CameraDriver not initialized");
            return false;
        }
        
        try {
            // Bắt đầu streaming
            startStreaming();
            is_running_ = true;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to start CameraDriver: %s", e.what());
            return false;
        }
    }
    
    bool stop() override {
        if (!is_running_) {
            return true;
        }
        
        try {
            // Dừng streaming
            stopStreaming();
            is_running_ = false;
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to stop CameraDriver: %s", e.what());
            return false;
        }
    }
    
    std::vector<double> getData() override {
        if (!is_running_) {
            RCLCPP_WARN(node_->get_logger(), "CameraDriver not running");
            return std::vector<double>();
        }
        
        try {
            // Đọc và xử lý frame
            return processFrame();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to read CameraDriver data: %s", e.what());
            return std::vector<double>();
        }
    }
    
    std::string getSensorType() const override {
        return "CAMERA";
    }
    
    void setParameters(const std::map<std::string, std::string>& params) override {
        for (const auto& [key, value] : params) {
            if (key == "resolution") {
                resolution_ = value;
            } else if (key == "fps") {
                fps_ = std::stoi(value);
            }
            // Thêm các parameters khác
        }
    }

private:
    bool initializeCamera() {
        // Khởi tạo camera
        return true;
    }
    
    void startStreaming() {
        // Bắt đầu streaming
    }
    
    void stopStreaming() {
        // Dừng streaming
    }
    
    std::vector<double> processFrame() {
        // Xử lý frame hiện tại
        return std::vector<double>();
    }
    
    rclcpp::Node::SharedPtr node_;
    std::string resolution_;
    int fps_;
    bool is_initialized_ = false;
    bool is_running_ = false;
};

// 3. Factory Interface
class SensorDriverFactory {
public:
    virtual ~SensorDriverFactory() = default;
    virtual std::unique_ptr<SensorDriver> createDriver(
        const std::string& sensor_type,
        const std::map<std::string, std::string>& params) = 0;
};

// 4. Concrete Factory
class RobotSensorFactory : public SensorDriverFactory {
public:
    std::unique_ptr<SensorDriver> createDriver(
        const std::string& sensor_type,
        const std::map<std::string, std::string>& params) override {
        
        std::unique_ptr<SensorDriver> driver = nullptr;
        
        if (sensor_type == "LIDAR") {
            driver = std::make_unique<LidarDriver>();
        } else if (sensor_type == "CAMERA") {
            driver = std::make_unique<CameraDriver>();
        } else {
            throw std::runtime_error("Unknown sensor type: " + sensor_type);
        }
        
        if (driver) {
            driver->setParameters(params);
        }
        
        return driver;
    }
};

// 5. Usage Example
class RobotSensorNode : public rclcpp::Node {
public:
    RobotSensorNode() : Node("robot_sensor") {
        // Tạo factory
        sensor_factory_ = std::make_unique<RobotSensorFactory>();
        
        // Khởi tạo các sensors
        initializeSensors();
        
        // Tạo timer để publish sensor data
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotSensorNode::publishSensorData, this));
    }
    
    ~RobotSensorNode() {
        // Cleanup
        for (auto& sensor : sensors_) {
            sensor->stop();
        }
    }

private:
    void initializeSensors() {
        try {
            // Khởi tạo LIDAR
            std::map<std::string, std::string> lidar_params = {
                {"port", "/dev/ttyUSB0"},
                {"baud_rate", "115200"}
            };
            auto lidar = sensor_factory_->createDriver("LIDAR", lidar_params);
            if (lidar->initialize(shared_from_this())) {
                lidar->start();
                sensors_.push_back(std::move(lidar));
            }
            
            // Khởi tạo Camera
            std::map<std::string, std::string> camera_params = {
                {"resolution", "640x480"},
                {"fps", "30"}
            };
            auto camera = sensor_factory_->createDriver("CAMERA", camera_params);
            if (camera->initialize(shared_from_this())) {
                camera->start();
                sensors_.push_back(std::move(camera));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize sensors: %s", e.what());
        }
    }
    
    void publishSensorData() {
        for (auto& sensor : sensors_) {
            try {
                auto data = sensor->getData();
                // Process và publish data
                publishData(sensor->getSensorType(), data);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), 
                    "Error reading %s data: %s", 
                    sensor->getSensorType().c_str(), e.what());
            }
        }
    }
    
    void publishData(const std::string& sensor_type, 
                    const std::vector<double>& data) {
        // Publish sensor data
    }
    
    std::unique_ptr<SensorDriverFactory> sensor_factory_;
    std::vector<std::unique_ptr<SensorDriver>> sensors_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#### 4. Giải thích chi tiết cách hoạt động
1. **Product Interface**:
   - SensorDriver định nghĩa interface chung
   - Các phương thức chuẩn cho mọi sensor
   - Tính đa hình thông qua virtual methods

2. **Concrete Products**:
   - LidarDriver và CameraDriver implements interface
   - Xử lý hardware-specific logic
   - Error handling và logging

3. **Factory Pattern**:
   - Factory method tạo sensor drivers
   - Encapsulation của object creation
   - Dynamic object creation

#### 5. Ưu điểm trong ROS2
1. **Flexibility**:
   - Dễ dàng thêm sensors mới
   - Không cần sửa code hiện có
   - Runtime configuration

2. **Maintainability**:
   - Code organization rõ ràng
   - Separation of concerns
   - Dễ test và debug

3. **Reusability**:
   - Common interface cho sensors
   - Shared functionality
   - Code reuse

#### 6. Các trường hợp sử dụng trong ROS2
1. **Sensor Systems**:
   - Different types of sensors
   - Multiple sensor configurations
   - Sensor fusion systems

2. **Plugin Management**:
   - ROS2 plugins
   - Custom node creation
   - Dynamic loading

3. **Message Handlers**:
   - Custom message types
   - Protocol adapters
   - Communication handlers

#### 7. Best Practices trong ROS2

1. **Error Handling**:
```cpp
try {
    auto driver = factory->createDriver("LIDAR", params);
    if (!driver->initialize(node)) {
        RCLCPP_ERROR(logger, "Driver initialization failed");
        return;
    }
} catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error creating driver: %s", e.what());
}
```

2. **Parameter Management**:
```cpp
void loadParameters() {
    auto params = node_->get_parameters("sensor");
    for (const auto& param : params) {
        config_[param.get_name()] = param.value_to_string();
    }
}
```

3. **Resource Management**:
```cpp
class SafeDriver {
    std::unique_ptr<SensorDriver> driver_;
public:
    ~SafeDriver() {
        if (driver_) {
            driver_->stop();
        }
    }
};
```

#### 8. Mở rộng và tùy chỉnh
1. **Dynamic Loading**:
```cpp
class PluginFactory : public SensorDriverFactory {
    std::unique_ptr<SensorDriver> createDriver(
        const std::string& type,
        const std::map<std::string, std::string>& params) override {
        // Load plugin dynamically
        return loadPlugin(type, params);
    }
};
```

2. **Configuration System**:
```cpp
class ConfigurableFactory : public SensorDriverFactory {
    void configure(const YAML::Node& config) {
        // Configure factory from YAML
        loadConfiguration(config);
    }
};
```

3. **Validation System**:
```cpp
class ValidatingFactory : public SensorDriverFactory {
    bool validateParams(const std::map<std::string, std::string>& params) {
        // Validate parameters
        return checkParameters(params);
    }
};
```

#### 9. Testing
1. **Mock Objects**:
```cpp
class MockSensorDriver : public SensorDriver {
public:
    MOCK_METHOD(bool, initialize, (const rclcpp::Node::SharedPtr&), (override));
    MOCK_METHOD(bool, start, (), (override));
    MOCK_METHOD(std::vector<double>, getData, (), (override));
};
```

2. **Factory Tests**:
```cpp
TEST(FactoryTest, CreateLidarDriver) {
    auto factory = std::make_unique<RobotSensorFactory>();
    auto driver = factory->createDriver("LIDAR", {});
    EXPECT_NE(driver, nullptr);
    EXPECT_EQ(driver->getSensorType(), "LIDAR");
}
```

3. **Integration Tests**:
```cpp
TEST(SensorSystemTest, FullSystemTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto factory = std::make_unique<RobotSensorFactory>();
    
    // Test LIDAR creation and initialization
    auto lidar = factory->createDriver("LIDAR", {
        {"port", "/dev/ttyUSB0"},
        {"baud_rate", "115200"}
    });
    EXPECT_TRUE(lidar->initialize(node));
    EXPECT_TRUE(lidar->start());
    
    // Test data acquisition
    auto data = lidar->getData();
    EXPECT_FALSE(data.empty());
    
    // Cleanup
    EXPECT_TRUE(lidar->stop());
}
```

# Factory Method Pattern in ROS2

## What is the Factory Method Pattern?

The Factory Method pattern is a creational design pattern that provides an interface for creating objects in a superclass, but allows subclasses to alter the type of objects that will be created. It defines an interface for creating an object, but lets subclasses decide which class to instantiate.

## Use Case in Robotics

In robotics, the Factory Method pattern is useful when you have a class that needs to create objects, but you want to allow subclasses to specify the exact type of object to create. For example, you might have a generic `Robot` class that needs to create a `PathPlanner` object. Different types of robots might require different path planning algorithms (e.g., A*, RRT, DWA). You can use the Factory Method to let subclasses of `Robot` (e.g., `DifferentialDriveRobot`, `LeggedRobot`) create the appropriate planner instance.

This allows you to write generic robot logic that works with any type of planner, as long as it conforms to the `PathPlanner` interface.

## C++ Example

Here is a C++ example of the Factory Method pattern for creating path planners for different robot types.

```cpp
// Product interface
class PathPlanner {
public:
    virtual ~PathPlanner() {}
    virtual void planPath(double startX, double startY, double endX, double endY) = 0;
};

// Concrete Product A
class AStarPlanner : public PathPlanner {
public:
    void planPath(double startX, double startY, double endX, double endY) override {
        // A* planning logic
    }
};

// Concrete Product B
class RRTPlanner : public PathPlanner {
public:
    void planPath(double startX, double startY, double endX, double endY) override {
        // RRT planning logic
    }
};

// Creator (defines the factory method)
class Robot {
protected:
    PathPlanner* planner;

public:
    virtual ~Robot() { delete planner; }

    // The factory method
    virtual PathPlanner* createPlanner() = 0;

    void planNavigation() {
        planner = createPlanner();
        planner->planPath(0, 0, 10, 10);
    }
};

// Concrete Creator A
class GroundRobot : public Robot {
public:
    PathPlanner* createPlanner() override {
        return new AStarPlanner();
    }
};

// Concrete Creator B
class AerialRobot : public Robot {
public:
    PathPlanner* createPlanner() override {
        return new RRTPlanner();
    }
};
```

## Best Practices

*   **Depend on Abstractions:** The creator class should depend on the abstract product interface, not the concrete product classes.
*   **Default Implementation:** The creator can provide a default implementation of the factory method that creates a default concrete product.
*   **Parameterization:** The factory method can be parameterized to create different variations of a product.

## Extensions and Variations

*   **Static Factory:** A static method in the creator class can act as a factory. This is simpler but not as flexible as the classic Factory Method, as it cannot be overridden by subclasses.

## Testing

*   **Unit Testing:** Test each concrete creator to ensure that it creates the correct type of product.
*   **Integration Testing:** Test the interaction between the creator and the products it creates.