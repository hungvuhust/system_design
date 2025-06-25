## OBSERVER PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Tưởng tượng bạn có một robot với nhiều cảm biến nhiệt độ ở các bộ phận khác nhau (động cơ, pin, bo mạch). Khi nhiệt độ của bất kỳ bộ phận nào vượt ngưỡng:

- Hệ thống cần gửi cảnh báo
- Giảm tốc độ robot
- Ghi log để phân tích sau

Observer Pattern giống như một "hệ thống theo dõi" tự động thông báo cho tất cả các thành phần quan tâm khi có thay đổi, thay vì phải liên tục kiểm tra.

#### 2. Định nghĩa chi tiết
Observer Pattern là một mẫu thiết kế hành vi cho phép bạn định nghĩa cơ chế đăng ký (subscription) để thông báo cho nhiều đối tượng về bất kỳ sự kiện nào xảy ra với đối tượng mà họ đang quan sát.

#### Các thành phần chính:
1. **Subject (Observable)**:
   - Đối tượng chứa trạng thái cần theo dõi
   - Trong ROS2: Temperature monitor

2. **Observer**:
   - Interface cho các observers
   - Trong ROS2: Temperature handlers

3. **Concrete Observers**:
   - Các observers cụ thể
   - Trong ROS2: Alert, Speed Control, Logger

#### 3. Ví dụ thực tế trong ROS2
Giả sử chúng ta cần xây dựng một hệ thống giám sát nhiệt độ cho robot:

```cpp
// 1. Observer Interface
class TemperatureObserver {
public:
    virtual ~TemperatureObserver() = default;
    
    virtual void update(const std::string& sensor_id,
                       double temperature,
                       double threshold) = 0;
                       
    virtual std::string getObserverType() const = 0;
};

// 2. Subject Interface
class TemperatureSubject {
public:
    virtual ~TemperatureSubject() = default;
    
    virtual void attach(std::shared_ptr<TemperatureObserver> observer) = 0;
    virtual void detach(std::shared_ptr<TemperatureObserver> observer) = 0;
    virtual void notify(const std::string& sensor_id,
                       double temperature,
                       double threshold) = 0;
};

// 3. Concrete Subject
class TemperatureMonitor : public TemperatureSubject,
                          public rclcpp::Node {
public:
    explicit TemperatureMonitor()
        : Node("temperature_monitor") {
        // Khởi tạo parameters
        initializeParameters();
        
        // Tạo subscribers cho các sensors
        createSensorSubscribers();
        
        // Timer để kiểm tra nhiệt độ định kỳ
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TemperatureMonitor::checkTemperatures, this));
            
        RCLCPP_INFO(get_logger(), "Temperature Monitor initialized");
    }
    
    void attach(std::shared_ptr<TemperatureObserver> observer) override {
        std::lock_guard<std::mutex> lock(observers_mutex_);
        observers_.push_back(observer);
        RCLCPP_INFO(get_logger(),
            "Attached observer: %s",
            observer->getObserverType().c_str());
    }
    
    void detach(std::shared_ptr<TemperatureObserver> observer) override {
        std::lock_guard<std::mutex> lock(observers_mutex_);
        observers_.erase(
            std::remove_if(
                observers_.begin(),
                observers_.end(),
                [&](const auto& obs) {
                    return obs->getObserverType() == observer->getObserverType();
                }),
            observers_.end());
    }
    
    void notify(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        std::lock_guard<std::mutex> lock(observers_mutex_);
        for (const auto& observer : observers_) {
            try {
                observer->update(sensor_id, temperature, threshold);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(),
                    "Error notifying observer %s: %s",
                    observer->getObserverType().c_str(),
                    e.what());
            }
        }
    }

private:
    void initializeParameters() {
        // Khai báo parameters
        declare_parameter("motor_temp_threshold", 80.0);
        declare_parameter("battery_temp_threshold", 60.0);
        declare_parameter("electronics_temp_threshold", 70.0);
        
        // Load thresholds
        thresholds_["motor"] = 
            get_parameter("motor_temp_threshold").as_double();
        thresholds_["battery"] = 
            get_parameter("battery_temp_threshold").as_double();
        thresholds_["electronics"] = 
            get_parameter("electronics_temp_threshold").as_double();
    }
    
    void createSensorSubscribers() {
        // Tạo subscribers cho từng sensor
        motor_temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "motor_temperature", 10,
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                temperatures_["motor"] = msg->temperature;
            });
            
        battery_temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "battery_temperature", 10,
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                temperatures_["battery"] = msg->temperature;
            });
            
        electronics_temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "electronics_temperature", 10,
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                temperatures_["electronics"] = msg->temperature;
            });
    }
    
    void checkTemperatures() {
        for (const auto& [sensor_id, temperature] : temperatures_) {
            auto threshold = thresholds_[sensor_id];
            if (temperature > threshold) {
                notify(sensor_id, temperature, threshold);
            }
        }
    }
    
    std::vector<std::shared_ptr<TemperatureObserver>> observers_;
    std::mutex observers_mutex_;
    std::map<std::string, double> temperatures_;
    std::map<std::string, double> thresholds_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr 
        motor_temp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr 
        battery_temp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr 
        electronics_temp_sub_;
};

// 4. Concrete Observers

// 4.1 Alert Observer
class AlertObserver : public TemperatureObserver,
                     public rclcpp::Node {
public:
    AlertObserver()
        : Node("temperature_alert") {
        // Publisher cho alerts
        alert_pub_ = create_publisher<std_msgs::msg::String>(
            "temperature_alerts", 10);
    }
    
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        auto msg = std_msgs::msg::String();
        msg.data = fmt::format(
            "WARNING: {} temperature ({:.1f}°C) exceeded threshold ({:.1f}°C)",
            sensor_id, temperature, threshold);
            
        alert_pub_->publish(msg);
        RCLCPP_WARN(get_logger(), "%s", msg.data.c_str());
    }
    
    std::string getObserverType() const override {
        return "ALERT_OBSERVER";
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

// 4.2 Speed Control Observer
class SpeedControlObserver : public TemperatureObserver,
                           public rclcpp::Node {
public:
    SpeedControlObserver()
        : Node("temperature_speed_control") {
        // Publisher cho speed commands
        speed_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
    }
    
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        // Tính toán speed reduction factor
        double temp_diff = temperature - threshold;
        double reduction_factor = std::max(0.0,
            1.0 - (temp_diff / threshold) * 0.5);
            
        // Publish reduced speed command
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = max_speed_ * reduction_factor;
        speed_pub_->publish(cmd);
        
        RCLCPP_INFO(get_logger(),
            "Reducing speed to %.1f%% due to high %s temperature",
            reduction_factor * 100.0, sensor_id.c_str());
    }
    
    std::string getObserverType() const override {
        return "SPEED_CONTROL_OBSERVER";
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
    const double max_speed_ = 1.0;  // m/s
};

// 4.3 Logger Observer
class LoggerObserver : public TemperatureObserver,
                      public rclcpp::Node {
public:
    LoggerObserver()
        : Node("temperature_logger") {
        // Tạo log file
        std::string log_dir = "temperature_logs";
        if (!std::filesystem::exists(log_dir)) {
            std::filesystem::create_directory(log_dir);
        }
        
        auto timestamp = 
            std::chrono::system_clock::now().time_since_epoch().count();
        log_file_.open(
            fmt::format("{}/temp_log_{}.csv", log_dir, timestamp));
            
        // Write header
        log_file_ << "timestamp,sensor_id,temperature,threshold\n";
    }
    
    ~LoggerObserver() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }
    
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Log file not open");
            return;
        }
        
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
            
        log_file_ << fmt::format("{},{},{:.1f},{:.1f}\n",
            timestamp, sensor_id, temperature, threshold);
        log_file_.flush();
        
        RCLCPP_DEBUG(get_logger(),
            "Logged temperature data for %s", sensor_id.c_str());
    }
    
    std::string getObserverType() const override {
        return "LOGGER_OBSERVER";
    }

private:
    std::ofstream log_file_;
};

// 5. Main node để khởi tạo và kết nối các components
class TemperatureMonitoringNode : public rclcpp::Node {
public:
    TemperatureMonitoringNode()
        : Node("temperature_monitoring") {
        // Khởi tạo monitor
        monitor_ = std::make_shared<TemperatureMonitor>();
        
        // Khởi tạo và đăng ký các observers
        auto alert_observer = std::make_shared<AlertObserver>();
        auto speed_observer = std::make_shared<SpeedControlObserver>();
        auto logger_observer = std::make_shared<LoggerObserver>();
        
        monitor_->attach(alert_observer);
        monitor_->attach(speed_observer);
        monitor_->attach(logger_observer);
        
        RCLCPP_INFO(get_logger(), "Temperature monitoring system initialized");
    }

private:
    std::shared_ptr<TemperatureMonitor> monitor_;
};

// 6. Main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Tạo executor để chạy nhiều nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // Khởi tạo node
    auto node = std::make_shared<TemperatureMonitoringNode>();
    
    // Thêm node vào executor
    executor.add_node(node);
    
    // Spin
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

#### 4. Giải thích chi tiết cách hoạt động
1. **Khởi tạo hệ thống**:
   - Tạo temperature monitor
   - Đăng ký các observers
   - Khởi tạo ROS2 subscribers

2. **Monitoring loop**:
   - Đọc nhiệt độ từ các sensors
   - So sánh với ngưỡng
   - Notify observers khi vượt ngưỡng

3. **Observer actions**:
   - Alert: Publish cảnh báo
   - Speed Control: Giảm tốc độ
   - Logger: Ghi log file

#### 5. Ưu điểm trong ROS2
1. **Loose Coupling**:
   - Monitor không biết về observers
   - Dễ thêm/xóa observers
   - Không ảnh hưởng code hiện có

2. **Real-time Response**:
   - Phản ứng ngay khi có thay đổi
   - Không cần polling
   - Giảm latency

3. **Extensibility**:
   - Dễ thêm sensors mới
   - Dễ thêm observers mới
   - Tái sử dụng code tốt

#### 6. Các trường hợp sử dụng trong ROS2
1. **Sensor Monitoring**:
   - Temperature monitoring
   - Battery monitoring
   - Error detection

2. **Robot State**:
   - Position tracking
   - Velocity monitoring
   - Load monitoring

3. **System Events**:
   - Emergency stops
   - Collision detection
   - Safety violations

#### 7. Best Practices trong ROS2
1. **Thread Safety**:
```cpp
class SafeObserver : public TemperatureObserver {
    std::mutex update_mutex_;
public:
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        std::lock_guard<std::mutex> lock(update_mutex_);
        // Handle update
    }
};
```

2. **Error Handling**:
```cpp
void notify(const std::string& sensor_id,
            double temperature,
            double threshold) override {
    try {
        for (const auto& observer : observers_) {
            observer->update(sensor_id, temperature, threshold);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Notification error: %s", e.what());
    }
}
```

3. **Resource Management**:
```cpp
class AutoCleanupObserver : public TemperatureObserver {
    std::unique_ptr<std::ofstream> log_file_;
public:
    ~AutoCleanupObserver() {
        if (log_file_ && log_file_->is_open()) {
            log_file_->close();
        }
    }
};
```

#### 8. Mở rộng và tùy chỉnh
1. **Priority Observers**:
```cpp
class PriorityObserver : public TemperatureObserver {
    int priority_;
public:
    explicit PriorityObserver(int priority) : priority_(priority) {}
    int getPriority() const { return priority_; }
};
```

2. **Filtered Notifications**:
```cpp
class FilteredObserver : public TemperatureObserver {
    double min_threshold_;
public:
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        if (temperature > min_threshold_) {
            // Process update
        }
    }
};
```

3. **Composite Observers**:
```cpp
class CompositeObserver : public TemperatureObserver {
    std::vector<std::shared_ptr<TemperatureObserver>> children_;
public:
    void update(const std::string& sensor_id,
                double temperature,
                double threshold) override {
        for (const auto& child : children_) {
            child->update(sensor_id, temperature, threshold);
        }
    }
};
```

#### 9. Testing
1. **Mock Objects**:
```cpp
class MockObserver : public TemperatureObserver {
public:
    MOCK_METHOD(void, update,
        (const std::string&, double, double), (override));
    MOCK_METHOD(std::string, getObserverType, (), (const, override));
};
```

2. **Unit Tests**:
```cpp
TEST(TemperatureMonitorTest, NotificationTest) {
    auto monitor = std::make_shared<TemperatureMonitor>();
    auto mock_observer = std::make_shared<MockObserver>();
    
    EXPECT_CALL(*mock_observer, update("motor", 90.0, 80.0))
        .Times(1);
        
    monitor->attach(mock_observer);
    monitor->notify("motor", 90.0, 80.0);
}
```

3. **Integration Tests**:
```cpp
TEST(TemperatureSystemTest, FullSystemTest) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<TemperatureMonitoringNode>();
    
    // Publish test temperature data
    auto temp_pub = node->create_publisher<sensor_msgs::msg::Temperature>(
        "motor_temperature", 10);
        
    auto msg = sensor_msgs::msg::Temperature();
    msg.temperature = 85.0;
    temp_pub->publish(msg);
    
    // Verify alerts and speed changes
    // ...
    
    rclcpp::shutdown();
}
```

#### 10. Kết luận
Observer Pattern là một mẫu thiết kế quan trọng trong ROS2 và robotics, đặc biệt trong các hệ thống theo dõi và phản ứng thời gian thực. Pattern này mang lại nhiều lợi ích thiết thực:

1. **Tính linh hoạt cao**:
   - Dễ dàng thêm/xóa các observers mới mà không ảnh hưởng đến code hiện có
   - Có thể tái sử dụng observers cho nhiều subjects khác nhau
   - Hỗ trợ tốt việc mở rộng hệ thống

2. **Phù hợp với robotics**:
   - Xử lý tốt các tình huống real-time monitoring
   - Tích hợp tự nhiên với hệ thống publish/subscribe của ROS2
   - Đáp ứng nhanh với các thay đổi trạng thái của robot

3. **Dễ bảo trì và test**:
   - Code được tổ chức rõ ràng, dễ hiểu
   - Có thể test riêng từng observer
   - Dễ dàng debug và xử lý lỗi

4. **Hiệu quả trong thực tế**:
   - Giảm thiểu việc polling không cần thiết
   - Tối ưu hóa tài nguyên hệ thống
   - Xử lý đồng thời nhiều observers một cách hiệu quả

Trong ví dụ về hệ thống giám sát nhiệt độ, chúng ta đã thấy Observer Pattern giúp xây dựng một hệ thống mạnh mẽ, có khả năng mở rộng và dễ bảo trì. Pattern này là lựa chọn tốt cho các hệ thống robotics cần theo dõi và phản ứng với các thay đổi trạng thái một cách real-time. 