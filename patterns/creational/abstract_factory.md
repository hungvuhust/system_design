## ABSTRACT FACTORY PATTERN TRONG ROS2

### 1. Giới thiệu đơn giản
Abstract Factory Pattern là một mẫu thiết kế khởi tạo cho phép tạo các họ đối tượng liên quan mà không cần chỉ định các lớp cụ thể của chúng. Trong ROS2, pattern này đặc biệt hữu ích khi:

- Tạo các bộ controllers hoàn chỉnh (position, velocity, effort)
- Khởi tạo các hệ thống sensor đa dạng (vision, lidar, IMU)
- Tạo các bộ message handlers và transformers
- Quản lý các subsystems phức tạp

### 2. Định nghĩa chi tiết
Abstract Factory Pattern cung cấp một interface để tạo các họ đối tượng liên quan hoặc phụ thuộc mà không cần chỉ định các lớp cụ thể của chúng.

#### Các thành phần chính:
1. **Abstract Factory**:
   - Interface chung cho việc tạo products
   - Định nghĩa các factory methods

2. **Concrete Factories**:
   - Implements abstract factory
   - Tạo các concrete products

3. **Abstract Products**:
   - Interface cho một loại product
   - Định nghĩa các operations chuẩn

4. **Concrete Products**:
   - Implements abstract products
   - Các sản phẩm cụ thể

### 3. Ví dụ thực tế trong ROS2
```cpp
// 1. Abstract Products
class MotionController {
public:
    virtual ~MotionController() = default;
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    virtual bool setTarget(const geometry_msgs::msg::Pose& target) = 0;
    virtual geometry_msgs::msg::Twist computeCommand() = 0;
    virtual std::string getControllerType() const = 0;
};

class ObstacleDetector {
public:
    virtual ~ObstacleDetector() = default;
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    virtual bool detectObstacles() = 0;
    virtual std::vector<geometry_msgs::msg::Point> getObstaclePositions() = 0;
    virtual std::string getDetectorType() const = 0;
};

class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    virtual bool planPath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal) = 0;
    virtual std::vector<geometry_msgs::msg::PoseStamped> getPath() = 0;
    virtual std::string getPlannerType() const = 0;
};

// 2. Concrete Products - Indoor Navigation System
class IndoorMotionController : public MotionController {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Load indoor-specific parameters
            auto params = node_->get_parameters("indoor_motion");
            max_vel_x_ = params[0].as_double();
            max_vel_theta_ = params[1].as_double();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Indoor Motion Controller initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Indoor Motion Controller: %s", e.what());
            return false;
        }
    }
    
    bool setTarget(const geometry_msgs::msg::Pose& target) override {
        if (!initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "Controller not initialized");
            return false;
        }
        current_target_ = target;
        return true;
    }
    
    geometry_msgs::msg::Twist computeCommand() override {
        geometry_msgs::msg::Twist cmd;
        // Indoor-specific motion control logic
        // Consider narrow spaces, doors, etc.
        return cmd;
    }
    
    std::string getControllerType() const override {
        return "INDOOR_MOTION_CONTROLLER";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double max_vel_x_;
    double max_vel_theta_;
    geometry_msgs::msg::Pose current_target_;
};

class IndoorObstacleDetector : public ObstacleDetector {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Setup indoor obstacle detection parameters
            auto params = node_->get_parameters("indoor_detection");
            min_obstacle_height_ = params[0].as_double();
            max_obstacle_distance_ = params[1].as_double();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Indoor Obstacle Detector initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Indoor Obstacle Detector: %s", e.what());
            return false;
        }
    }
    
    bool detectObstacles() override {
        if (!initialized_) {
            return false;
        }
        // Indoor-specific obstacle detection
        // Focus on walls, furniture, doors
        return true;
    }
    
    std::vector<geometry_msgs::msg::Point> getObstaclePositions() override {
        std::vector<geometry_msgs::msg::Point> obstacles;
        // Return detected indoor obstacles
        return obstacles;
    }
    
    std::string getDetectorType() const override {
        return "INDOOR_OBSTACLE_DETECTOR";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double min_obstacle_height_;
    double max_obstacle_distance_;
};

class IndoorPathPlanner : public PathPlanner {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Setup indoor path planning parameters
            auto params = node_->get_parameters("indoor_planning");
            corridor_width_ = params[0].as_double();
            door_width_ = params[1].as_double();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Indoor Path Planner initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Indoor Path Planner: %s", e.what());
            return false;
        }
    }
    
    bool planPath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal) override {
        if (!initialized_) {
            return false;
        }
        // Indoor-specific path planning
        // Consider room layout, corridors, doors
        return true;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> getPath() override {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        // Return planned indoor path
        return path;
    }
    
    std::string getPlannerType() const override {
        return "INDOOR_PATH_PLANNER";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double corridor_width_;
    double door_width_;
};

// 3. Concrete Products - Outdoor Navigation System
class OutdoorMotionController : public MotionController {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Load outdoor-specific parameters
            auto params = node_->get_parameters("outdoor_motion");
            max_vel_x_ = params[0].as_double();
            max_vel_theta_ = params[1].as_double();
            terrain_type_ = params[2].as_string();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Outdoor Motion Controller initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Outdoor Motion Controller: %s", e.what());
            return false;
        }
    }
    
    bool setTarget(const geometry_msgs::msg::Pose& target) override {
        if (!initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "Controller not initialized");
            return false;
        }
        current_target_ = target;
        return true;
    }
    
    geometry_msgs::msg::Twist computeCommand() override {
        geometry_msgs::msg::Twist cmd;
        // Outdoor-specific motion control logic
        // Consider terrain, weather conditions
        return cmd;
    }
    
    std::string getControllerType() const override {
        return "OUTDOOR_MOTION_CONTROLLER";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double max_vel_x_;
    double max_vel_theta_;
    std::string terrain_type_;
    geometry_msgs::msg::Pose current_target_;
};

class OutdoorObstacleDetector : public ObstacleDetector {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Setup outdoor obstacle detection parameters
            auto params = node_->get_parameters("outdoor_detection");
            min_obstacle_size_ = params[0].as_double();
            weather_condition_ = params[1].as_string();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Outdoor Obstacle Detector initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Outdoor Obstacle Detector: %s", e.what());
            return false;
        }
    }
    
    bool detectObstacles() override {
        if (!initialized_) {
            return false;
        }
        // Outdoor-specific obstacle detection
        // Focus on terrain features, dynamic obstacles
        return true;
    }
    
    std::vector<geometry_msgs::msg::Point> getObstaclePositions() override {
        std::vector<geometry_msgs::msg::Point> obstacles;
        // Return detected outdoor obstacles
        return obstacles;
    }
    
    std::string getDetectorType() const override {
        return "OUTDOOR_OBSTACLE_DETECTOR";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double min_obstacle_size_;
    std::string weather_condition_;
};

class OutdoorPathPlanner : public PathPlanner {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            // Setup outdoor path planning parameters
            auto params = node_->get_parameters("outdoor_planning");
            terrain_resolution_ = params[0].as_double();
            gps_accuracy_ = params[1].as_double();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Outdoor Path Planner initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize Outdoor Path Planner: %s", e.what());
            return false;
        }
    }
    
    bool planPath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal) override {
        if (!initialized_) {
            return false;
        }
        // Outdoor-specific path planning
        // Consider terrain, GPS waypoints
        return true;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> getPath() override {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        // Return planned outdoor path
        return path;
    }
    
    std::string getPlannerType() const override {
        return "OUTDOOR_PATH_PLANNER";
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    double terrain_resolution_;
    double gps_accuracy_;
};

// 4. Abstract Factory
class NavigationSystemFactory {
public:
    virtual ~NavigationSystemFactory() = default;
    virtual std::unique_ptr<MotionController> createMotionController() = 0;
    virtual std::unique_ptr<ObstacleDetector> createObstacleDetector() = 0;
    virtual std::unique_ptr<PathPlanner> createPathPlanner() = 0;
};

// 5. Concrete Factories
class IndoorNavigationFactory : public NavigationSystemFactory {
public:
    std::unique_ptr<MotionController> createMotionController() override {
        return std::make_unique<IndoorMotionController>();
    }
    
    std::unique_ptr<ObstacleDetector> createObstacleDetector() override {
        return std::make_unique<IndoorObstacleDetector>();
    }
    
    std::unique_ptr<PathPlanner> createPathPlanner() override {
        return std::make_unique<IndoorPathPlanner>();
    }
};

class OutdoorNavigationFactory : public NavigationSystemFactory {
public:
    std::unique_ptr<MotionController> createMotionController() override {
        return std::make_unique<OutdoorMotionController>();
    }
    
    std::unique_ptr<ObstacleDetector> createObstacleDetector() override {
        return std::make_unique<OutdoorObstacleDetector>();
    }
    
    std::unique_ptr<PathPlanner> createPathPlanner() override {
        return std::make_unique<OutdoorPathPlanner>();
    }
};

// 6. Usage Example
class NavigationSystem {
public:
    NavigationSystem(
        const rclcpp::Node::SharedPtr& node,
        std::unique_ptr<NavigationSystemFactory> factory)
        : node_(node), factory_(std::move(factory)) {
        
        // Initialize all components
        initializeComponents();
        
        // Setup ROS2 communication
        setupCommunication();
    }
    
    bool navigate(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal) {
        try {
            // 1. Detect obstacles
            if (!obstacle_detector_->detectObstacles()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to detect obstacles");
                return false;
            }
            
            // 2. Plan path
            if (!path_planner_->planPath(start, goal)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to plan path");
                return false;
            }
            
            // 3. Follow path
            auto path = path_planner_->getPath();
            for (const auto& pose : path) {
                if (!motion_controller_->setTarget(pose.pose)) {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set target");
                    return false;
                }
                
                // Execute motion
                auto cmd_vel = motion_controller_->computeCommand();
                cmd_vel_pub_->publish(cmd_vel);
            }
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Navigation error: %s", e.what());
            return false;
        }
    }

private:
    void initializeComponents() {
        // Create components using factory
        motion_controller_ = factory_->createMotionController();
        obstacle_detector_ = factory_->createObstacleDetector();
        path_planner_ = factory_->createPathPlanner();
        
        // Initialize components
        if (!motion_controller_->initialize(node_)) {
            throw std::runtime_error("Failed to initialize motion controller");
        }
        if (!obstacle_detector_->initialize(node_)) {
            throw std::runtime_error("Failed to initialize obstacle detector");
        }
        if (!path_planner_->initialize(node_)) {
            throw std::runtime_error("Failed to initialize path planner");
        }
    }
    
    void setupCommunication() {
        // Create publishers and subscribers
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
            
        goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 1,
            std::bind(&NavigationSystem::goalCallback, this, std::placeholders::_1));
    }
    
    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Get current pose
        geometry_msgs::msg::Pose current_pose;
        // TODO: Get current pose from tf or odometry
        
        // Navigate to goal
        if (!navigate(current_pose, msg->pose)) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation failed");
        }
    }
    
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<NavigationSystemFactory> factory_;
    std::unique_ptr<MotionController> motion_controller_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    std::unique_ptr<PathPlanner> path_planner_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

// 7. Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("navigation_node");
    
    // Get environment type from parameter
    std::string env_type = node->declare_parameter("environment_type", "indoor");
    
    // Create appropriate factory
    std::unique_ptr<NavigationSystemFactory> factory;
    if (env_type == "indoor") {
        factory = std::make_unique<IndoorNavigationFactory>();
    } else if (env_type == "outdoor") {
        factory = std::make_unique<OutdoorNavigationFactory>();
    } else {
        RCLCPP_ERROR(node->get_logger(), 
            "Unknown environment type: %s", env_type.c_str());
        return 1;
    }
    
    // Create navigation system
    auto navigation = std::make_shared<NavigationSystem>(node, std::move(factory));
    
    // Spin node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 4. Giải thích chi tiết cách hoạt động
1. **Abstract Factory**:
   - NavigationSystemFactory định nghĩa interface
   - Tạo các components liên quan
   - Đảm bảo tính tương thích

2. **Concrete Factories**:
   - IndoorNavigationFactory cho môi trường trong nhà
   - OutdoorNavigationFactory cho môi trường ngoài trời
   - Tạo các components phù hợp

3. **Abstract Products**:
   - MotionController, ObstacleDetector, PathPlanner
   - Định nghĩa interface chung
   - Tính đa hình thông qua virtual methods

### 5. Ưu điểm trong ROS2
1. **System Consistency**:
   - Components tương thích
   - Cấu hình nhất quán
   - Error handling đồng bộ

2. **Flexibility**:
   - Dễ thêm môi trường mới
   - Runtime configuration
   - Component swapping

3. **Maintainability**:
   - Code organization rõ ràng
   - Separation of concerns
   - Dễ test và debug

### 6. Các trường hợp sử dụng trong ROS2
1. **Navigation Systems**:
   - Indoor/Outdoor navigation
   - Multi-robot systems
   - Hybrid environments

2. **Sensor Systems**:
   - Different sensor suites
   - Environment-specific processing
   - Sensor fusion

3. **Control Systems**:
   - Different control strategies
   - Environment adaptation
   - Multi-mode operation

### 7. Best Practices trong ROS2
1. **Error Handling**:
```cpp
try {
    auto system = createNavigationSystem(env_type);
    if (!system->initialize()) {
        RCLCPP_ERROR(logger, "System initialization failed");
        return;
    }
} catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "System creation error: %s", e.what());
}
```

2. **Configuration Management**:
```cpp
class ConfigurableFactory : public NavigationSystemFactory {
    void configure(const YAML::Node& config) {
        // Configure factory from YAML
        loadConfiguration(config);
    }
};
```

3. **Resource Management**:
```cpp
class SafeNavigation {
    std::unique_ptr<NavigationSystem> system_;
public:
    ~SafeNavigation() {
        if (system_) {
            system_->shutdown();
        }
    }
};
```

### 8. Mở rộng và tùy chỉnh
1. **Environment Detection**:
```cpp
class AutoNavigationFactory : public NavigationSystemFactory {
    std::unique_ptr<NavigationSystemFactory> detectEnvironment() {
        // Detect environment type and return appropriate factory
        if (isIndoorEnvironment()) {
            return std::make_unique<IndoorNavigationFactory>();
        }
        return std::make_unique<OutdoorNavigationFactory>();
    }
};
```

2. **Hybrid Systems**:
```cpp
class HybridNavigationFactory : public NavigationSystemFactory {
    std::unique_ptr<MotionController> createMotionController() override {
        // Create controller that can handle both environments
        return std::make_unique<HybridMotionController>();
    }
};
```

3. **Plugin System**:
```cpp
class PluginNavigationFactory : public NavigationSystemFactory {
    std::unique_ptr<MotionController> loadControllerPlugin(
        const std::string& name) {
        // Load controller plugin dynamically
        return loadPlugin<MotionController>(name);
    }
};
```

### 9. Testing
1. **Mock Objects**:
```cpp
class MockNavigationFactory : public NavigationSystemFactory {
public:
    MOCK_METHOD(std::unique_ptr<MotionController>, 
                createMotionController, (), (override));
    MOCK_METHOD(std::unique_ptr<ObstacleDetector>, 
                createObstacleDetector, (), (override));
    MOCK_METHOD(std::unique_ptr<PathPlanner>, 
                createPathPlanner, (), (override));
};
```

2. **Factory Tests**:
```cpp
TEST(NavigationTest, IndoorFactoryTest) {
    auto factory = std::make_unique<IndoorNavigationFactory>();
    auto controller = factory->createMotionController();
    EXPECT_NE(controller, nullptr);
    EXPECT_EQ(controller->getControllerType(), "INDOOR_MOTION_CONTROLLER");
}
```

3. **Integration Tests**:
```cpp
TEST(NavigationSystemTest, FullSystemTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto factory = std::make_unique<IndoorNavigationFactory>();
    auto navigation = std::make_shared<NavigationSystem>(
        node, std::move(factory));
    
    geometry_msgs::msg::Pose start, goal;
    EXPECT_TRUE(navigation->navigate(start, goal));
}
```

### 10. Kết luận
Abstract Factory Pattern là một mẫu thiết kế quan trọng trong ROS2, đặc biệt hữu ích cho việc tạo các hệ thống phức tạp với nhiều components liên quan. Pattern này mang lại nhiều lợi ích:

1. **System Consistency**:
   - Đảm bảo tính tương thích giữa các components
   - Cấu hình nhất quán cho từng môi trường
   - Error handling đồng bộ

2. **Flexibility và Extensibility**:
   - Dễ dàng thêm môi trường mới
   - Runtime configuration
   - Component swapping linh hoạt

3. **Code Organization**:
   - Separation of concerns rõ ràng
   - Interface standards
   - Dễ maintain và test

4. **Resource Management**:
   - Clean initialization và cleanup
   - Safe resource handling
   - Memory management hiệu quả

Trong ví dụ về navigation system, chúng ta đã thấy Abstract Factory Pattern giúp xây dựng một hệ thống navigation linh hoạt và mạnh mẽ, có thể dễ dàng chuyển đổi giữa môi trường trong nhà và ngoài trời. Pattern này là lựa chọn tốt cho các hệ thống ROS2 cần quản lý nhiều components liên quan và có thể hoạt động trong các môi trường khác nhau. 