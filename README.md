# DESIGN PATTERNS TRONG ROBOTICS

## Giới thiệu

Design Patterns là những giải pháp thiết kế phần mềm được tái sử dụng cho các vấn đề thường gặp trong lập trình. Trong ngữ cảnh ROS2 và robotics, các pattern này trở nên đặc biệt quan trọng vì:

1. **Quản lý độ phức tạp**: Hệ thống robot hiện đại cực kỳ phức tạp với nhiều sensors, actuators, algorithms
2. **Yêu cầu thời gian thực**: Robotics cần performance cao và response time thấp
3. **Tính an toàn quan trọng**: Lỗi trong hệ thống robot có thể gây nguy hiểm
4. **Tính module hóa**: Cần thiết kế modular để dễ maintain và extend
5. **Khả năng tái sử dụng**: Code robotics thường được tái sử dụng cho nhiều robot khác nhau
6. **Hệ thống phân tán**: ROS2 là distributed system với nhiều nodes communication

## Lợi ích của Design Patterns trong ROS2

- **Tái sử dụng code**: Tái sử dụng solutions đã được verify
- **Bảo trì dễ dàng**: Code structure rõ ràng, dễ maintain
- **Khả năng mở rộng**: Dễ dàng scale từ single robot đến robot fleets
- **Kiểm thử**: Pattern chuẩn hóa giúp testing dễ dàng hơn
- **Tài liệu**: Common vocabulary cho team development
- **Hiệu năng**: Optimized solutions cho common problems

## Các nhóm Design Pattern

### 1. Creational Patterns
Các pattern liên quan đến cơ chế tạo đối tượng:

- **[Abstract Factory](./patterns/creational/abstract_factory.md)**: Tạo families của related objects
- **[Builder](./patterns/creational/builder.md)**: Xây dựng complex objects step-by-step
- **[Factory](./patterns/creational/factory.md)**: Tạo objects mà không specify exact class
- **[Prototype](./patterns/creational/prototype.md)**: Clone objects thay vì tạo mới
- **[Singleton](./patterns/creational/singleton.md)**: Ensure chỉ có một instance của class

### 2. Structural Patterns
Các pattern liên quan đến cấu trúc và tổ chức đối tượng:

- **[Adapter](./patterns/structural/adapter.md)**: Interface compatibility giữa incompatible classes
- **[Bridge](./patterns/structural/bridge.md)**: Tách abstraction khỏi implementation
- **[Composite](./patterns/structural/composite.md)**: Tổ chức đối tượng theo cấu trúc cây
- **[Decorator](./patterns/structural/decorator.md)**: Thêm chức năng cho đối tượng linh hoạt
- **[Facade](./patterns/structural/facade.md)**: Giao diện đơn giản cho hệ thống phức tạp
- **[Flyweight](./patterns/structural/flyweight.md)**: Tối ưu bộ nhớ bằng chia sẻ trạng thái
- **[Proxy](./patterns/structural/proxy.md)**: Kiểm soát truy cập đối tượng qua proxy

### 3. Behavioral Patterns
Các pattern liên quan đến tương tác và giao tiếp giữa đối tượng:

- **[Chain of Responsibility](./patterns/behavioral/chain_of_responsibility.md)**: Xử lý requests theo chuỗi
- **[Command](./patterns/behavioral/command.md)**: Đóng gói requests thành objects
- **[Interpreter](./patterns/behavioral/interpreter.md)**: Định nghĩa ngữ pháp cho ngôn ngữ
- **[Iterator](./patterns/behavioral/iterator.md)**: Truy cập tuần tự các phần tử
- **[Mediator](./patterns/behavioral/mediator.md)**: Quản lý giao tiếp giữa objects
- **[Memento](./patterns/behavioral/memento.md)**: Lưu và khôi phục trạng thái
- **[Observer](./patterns/behavioral/observer.md)**: Thông báo khi có thay đổi trạng thái
- **[State](./patterns/behavioral/state.md)**: Thay đổi hành vi theo trạng thái
- **[Strategy](./patterns/behavioral/strategy.md)**: Đóng gói và hoán đổi thuật toán
- **[Template Method](./patterns/behavioral/template_method.md)**: Định nghĩa khung thuật toán
- **[Visitor](./patterns/behavioral/visitor.md)**: Thêm thao tác mới cho cấu trúc object

## Áp dụng trong ROS2

### 1. Kiến trúc Node
ROS2 nodes tự nhiên áp dụng nhiều patterns:
- Publishers/Subscribers: Observer Pattern
- Action servers: Command Pattern
- Plugin systems: Strategy Pattern

### 2. Mô hình giao tiếp
- **Topics**: Publish/Subscribe (Observer pattern)
- **Services**: Request/Response (Command pattern)
- **Actions**: Long-running tasks (State pattern)
- **Parameters**: Configuration (Strategy pattern)

### 3. Quản lý vòng đời
- **Managed Nodes**: State pattern
- **Component Lifecycle**: Factory và Builder patterns
- **Resource Management**: RAII và Singleton patterns

## Ví dụ thực tế

### 1. Sensor Fusion
```cpp
// Strategy pattern cho multiple sensor fusion algorithms
class SensorFusion {
    std::unique_ptr<FusionStrategy> strategy_;
public:
    void setStrategy(std::unique_ptr<FusionStrategy> strategy) {
        strategy_ = std::move(strategy);
    }
    
    SensorData fuse(const std::vector<SensorData>& inputs) {
        return strategy_->fuse(inputs);
    }
};
```

### 2. Path Planning
```cpp
// Factory pattern cho different planners
class PlannerFactory {
public:
    static std::unique_ptr<PathPlanner> createPlanner(const std::string& type) {
        if (type == "A*") return std::make_unique<AStarPlanner>();
        if (type == "RRT") return std::make_unique<RRTPlanner>();
        if (type == "DWA") return std::make_unique<DWAPlanner>();
        return nullptr;
    }
};
```

### 3. Robot Fleet Management
```cpp
// Observer pattern cho fleet coordination
class FleetManager : public Subject {
    std::vector<std::unique_ptr<Robot>> fleet_;
public:
    void addRobot(std::unique_ptr<Robot> robot) {
        fleet_.push_back(std::move(robot));
        notifyObservers(RobotAddedEvent{robot.get()});
    }
};
```

### 4. Hardware Abstraction
```cpp
// Bridge pattern cho hardware independence
class RobotHardware {
    std::unique_ptr<HardwareImplementation> impl_;
public:
    RobotHardware(std::unique_ptr<HardwareImplementation> impl) 
        : impl_(std::move(impl)) {}
    
    void moveForward(double speed) {
        impl_->setMotorSpeeds(speed, speed);
    }
};
```

## Hiệu năng và Tối ưu

### 1. Quản lý bộ nhớ
- **RAII**: Resource Acquisition Is Initialization
- **Smart Pointers**: Quản lý bộ nhớ tự động
- **Object Pooling**: Tái sử dụng đối tượng tốn kém

### 2. Ràng buộc thời gian thực
- **Lock-free Programming**: Tránh blocking operations
- **Memory Pre-allocation**: Tránh cấp phát động trong vòng lặp
- **Priority Scheduling**: Sử dụng độ ưu tiên thread phù hợp

### 3. Tối ưu mạng
- **Message Batching**: Gộp nhiều messages
- **Compression**: Giảm băng thông mạng
- **Local Processing**: Giảm thiểu giao tiếp mạng

## Hướng dẫn thực hành

### 1. Chọn Pattern phù hợp
- Chỉ dùng pattern khi cần thiết
- Cân nhắc các giải pháp thay thế
- Đánh giá tác động hiệu năng

### 2. Tích hợp với ROS2
- Tận dụng patterns có sẵn trong ROS2
- Thiết kế theo kiến trúc component
- Sử dụng hệ thống parameter

### 3. Chiến lược kiểm thử
- Unit testing với mock objects
- Integration tests cho tương tác patterns
- Performance tests cho yêu cầu thời gian thực

### 4. Tài liệu hóa
- Ghi rõ lý do chọn pattern
- Cung cấp ví dụ cụ thể
- Liệt kê các phương án thay thế

## Công cụ và tiện ích

### 1. Build Tools
```bash
# Tạo PDF từ markdown
./convert_to_pdf.sh
```

### 2. Cấu hình
- **YAML configs**: Cấu hình patterns
- **Launch files**: Cấu hình khởi chạy ROS2
- **Parameter files**: Cấu hình runtime

## Getting Started

1. **Đọc introduction**: Hiểu concepts cơ bản
2. **Chọn pattern**: Based on use case
3. **Study examples**: Xem implementation details
4. **Practice**: Implement trong project riêng
5. **Optimize**: Tune for specific requirements

## Contributing

Để contribute vào documentation này:

1. **Fork repository**
2. **Add new patterns**: Follow existing structure
3. **Update examples**: Keep ROS2 examples current
4. **Test code**: Ensure examples compile và run
5. **Submit PR**: Include clear description

## Resources

### ROS2 Documentation
- [ROS2 Design Principles](https://design.ros2.org/)
- [ROS2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### Design Patterns Books
- "Design Patterns" by Gang of Four
- "Effective C++" by Scott Meyers
- "Real-Time C++" by Christopher Kormanyos

### Robotics Resources
- [Navigation2](https://docs.nav2.org/)
- [MoveIt2](https://moveit.ros.org/)
- [Gazebo](https://gazebosim.org/)


---

**Note**: Các examples trong documentation này được test với ROS2 Humble và C++17. Update có thể cần thiết cho newer versions.

## Architectural Patterns trong ROS2

### 1. Node-based Architecture
ROS2 được xây dựng trên kiến trúc node-based, mỗi node là một đơn vị độc lập có thể:

- Xử lý một chức năng cụ thể
- Giao tiếp với các node khác
- Quản lý vòng đời riêng

#### Ví dụ Node Architecture
```cpp
class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("sensor_node") {
        // Publishers
        data_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "scan_data", 10);
        
        // Services
        calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
            "calibrate_sensor",
            std::bind(&SensorNode::handleCalibration, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer-based processing
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorNode::publishData, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr data_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### 2. Communication Patterns

#### a. Publisher-Subscriber (Observer Pattern)
- **Mục đích**: Truyền dữ liệu một chiều, một-nhiều
- **Use Cases**: 
  - Sensor data streaming
  - Robot state broadcasting
  - Event notifications

```cpp
// Publisher
auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

// Subscriber
auto subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    [](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process laser scan data
    }
);
```

#### b. Client-Server (Request-Response Pattern)
- **Mục đích**: Giao tiếp hai chiều, yêu cầu-phản hồi
- **Use Cases**:
  - Robot calibration
  - Path planning requests
  - System configuration

```cpp
// Service Server
auto server = node->create_service<nav_msgs::srv::GetPlan>(
    "plan_path",
    [](const nav_msgs::srv::GetPlan::Request::SharedPtr req,
       nav_msgs::srv::GetPlan::Response::SharedPtr res) {
        // Generate and return path
    }
);

// Service Client
auto client = node->create_client<nav_msgs::srv::GetPlan>("plan_path");
auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
auto future = client->async_send_request(request);
```

#### c. Action Pattern (Long-running Tasks)
- **Mục đích**: Xử lý tác vụ dài hạn với feedback
- **Use Cases**:
  - Navigation tasks
  - Manipulation sequences
  - Complex behaviors

```cpp
// Action Server
class NavigateActionServer : public rclcpp::Node {
public:
    using NavigateAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateAction>;

    explicit NavigateActionServer()
        : Node("navigate_action_server")
    {
        action_server_ = rclcpp_action::create_server<NavigateAction>(
            this, "navigate_to_pose",
            std::bind(&NavigateActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigateActionServer::handleCancel, this, std::placeholders::_1),
            std::bind(&NavigateActionServer::handleAccepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<NavigateAction>::SharedPtr action_server_;
};
```

### 3. Component Architecture
ROS2 hỗ trợ component-based development để tăng tính tái sử dụng và module hóa:

#### a. Lifecycle Components
- **Mục đích**: Quản lý vòng đời node theo state machine
- **States**: Unconfigured → Inactive → Active → Finalized
```cpp
class MotorController : public rclcpp_lifecycle::LifecycleNode {
public:
    MotorController() : LifecycleNode("motor_controller") {
        // Initialize hardware interface
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        // Configure hardware
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        // Start motors
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        // Stop motors
        return CallbackReturn::SUCCESS;
    }
};
```

#### b. Composable Nodes
- **Mục đích**: Kết hợp nhiều nodes trong cùng một process
- **Lợi ích**: Giảm overhead giao tiếp, tối ưu hiệu năng
```cpp
class SensorProcessor : public rclcpp::Node {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SensorProcessor)
    
    SensorProcessor(const rclcpp::NodeOptions & options)
        : Node("sensor_processor", options)
    {
        // Node implementation
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SensorProcessor)
```

### 4. Plugin Architecture
ROS2 sử dụng plugin system để mở rộng chức năng một cách linh hoạt:

#### a. Plugin Interfaces
```cpp
class PlannerInterface : public rclcpp::plugin::PluginInterface {
public:
    virtual Path planPath(const Point& start, const Point& goal) = 0;
    virtual ~PlannerInterface() = default;
};

class AStarPlanner : public PlannerInterface {
public:
    Path planPath(const Point& start, const Point& goal) override {
        // A* implementation
    }
};
```

#### b. Plugin Loading
```cpp
#include "pluginlib/class_loader.hpp"

pluginlib::ClassLoader<PlannerInterface> planner_loader(
    "nav2_core", "nav2_core::PlannerInterface");
std::shared_ptr<PlannerInterface> planner = 
    planner_loader.createSharedInstance("nav2_navfn_planner/NavfnPlanner");
```

### 5. Parameter Management
ROS2 cung cấp hệ thống parameter để cấu hình node một cách linh hoạt:

```cpp
class ConfigurableNode : public rclcpp::Node {
public:
    ConfigurableNode() : Node("configurable_node") {
        // Declare parameters
        declare_parameter("max_speed", 1.0);
        declare_parameter("control_rate", 10);
        
        // Parameter callback
        auto param_callback = 
            [this](const std::vector<rclcpp::Parameter> & params) {
                for (const auto & param : params) {
                    if (param.get_name() == "max_speed") {
                        max_speed_ = param.as_double();
                    }
                }
                return rcl_interfaces::msg::SetParametersResult{};
            };
        
        param_callback_handle_ = 
            add_on_set_parameters_callback(param_callback);
    }

private:
    double max_speed_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
```

## Best Practices
1. **SOLID Principles trong ROS2**
2. **Clean Architecture**
3. **Error Handling**
4. **Testing Strategies**
5. **Documentation**

## Lưu ý khi sử dụng Design Patterns
1. Không lạm dụng patterns
2. Giữ code đơn giản khi có thể
3. Cân nhắc context cụ thể của ứng dụng
4. Ưu tiên khả năng bảo trì và đọc hiểu
