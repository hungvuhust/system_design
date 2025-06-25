## STRATEGY PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Strategy Pattern cho phép định nghĩa một nhóm các thuật toán, đóng gói từng thuật toán và làm cho chúng có thể hoán đổi cho nhau. Trong ROS2, pattern này đặc biệt hữu ích cho:

- Các thuật toán planning khác nhau
- Các chiến lược điều khiển robot
- Các phương pháp xử lý sensor data
- Các chiến lược navigation

#### 2. Định nghĩa chi tiết
Strategy Pattern cho phép thay đổi thuật toán độc lập với code sử dụng thuật toán đó. Pattern này bao gồm:

#### Các thành phần chính:
1. **Strategy Interface**:
   - Định nghĩa interface chung
   - Các phương thức chuẩn

2. **Concrete Strategies**:
   - Implements các thuật toán cụ thể
   - Tuân theo strategy interface

3. **Context**:
   - Sử dụng strategy
   - Có thể thay đổi strategy runtime

#### 3. Ví dụ thực tế trong ROS2
```cpp
// 1. Strategy Interface
class NavigationStrategy {
public:
    virtual ~NavigationStrategy() = default;
    
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    virtual geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const nav_msgs::msg::OccupancyGrid& map) = 0;
    virtual bool isGoalReached(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose) = 0;
    virtual std::string getName() const = 0;
};

// 2. Concrete Strategies
class DWAPlanner : public NavigationStrategy {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            
            // Load DWA parameters
            auto params = node_->get_parameters("dwa");
            max_vel_x_ = params[0].as_double();
            max_vel_theta_ = params[1].as_double();
            acc_lim_x_ = params[2].as_double();
            acc_lim_theta_ = params[3].as_double();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "DWA Planner initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize DWA Planner: %s", e.what());
            return false;
        }
    }
    
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const nav_msgs::msg::OccupancyGrid& map) override {
        
        if (!initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "DWA Planner not initialized");
            return geometry_msgs::msg::Twist();
        }
        
        try {
            // 1. Generate velocity samples
            auto samples = generateVelocitySamples();
            
            // 2. Score each trajectory
            auto scored_trajectories = scoreTrajectories(
                samples, current_pose, goal_pose, map);
            
            // 3. Select best trajectory
            auto best_trajectory = selectBestTrajectory(scored_trajectories);
            
            // 4. Return velocity command
            return trajectoryToTwist(best_trajectory);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error computing velocity: %s", e.what());
            return geometry_msgs::msg::Twist();
        }
    }
    
    bool isGoalReached(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose) override {
        
        // Calculate distance to goal
        double dx = goal_pose.pose.position.x - current_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - current_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle difference
        double angle_diff = calculateAngleDifference(
            current_pose.pose.orientation,
            goal_pose.pose.orientation);
        
        return distance < position_tolerance_ && 
               std::abs(angle_diff) < angle_tolerance_;
    }
    
    std::string getName() const override {
        return "DWA_PLANNER";
    }

private:
    std::vector<Trajectory> generateVelocitySamples() {
        std::vector<Trajectory> samples;
        
        // Generate velocity samples within constraints
        for (double vx = -max_vel_x_; vx <= max_vel_x_; vx += vel_x_step_) {
            for (double vth = -max_vel_theta_; vth <= max_vel_theta_; 
                 vth += vel_theta_step_) {
                // Create trajectory from velocity pair
                samples.push_back(createTrajectory(vx, vth));
            }
        }
        
        return samples;
    }
    
    std::vector<ScoredTrajectory> scoreTrajectories(
        const std::vector<Trajectory>& trajectories,
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const nav_msgs::msg::OccupancyGrid& map) {
        
        std::vector<ScoredTrajectory> scored_trajectories;
        
        for (const auto& trajectory : trajectories) {
            double obstacle_cost = calculateObstacleCost(trajectory, map);
            double goal_cost = calculateGoalCost(trajectory, goal_pose);
            double path_cost = calculatePathCost(trajectory);
            
            double total_cost = obstacle_weight_ * obstacle_cost +
                              goal_weight_ * goal_cost +
                              path_weight_ * path_cost;
                              
            scored_trajectories.push_back({trajectory, total_cost});
        }
        
        return scored_trajectories;
    }
    
    Trajectory selectBestTrajectory(
        const std::vector<ScoredTrajectory>& scored_trajectories) {
        
        auto best = std::min_element(
            scored_trajectories.begin(),
            scored_trajectories.end(),
            [](const auto& a, const auto& b) {
                return a.cost < b.cost;
            });
            
        return best->trajectory;
    }
    
    geometry_msgs::msg::Twist trajectoryToTwist(
        const Trajectory& trajectory) {
        
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = trajectory.vx;
        cmd_vel.angular.z = trajectory.vth;
        return cmd_vel;
    }
    
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    
    // DWA parameters
    double max_vel_x_;
    double max_vel_theta_;
    double acc_lim_x_;
    double acc_lim_theta_;
    double vel_x_step_ = 0.1;
    double vel_theta_step_ = 0.1;
    
    // Cost weights
    double obstacle_weight_ = 0.8;
    double goal_weight_ = 0.6;
    double path_weight_ = 0.4;
    
    // Goal tolerances
    double position_tolerance_ = 0.1;  // meters
    double angle_tolerance_ = 0.1;     // radians
};

class TebPlanner : public NavigationStrategy {
public:
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        try {
            node_ = node;
            
            // Load TEB parameters
            auto params = node_->get_parameters("teb");
            max_vel_x_ = params[0].as_double();
            max_vel_theta_ = params[1].as_double();
            optimization_steps_ = params[2].as_int();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "TEB Planner initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize TEB Planner: %s", e.what());
            return false;
        }
    }
    
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const nav_msgs::msg::OccupancyGrid& map) override {
        
        if (!initialized_) {
            RCLCPP_ERROR(node_->get_logger(), "TEB Planner not initialized");
            return geometry_msgs::msg::Twist();
        }
        
        try {
            // 1. Generate initial trajectory
            auto initial_trajectory = generateInitialTrajectory(
                current_pose, goal_pose);
            
            // 2. Optimize trajectory
            auto optimized_trajectory = optimizeTrajectory(
                initial_trajectory, map);
            
            // 3. Extract velocity command
            return extractVelocity(optimized_trajectory);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error computing velocity: %s", e.what());
            return geometry_msgs::msg::Twist();
        }
    }
    
    bool isGoalReached(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose) override {
        
        // Calculate distance to goal
        double dx = goal_pose.pose.position.x - current_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - current_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle difference
        double angle_diff = calculateAngleDifference(
            current_pose.pose.orientation,
            goal_pose.pose.orientation);
        
        return distance < position_tolerance_ && 
               std::abs(angle_diff) < angle_tolerance_;
    }
    
    std::string getName() const override {
        return "TEB_PLANNER";
    }

private:
    Trajectory generateInitialTrajectory(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {
        // Generate initial trajectory using simple interpolation
        return createInitialGuess(start, goal);
    }
    
    Trajectory optimizeTrajectory(
        const Trajectory& initial_trajectory,
        const nav_msgs::msg::OccupancyGrid& map) {
        
        Trajectory current_trajectory = initial_trajectory;
        
        // Optimize trajectory using TEB approach
        for (int i = 0; i < optimization_steps_; ++i) {
            // 1. Calculate cost terms
            auto costs = calculateCosts(current_trajectory, map);
            
            // 2. Build optimization problem
            auto problem = buildOptimizationProblem(
                current_trajectory, costs);
            
            // 3. Solve one iteration
            current_trajectory = solveIteration(problem);
        }
        
        return current_trajectory;
    }
    
    geometry_msgs::msg::Twist extractVelocity(
        const Trajectory& trajectory) {
        
        geometry_msgs::msg::Twist cmd_vel;
        
        // Extract velocity from first trajectory segment
        cmd_vel.linear.x = trajectory.segments[0].velocity.linear;
        cmd_vel.angular.z = trajectory.segments[0].velocity.angular;
        
        return cmd_vel;
    }
    
    rclcpp::Node::SharedPtr node_;
    bool initialized_ = false;
    
    // TEB parameters
    double max_vel_x_;
    double max_vel_theta_;
    int optimization_steps_;
    
    // Goal tolerances
    double position_tolerance_ = 0.1;  // meters
    double angle_tolerance_ = 0.1;     // radians
};

// 3. Context
class NavigationManager {
public:
    NavigationManager(const rclcpp::Node::SharedPtr& node)
        : node_(node) {
        // Initialize available strategies
        strategies_["DWA"] = std::make_unique<DWAPlanner>();
        strategies_["TEB"] = std::make_unique<TebPlanner>();
        
        // Get default strategy from parameter
        std::string default_strategy = 
            node_->get_parameter("default_strategy").as_string();
        
        // Set default strategy
        setStrategy(default_strategy);
        
        // Initialize subscribers
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10,
            std::bind(&NavigationManager::poseCallback, this, std::placeholders::_1));
            
        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 1,
            std::bind(&NavigationManager::mapCallback, this, std::placeholders::_1));
            
        goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 1,
            std::bind(&NavigationManager::goalCallback, this, std::placeholders::_1));
            
        // Initialize publisher
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
            
        // Create timer for control loop
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigationManager::controlLoop, this));
    }
    
    bool setStrategy(const std::string& strategy_name) {
        auto it = strategies_.find(strategy_name);
        if (it == strategies_.end()) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Unknown strategy: %s", strategy_name.c_str());
            return false;
        }
        
        current_strategy_ = it->second.get();
        return current_strategy_->initialize(node_);
    }
    
    void controlLoop() {
        if (!current_strategy_ || !current_pose_ || !goal_pose_ || !map_) {
            return;
        }
        
        try {
            // Check if goal is reached
            if (current_strategy_->isGoalReached(*current_pose_, *goal_pose_)) {
                RCLCPP_INFO(node_->get_logger(), "Goal reached!");
                publishZeroVelocity();
                return;
            }
            
            // Compute velocity command
            auto cmd_vel = current_strategy_->computeVelocity(
                *current_pose_, *goal_pose_, *map_);
            
            // Publish command
            cmd_vel_pub_->publish(cmd_vel);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error in control loop: %s", e.what());
            publishZeroVelocity();
        }
    }

private:
    void poseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg;
    }
    
    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
    }
    
    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = msg;
    }
    
    void publishZeroVelocity() {
        geometry_msgs::msg::Twist zero_vel;
        cmd_vel_pub_->publish(zero_vel);
    }
    
    rclcpp::Node::SharedPtr node_;
    NavigationStrategy* current_strategy_ = nullptr;
    std::map<std::string, std::unique_ptr<NavigationStrategy>> strategies_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Latest data
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

// 4. Usage Example
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("navigation_node");
    
    // Declare parameters
    node->declare_parameter("default_strategy", "DWA");
    node->declare_parameter("dwa.max_vel_x", 0.5);
    node->declare_parameter("dwa.max_vel_theta", 1.0);
    node->declare_parameter("dwa.acc_lim_x", 2.5);
    node->declare_parameter("dwa.acc_lim_theta", 3.2);
    
    node->declare_parameter("teb.max_vel_x", 0.4);
    node->declare_parameter("teb.max_vel_theta", 0.9);
    node->declare_parameter("teb.optimization_steps", 3);
    
    // Create navigation manager
    auto navigation = std::make_shared<NavigationManager>(node);
    
    // Spin node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

#### 4. Giải thích chi tiết cách hoạt động
1. **Strategy Interface**:
   - NavigationStrategy định nghĩa interface chung
   - Các phương thức chuẩn cho mọi planner
   - Tính đa hình thông qua virtual methods

2. **Concrete Strategies**:
   - DWAPlanner và TebPlanner implements interface
   - Mỗi planner có thuật toán riêng
   - Error handling và logging

3. **Context**:
   - NavigationManager quản lý strategies
   - Thay đổi strategy runtime
   - Xử lý ROS2 communication

#### 5. Ưu điểm trong ROS2
1. **Flexibility**:
   - Dễ dàng thêm planners mới
   - Runtime strategy switching
   - Parameter configuration

2. **Maintainability**:
   - Code organization rõ ràng
   - Separation of concerns
   - Dễ test và debug

3. **Reusability**:
   - Common interface cho planners
   - Shared functionality
   - Code reuse

#### 6. Các trường hợp sử dụng trong ROS2
1. **Navigation**:
   - Path planning
   - Obstacle avoidance
   - Local planning

2. **Control**:
   - Motion controllers
   - Joint controllers
   - Force controllers

3. **Perception**:
   - Object detection
   - SLAM algorithms
   - Sensor fusion

#### 7. Best Practices trong ROS2
1. **Error Handling**:
```cpp
try {
    auto cmd_vel = strategy->computeVelocity(current_pose, goal_pose, map);
    if (!validateCommand(cmd_vel)) {
        RCLCPP_WARN(logger, "Invalid velocity command");
        return;
    }
    publishCommand(cmd_vel);
} catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Computation error: %s", e.what());
}
```

2. **Parameter Management**:
```cpp
void loadParameters() {
    auto params = node_->get_parameters("planner");
    for (const auto& param : params) {
        config_[param.get_name()] = param.value_to_string();
    }
}
```

3. **Resource Management**:
```cpp
class SafeStrategy {
    std::unique_ptr<NavigationStrategy> strategy_;
public:
    ~SafeStrategy() {
        if (strategy_) {
            strategy_->cleanup();
        }
    }
};
```

#### 8. Mở rộng và tùy chỉnh
1. **Dynamic Loading**:
```cpp
class PluginStrategy : public NavigationStrategy {
    void loadPlugin(const std::string& name) {
        // Load navigation plugin dynamically
        plugin_ = loadNavigationPlugin(name);
    }
};
```

2. **Configuration System**:
```cpp
class ConfigurableStrategy : public NavigationStrategy {
    void configure(const YAML::Node& config) {
        // Configure strategy from YAML
        loadConfiguration(config);
    }
};
```

3. **Hybrid System**:
```cpp
class HybridStrategy : public NavigationStrategy {
    void selectStrategy(const State& state) {
        // Select appropriate strategy based on state
        current_ = chooseStrategy(state);
    }
};
```

#### 9. Testing
1. **Mock Objects**:
```cpp
class MockNavigationStrategy : public NavigationStrategy {
public:
    MOCK_METHOD(bool, initialize, (const rclcpp::Node::SharedPtr&), (override));
    MOCK_METHOD(geometry_msgs::msg::Twist, computeVelocity, 
        (const geometry_msgs::msg::PoseStamped&,
         const geometry_msgs::msg::PoseStamped&,
         const nav_msgs::msg::OccupancyGrid&), (override));
};
```

2. **Strategy Tests**:
```cpp
TEST(NavigationTest, DWAPlannerTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto planner = std::make_unique<DWAPlanner>();
    
    EXPECT_TRUE(planner->initialize(node));
    
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped goal_pose;
    nav_msgs::msg::OccupancyGrid map;
    
    auto cmd_vel = planner->computeVelocity(current_pose, goal_pose, map);
    EXPECT_NE(cmd_vel.linear.x, 0.0);
}
```

3. **Integration Tests**:
```cpp
TEST(NavigationSystemTest, FullSystemTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto navigation = std::make_shared<NavigationManager>(node);
    
    // Test strategy switching
    EXPECT_TRUE(navigation->setStrategy("DWA"));
    EXPECT_TRUE(navigation->setStrategy("TEB"));
    
    // Test navigation to goal
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 1.0;
    
    navigation->setGoal(goal);
    
    // Wait for navigation
    rclcpp::spin_some(node);
    
    // Verify goal reached
    EXPECT_TRUE(navigation->isGoalReached());
}
```

#### 10. Kết luận
Strategy Pattern là một mẫu thiết kế quan trọng trong ROS2, đặc biệt hữu ích cho việc quản lý các thuật toán và chiến lược khác nhau trong robotics. Pattern này mang lại nhiều lợi ích:

1. **Flexibility và Adaptability**:
   - Dễ dàng thêm thuật toán mới
   - Runtime strategy switching
   - Dynamic configuration

2. **Code Organization**:
   - Clean separation of concerns
   - Interface standards
   - Dễ maintain và test

3. **Error Handling**:
   - Robust error management
   - Graceful degradation
   - Safe fallbacks

4. **Performance Optimization**:
   - Strategy selection based on conditions
   - Resource efficient
   - Runtime optimization

Trong ví dụ về navigation system, chúng ta đã thấy Strategy Pattern giúp xây dựng một hệ thống navigation linh hoạt và mạnh mẽ. Pattern này là lựa chọn tốt cho các hệ thống ROS2 cần quản lý nhiều thuật toán khác nhau và có thể thay đổi chiến lược runtime. 