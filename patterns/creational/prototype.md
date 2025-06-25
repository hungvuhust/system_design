## PROTOTYPE PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Prototype Pattern cho phép tạo ra các đối tượng mới bằng cách cloning các đối tượng hiện có thay vì tạo từ đầu. Trong ROS2, pattern này đặc biệt hữu ích cho:

- Clone robot configurations
- Tạo sensor templates
- Copy navigation waypoints
- Duplicate mission plans
- Clone node configurations
- Tạo fleet của robots tương tự

#### 2. Định nghĩa chi tiết
Prototype Pattern xác định các loại đối tượng cần tạo bằng cách sử dụng một instance nguyên mẫu, và tạo các đối tượng mới bằng cách copying nguyên mẫu này.

#### Các thành phần chính:
1. **Prototype Interface**:
   - Định nghĩa clone method
   - Common interface cho cloning

2. **Concrete Prototypes**:
   - Implements clone method
   - Thực hiện deep/shallow copy

3. **Client**:
   - Sử dụng prototype để tạo objects
   - Quản lý prototype registry

4. **Prototype Registry**:
   - Lưu trữ các prototypes
   - Factory pattern integration

#### 3. Ví dụ thực tế trong ROS2

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

// 1. Prototype Interface
class RobotComponentPrototype {
public:
    virtual ~RobotComponentPrototype() = default;
    
    // Core cloning method
    virtual std::unique_ptr<RobotComponentPrototype> clone() const = 0;
    
    // Configuration methods
    virtual void configure(const YAML::Node& config) = 0;
    virtual YAML::Node getConfiguration() const = 0;
    
    // Validation and initialization
    virtual bool validate() const = 0;
    virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;
    
    // Type information
    virtual std::string getType() const = 0;
    virtual std::string getName() const = 0;
    virtual void setName(const std::string& name) = 0;
};

// 2. Sensor Component Prototype
class SensorComponentPrototype : public RobotComponentPrototype {
public:
    struct SensorConfig {
        std::string sensor_type;
        std::string topic_name;
        std::string frame_id;
        double frequency;
        std::map<std::string, double> parameters;
        bool enabled;
    };
    
    SensorComponentPrototype(const SensorConfig& config) 
        : config_(config), name_("sensor_" + config.sensor_type) {}
    
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        // Deep copy of configuration
        auto cloned_config = config_;
        auto cloned = std::make_unique<SensorComponentPrototype>(cloned_config);
        cloned->setName(name_ + "_clone");
        
        // Copy runtime state if needed
        cloned->initialized_ = false;  // Reset initialization state
        
        return std::move(cloned);
    }
    
    void configure(const YAML::Node& config) override {
        try {
            if (config["sensor_type"]) {
                config_.sensor_type = config["sensor_type"].as<std::string>();
            }
            if (config["topic_name"]) {
                config_.topic_name = config["topic_name"].as<std::string>();
            }
            if (config["frame_id"]) {
                config_.frame_id = config["frame_id"].as<std::string>();
            }
            if (config["frequency"]) {
                config_.frequency = config["frequency"].as<double>();
            }
            if (config["enabled"]) {
                config_.enabled = config["enabled"].as<bool>();
            }
            if (config["parameters"]) {
                config_.parameters.clear();
                for (const auto& param : config["parameters"]) {
                    config_.parameters[param.first.as<std::string>()] = 
                        param.second.as<double>();
                }
            }
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to configure sensor: " + std::string(e.what()));
        }
    }
    
    YAML::Node getConfiguration() const override {
        YAML::Node config;
        config["sensor_type"] = config_.sensor_type;
        config["topic_name"] = config_.topic_name;
        config["frame_id"] = config_.frame_id;
        config["frequency"] = config_.frequency;
        config["enabled"] = config_.enabled;
        
        for (const auto& param : config_.parameters) {
            config["parameters"][param.first] = param.second;
        }
        
        return config;
    }
    
    bool validate() const override {
        return !config_.sensor_type.empty() && 
               !config_.topic_name.empty() && 
               !config_.frame_id.empty() && 
               config_.frequency > 0.0;
    }
    
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        if (!validate()) {
            RCLCPP_ERROR(node->get_logger(), "Invalid sensor configuration");
            return false;
        }
        
        try {
            node_ = node;
            
            // Create publisher based on sensor type
            if (config_.sensor_type == "laser") {
                laser_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
                    config_.topic_name, 10);
            } else if (config_.sensor_type == "camera") {
                // Create camera publisher
                // Implementation specific to sensor type
            }
            
            // Create timer for publishing
            auto period = std::chrono::milliseconds(
                static_cast<int>(1000.0 / config_.frequency));
            timer_ = node_->create_wall_timer(
                period, std::bind(&SensorComponentPrototype::publishData, this));
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                "Sensor %s initialized on topic %s", 
                name_.c_str(), config_.topic_name.c_str());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize sensor %s: %s", name_.c_str(), e.what());
            return false;
        }
    }
    
    std::string getType() const override { return "SENSOR"; }
    std::string getName() const override { return name_; }
    void setName(const std::string& name) override { name_ = name; }
    
    // Sensor-specific methods
    const SensorConfig& getSensorConfig() const { return config_; }
    void setSensorConfig(const SensorConfig& config) { config_ = config; }

private:
    void publishData() {
        if (!initialized_ || !config_.enabled) return;
        
        try {
            if (config_.sensor_type == "laser" && laser_pub_) {
                auto laser_msg = generateLaserData();
                laser_pub_->publish(laser_msg);
            }
            // Add other sensor types as needed
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error publishing sensor data: %s", e.what());
        }
    }
    
    sensor_msgs::msg::LaserScan generateLaserData() {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = node_->now();
        scan.header.frame_id = config_.frame_id;
        
        // Generate simulated laser data
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI / 180.0;
        scan.time_increment = 0.0;
        scan.scan_time = 1.0 / config_.frequency;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        
        size_t num_readings = 360;
        scan.ranges.resize(num_readings);
        for (size_t i = 0; i < num_readings; ++i) {
            scan.ranges[i] = 5.0;  // Simulated distance
        }
        
        return scan;
    }
    
    SensorConfig config_;
    std::string name_;
    bool initialized_ = false;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 3. Navigation Component Prototype
class NavigationComponentPrototype : public RobotComponentPrototype {
public:
    struct NavigationConfig {
        std::string planner_type;
        std::string controller_type;
        double max_velocity;
        double max_acceleration;
        double goal_tolerance;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::map<std::string, double> planner_params;
    };
    
    NavigationComponentPrototype(const NavigationConfig& config)
        : config_(config), name_("navigation_" + config.planner_type) {}
    
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        // Deep copy navigation configuration
        auto cloned_config = config_;
        
        // Deep copy waypoints
        cloned_config.waypoints.clear();
        for (const auto& waypoint : config_.waypoints) {
            cloned_config.waypoints.push_back(waypoint);
        }
        
        // Deep copy parameters
        cloned_config.planner_params = config_.planner_params;
        
        auto cloned = std::make_unique<NavigationComponentPrototype>(cloned_config);
        cloned->setName(name_ + "_clone");
        
        return std::move(cloned);
    }
    
    void configure(const YAML::Node& config) override {
        try {
            if (config["planner_type"]) {
                config_.planner_type = config["planner_type"].as<std::string>();
            }
            if (config["controller_type"]) {
                config_.controller_type = config["controller_type"].as<std::string>();
            }
            if (config["max_velocity"]) {
                config_.max_velocity = config["max_velocity"].as<double>();
            }
            if (config["max_acceleration"]) {
                config_.max_acceleration = config["max_acceleration"].as<double>();
            }
            if (config["goal_tolerance"]) {
                config_.goal_tolerance = config["goal_tolerance"].as<double>();
            }
            
            // Load waypoints
            if (config["waypoints"]) {
                config_.waypoints.clear();
                for (const auto& wp : config["waypoints"]) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = wp["x"].as<double>();
                    pose.pose.position.y = wp["y"].as<double>();
                    pose.pose.position.z = wp["z"].as<double>(0.0);
                    config_.waypoints.push_back(pose);
                }
            }
            
            // Load planner parameters
            if (config["planner_params"]) {
                config_.planner_params.clear();
                for (const auto& param : config["planner_params"]) {
                    config_.planner_params[param.first.as<std::string>()] = 
                        param.second.as<double>();
                }
            }
            
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to configure navigation: " + std::string(e.what()));
        }
    }
    
    YAML::Node getConfiguration() const override {
        YAML::Node config;
        config["planner_type"] = config_.planner_type;
        config["controller_type"] = config_.controller_type;
        config["max_velocity"] = config_.max_velocity;
        config["max_acceleration"] = config_.max_acceleration;
        config["goal_tolerance"] = config_.goal_tolerance;
        
        // Save waypoints
        for (size_t i = 0; i < config_.waypoints.size(); ++i) {
            const auto& wp = config_.waypoints[i];
            config["waypoints"][i]["x"] = wp.pose.position.x;
            config["waypoints"][i]["y"] = wp.pose.position.y;
            config["waypoints"][i]["z"] = wp.pose.position.z;
        }
        
        // Save planner parameters
        for (const auto& param : config_.planner_params) {
            config["planner_params"][param.first] = param.second;
        }
        
        return config;
    }
    
    bool validate() const override {
        return !config_.planner_type.empty() && 
               !config_.controller_type.empty() && 
               config_.max_velocity > 0.0 && 
               config_.max_acceleration > 0.0 && 
               config_.goal_tolerance > 0.0;
    }
    
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        if (!validate()) {
            RCLCPP_ERROR(node->get_logger(), "Invalid navigation configuration");
            return false;
        }
        
        try {
            node_ = node;
            
            // Create navigation publishers and subscribers
            path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
            goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10,
                std::bind(&NavigationComponentPrototype::goalCallback, this, 
                         std::placeholders::_1));
            
            // Initialize planner based on type
            initializePlanner();
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                "Navigation component %s initialized with planner %s", 
                name_.c_str(), config_.planner_type.c_str());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize navigation %s: %s", name_.c_str(), e.what());
            return false;
        }
    }
    
    std::string getType() const override { return "NAVIGATION"; }
    std::string getName() const override { return name_; }
    void setName(const std::string& name) override { name_ = name; }
    
    // Navigation-specific methods
    const NavigationConfig& getNavigationConfig() const { return config_; }
    void addWaypoint(const geometry_msgs::msg::PoseStamped& waypoint) {
        config_.waypoints.push_back(waypoint);
    }
    
    void clearWaypoints() {
        config_.waypoints.clear();
    }

private:
    void initializePlanner() {
        // Initialize planner based on type
        if (config_.planner_type == "A*") {
            // Initialize A* planner
        } else if (config_.planner_type == "RRT") {
            // Initialize RRT planner
        } else if (config_.planner_type == "DWA") {
            // Initialize DWA planner
        }
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!initialized_) return;
        
        try {
            // Plan path to goal
            auto path = planPath(*msg);
            if (!path.poses.empty()) {
                path_pub_->publish(path);
                RCLCPP_INFO(node_->get_logger(), 
                    "Published path with %zu poses", path.poses.size());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error planning path: %s", e.what());
        }
    }
    
    nav_msgs::msg::Path planPath(const geometry_msgs::msg::PoseStamped& goal) {
        nav_msgs::msg::Path path;
        path.header.stamp = node_->now();
        path.header.frame_id = "map";
        
        // Simple path planning - use waypoints or direct line
        if (!config_.waypoints.empty()) {
            path.poses = config_.waypoints;
        } else {
            // Create simple straight line path
            geometry_msgs::msg::PoseStamped start;
            start.header = path.header;
            // Assume robot starts at origin
            start.pose.position.x = 0.0;
            start.pose.position.y = 0.0;
            start.pose.position.z = 0.0;
            
            path.poses.push_back(start);
            path.poses.push_back(goal);
        }
        
        return path;
    }
    
    NavigationConfig config_;
    std::string name_;
    bool initialized_ = false;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

// 4. Robot Configuration Prototype
class RobotConfigurationPrototype : public RobotComponentPrototype {
public:
    struct RobotConfig {
        std::string robot_type;
        std::string robot_name;
        std::string urdf_path;
        std::map<std::string, std::unique_ptr<RobotComponentPrototype>> components;
        std::map<std::string, std::string> parameters;
    };
    
    RobotConfigurationPrototype(const std::string& robot_type)
        : robot_type_(robot_type), name_("robot_" + robot_type) {}
    
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        auto cloned = std::make_unique<RobotConfigurationPrototype>(robot_type_);
        cloned->setName(name_ + "_clone");
        cloned->urdf_path_ = urdf_path_;
        cloned->parameters_ = parameters_;
        
        // Deep clone all components
        for (const auto& [component_name, component] : components_) {
            auto cloned_component = component->clone();
            cloned_component->setName(component_name + "_clone");
            cloned->components_[component_name + "_clone"] = std::move(cloned_component);
        }
        
        return std::move(cloned);
    }
    
    void configure(const YAML::Node& config) override {
        try {
            if (config["robot_type"]) {
                robot_type_ = config["robot_type"].as<std::string>();
            }
            if (config["urdf_path"]) {
                urdf_path_ = config["urdf_path"].as<std::string>();
            }
            if (config["parameters"]) {
                parameters_.clear();
                for (const auto& param : config["parameters"]) {
                    parameters_[param.first.as<std::string>()] = 
                        param.second.as<std::string>();
                }
            }
            
            // Configure components
            if (config["components"]) {
                for (const auto& comp : config["components"]) {
                    std::string comp_name = comp.first.as<std::string>();
                    if (auto it = components_.find(comp_name); it != components_.end()) {
                        it->second->configure(comp.second);
                    }
                }
            }
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to configure robot: " + std::string(e.what()));
        }
    }
    
    YAML::Node getConfiguration() const override {
        YAML::Node config;
        config["robot_type"] = robot_type_;
        config["urdf_path"] = urdf_path_;
        
        for (const auto& param : parameters_) {
            config["parameters"][param.first] = param.second;
        }
        
        for (const auto& [comp_name, component] : components_) {
            config["components"][comp_name] = component->getConfiguration();
        }
        
        return config;
    }
    
    bool validate() const override {
        if (robot_type_.empty() || urdf_path_.empty()) {
            return false;
        }
        
        // Validate all components
        for (const auto& [name, component] : components_) {
            if (!component->validate()) {
                return false;
            }
        }
        
        return true;
    }
    
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        if (!validate()) {
            RCLCPP_ERROR(node->get_logger(), "Invalid robot configuration");
            return false;
        }
        
        try {
            node_ = node;
            
            // Initialize all components
            for (const auto& [comp_name, component] : components_) {
                if (!component->initialize(node)) {
                    RCLCPP_ERROR(node->get_logger(), 
                        "Failed to initialize component: %s", comp_name.c_str());
                    return false;
                }
            }
            
            initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                "Robot configuration %s initialized with %zu components", 
                name_.c_str(), components_.size());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to initialize robot %s: %s", name_.c_str(), e.what());
            return false;
        }
    }
    
    std::string getType() const override { return "ROBOT_CONFIG"; }
    std::string getName() const override { return name_; }
    void setName(const std::string& name) override { name_ = name; }
    
    // Robot-specific methods
    void addComponent(const std::string& name, 
                     std::unique_ptr<RobotComponentPrototype> component) {
        components_[name] = std::move(component);
    }
    
    void removeComponent(const std::string& name) {
        components_.erase(name);
    }
    
    RobotComponentPrototype* getComponent(const std::string& name) {
        auto it = components_.find(name);
        return (it != components_.end()) ? it->second.get() : nullptr;
    }
    
    void setParameter(const std::string& name, const std::string& value) {
        parameters_[name] = value;
    }
    
    std::string getParameter(const std::string& name) const {
        auto it = parameters_.find(name);
        return (it != parameters_.end()) ? it->second : "";
    }

private:
    std::string robot_type_;
    std::string name_;
    std::string urdf_path_;
    std::map<std::string, std::unique_ptr<RobotComponentPrototype>> components_;
    std::map<std::string, std::string> parameters_;
    bool initialized_ = false;
    
    rclcpp::Node::SharedPtr node_;
};

// 5. Prototype Registry
class RobotPrototypeRegistry {
public:
    static RobotPrototypeRegistry& getInstance() {
        static RobotPrototypeRegistry instance;
        return instance;
    }
    
    void registerPrototype(const std::string& name, 
                          std::unique_ptr<RobotComponentPrototype> prototype) {
        prototypes_[name] = std::move(prototype);
        RCLCPP_INFO(rclcpp::get_logger("PrototypeRegistry"), 
            "Registered prototype: %s", name.c_str());
    }
    
    std::unique_ptr<RobotComponentPrototype> createFromPrototype(const std::string& name) {
        auto it = prototypes_.find(name);
        if (it == prototypes_.end()) {
            throw std::runtime_error("Prototype not found: " + name);
        }
        
        return it->second->clone();
    }
    
    bool hasPrototype(const std::string& name) const {
        return prototypes_.find(name) != prototypes_.end();
    }
    
    std::vector<std::string> getAvailablePrototypes() const {
        std::vector<std::string> names;
        for (const auto& [name, prototype] : prototypes_) {
            names.push_back(name);
        }
        return names;
    }
    
    void loadPrototypesFromFile(const std::string& config_file) {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            if (config["prototypes"]) {
                for (const auto& proto : config["prototypes"]) {
                    std::string name = proto["name"].as<std::string>();
                    std::string type = proto["type"].as<std::string>();
                    
                    std::unique_ptr<RobotComponentPrototype> prototype;
                    
                    if (type == "sensor") {
                        SensorComponentPrototype::SensorConfig sensor_config;
                        sensor_config.sensor_type = proto["config"]["sensor_type"].as<std::string>();
                        sensor_config.topic_name = proto["config"]["topic_name"].as<std::string>();
                        sensor_config.frame_id = proto["config"]["frame_id"].as<std::string>();
                        sensor_config.frequency = proto["config"]["frequency"].as<double>();
                        sensor_config.enabled = proto["config"]["enabled"].as<bool>(true);
                        
                        prototype = std::make_unique<SensorComponentPrototype>(sensor_config);
                    } else if (type == "navigation") {
                        NavigationComponentPrototype::NavigationConfig nav_config;
                        nav_config.planner_type = proto["config"]["planner_type"].as<std::string>();
                        nav_config.controller_type = proto["config"]["controller_type"].as<std::string>();
                        nav_config.max_velocity = proto["config"]["max_velocity"].as<double>();
                        nav_config.max_acceleration = proto["config"]["max_acceleration"].as<double>();
                        nav_config.goal_tolerance = proto["config"]["goal_tolerance"].as<double>();
                        
                        prototype = std::make_unique<NavigationComponentPrototype>(nav_config);
                    } else if (type == "robot") {
                        prototype = std::make_unique<RobotConfigurationPrototype>(
                            proto["config"]["robot_type"].as<std::string>());
                    }
                    
                    if (prototype) {
                        prototype->configure(proto["config"]);
                        registerPrototype(name, std::move(prototype));
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("PrototypeRegistry"), 
                "Failed to load prototypes: %s", e.what());
        }
    }
    
    void savePrototypesToFile(const std::string& config_file) {
        try {
            YAML::Node config;
            
            for (const auto& [name, prototype] : prototypes_) {
                YAML::Node proto_node;
                proto_node["name"] = name;
                proto_node["type"] = prototype->getType();
                proto_node["config"] = prototype->getConfiguration();
                
                config["prototypes"].push_back(proto_node);
            }
            
            std::ofstream file(config_file);
            file << config;
            
            RCLCPP_INFO(rclcpp::get_logger("PrototypeRegistry"), 
                "Saved %zu prototypes to %s", prototypes_.size(), config_file.c_str());
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("PrototypeRegistry"), 
                "Failed to save prototypes: %s", e.what());
        }
    }

private:
    RobotPrototypeRegistry() = default;
    std::map<std::string, std::unique_ptr<RobotComponentPrototype>> prototypes_;
};

// 6. Robot Factory using Prototypes
class RobotFactory {
public:
    RobotFactory() : registry_(RobotPrototypeRegistry::getInstance()) {}
    
    std::unique_ptr<RobotComponentPrototype> createRobot(const std::string& robot_type) {
        try {
            return registry_.createFromPrototype(robot_type);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotFactory"), 
                "Failed to create robot: %s", e.what());
            return nullptr;
        }
    }
    
    std::unique_ptr<RobotComponentPrototype> createComponent(const std::string& component_type) {
        try {
            return registry_.createFromPrototype(component_type);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotFactory"), 
                "Failed to create component: %s", e.what());
            return nullptr;
        }
    }
    
    std::unique_ptr<RobotConfigurationPrototype> createRobotFleet(
        const std::string& base_robot_type, int fleet_size) {
        
        auto fleet_config = std::make_unique<RobotConfigurationPrototype>("fleet");
        fleet_config->setName("robot_fleet");
        
        for (int i = 0; i < fleet_size; ++i) {
            auto robot = createRobot(base_robot_type);
            if (robot) {
                robot->setName(base_robot_type + "_" + std::to_string(i));
                fleet_config->addComponent(robot->getName(), std::move(robot));
            }
        }
        
        return fleet_config;
    }
    
    bool customizeRobot(RobotComponentPrototype* robot, 
                       const std::map<std::string, YAML::Node>& customizations) {
        try {
            for (const auto& [component_name, config] : customizations) {
                if (auto robot_config = dynamic_cast<RobotConfigurationPrototype*>(robot)) {
                    if (auto component = robot_config->getComponent(component_name)) {
                        component->configure(config);
                    }
                }
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotFactory"), 
                "Failed to customize robot: %s", e.what());
            return false;
        }
    }

private:
    RobotPrototypeRegistry& registry_;
};

// 7. Robot Manager for handling multiple robots
class RobotManager {
public:
    RobotManager(const rclcpp::Node::SharedPtr& node) 
        : node_(node), factory_(std::make_unique<RobotFactory>()) {
        
        // Load prototypes from configuration
        std::string config_file = node_->declare_parameter<std::string>(
            "prototype_config", "config/robot_prototypes.yaml");
        
        RobotPrototypeRegistry::getInstance().loadPrototypesFromFile(config_file);
    }
    
    bool createRobot(const std::string& robot_name, const std::string& robot_type) {
        try {
            auto robot = factory_->createRobot(robot_type);
            if (!robot) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "Failed to create robot of type: %s", robot_type.c_str());
                return false;
            }
            
            robot->setName(robot_name);
            
            if (!robot->initialize(node_)) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "Failed to initialize robot: %s", robot_name.c_str());
                return false;
            }
            
            robots_[robot_name] = std::move(robot);
            
            RCLCPP_INFO(node_->get_logger(), 
                "Created and initialized robot: %s", robot_name.c_str());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error creating robot %s: %s", robot_name.c_str(), e.what());
            return false;
        }
    }
    
    bool cloneRobot(const std::string& source_robot, const std::string& new_robot_name) {
        auto it = robots_.find(source_robot);
        if (it == robots_.end()) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Source robot not found: %s", source_robot.c_str());
            return false;
        }
        
        try {
            auto cloned_robot = it->second->clone();
            cloned_robot->setName(new_robot_name);
            
            if (!cloned_robot->initialize(node_)) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "Failed to initialize cloned robot: %s", new_robot_name.c_str());
                return false;
            }
            
            robots_[new_robot_name] = std::move(cloned_robot);
            
            RCLCPP_INFO(node_->get_logger(), 
                "Cloned robot %s as %s", source_robot.c_str(), new_robot_name.c_str());
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error cloning robot: %s", e.what());
            return false;
        }
    }
    
    bool createFleet(const std::string& fleet_name, const std::string& robot_type, int size) {
        try {
            auto fleet = factory_->createRobotFleet(robot_type, size);
            if (!fleet) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create fleet");
                return false;
            }
            
            fleet->setName(fleet_name);
            
            if (!fleet->initialize(node_)) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "Failed to initialize fleet: %s", fleet_name.c_str());
                return false;
            }
            
            robots_[fleet_name] = std::move(fleet);
            
            RCLCPP_INFO(node_->get_logger(), 
                "Created fleet %s with %d robots", fleet_name.c_str(), size);
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), 
                "Error creating fleet: %s", e.what());
            return false;
        }
    }
    
    RobotComponentPrototype* getRobot(const std::string& robot_name) {
        auto it = robots_.find(robot_name);
        return (it != robots_.end()) ? it->second.get() : nullptr;
    }
    
    std::vector<std::string> getActiveRobots() const {
        std::vector<std::string> names;
        for (const auto& [name, robot] : robots_) {
            names.push_back(name);
        }
        return names;
    }
    
    void removeRobot(const std::string& robot_name) {
        robots_.erase(robot_name);
        RCLCPP_INFO(node_->get_logger(), "Removed robot: %s", robot_name.c_str());
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<RobotFactory> factory_;
    std::map<std::string, std::unique_ptr<RobotComponentPrototype>> robots_;
};

// 8. Usage Example
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_prototype_manager");
    
    try {
        // Create robot manager
        RobotManager manager(node);
        
        // Example 1: Create robots from prototypes
        RCLCPP_INFO(node->get_logger(), "Creating robots from prototypes...");
        
        manager.createRobot("turtlebot_1", "turtlebot_prototype");
        manager.createRobot("industrial_arm", "ur5_prototype");
        manager.createRobot("drone_1", "quadrotor_prototype");
        
        // Example 2: Clone existing robot
        RCLCPP_INFO(node->get_logger(), "Cloning robots...");
        
        manager.cloneRobot("turtlebot_1", "turtlebot_2");
        manager.cloneRobot("turtlebot_1", "turtlebot_3");
        
        // Example 3: Create robot fleet
        RCLCPP_INFO(node->get_logger(), "Creating robot fleet...");
        
        manager.createFleet("delivery_fleet", "delivery_robot_prototype", 5);
        
        // Example 4: Runtime prototype registration
        RCLCPP_INFO(node->get_logger(), "Registering new prototype...");
        
        auto& registry = RobotPrototypeRegistry::getInstance();
        
        // Create custom sensor prototype
        SensorComponentPrototype::SensorConfig custom_sensor;
        custom_sensor.sensor_type = "custom_lidar";
        custom_sensor.topic_name = "/custom_scan";
        custom_sensor.frame_id = "custom_lidar_frame";
        custom_sensor.frequency = 20.0;
        custom_sensor.enabled = true;
        
        auto custom_prototype = std::make_unique<SensorComponentPrototype>(custom_sensor);
        registry.registerPrototype("custom_lidar_prototype", std::move(custom_prototype));
        
        // Use the new prototype
        manager.createRobot("custom_robot", "custom_lidar_prototype");
        
        // Example 5: List available prototypes and robots
        auto prototypes = registry.getAvailablePrototypes();
        RCLCPP_INFO(node->get_logger(), "Available prototypes:");
        for (const auto& proto : prototypes) {
            RCLCPP_INFO(node->get_logger(), "  - %s", proto.c_str());
        }
        
        auto active_robots = manager.getActiveRobots();
        RCLCPP_INFO(node->get_logger(), "Active robots:");
        for (const auto& robot : active_robots) {
            RCLCPP_INFO(node->get_logger(), "  - %s", robot.c_str());
        }
        
        // Run the system
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Application error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
```

#### 4. Giải thích chi tiết cách hoạt động

1. **Prototype Interface**:
   - `RobotComponentPrototype` định nghĩa clone method
   - Common interface cho configuration và validation
   - Support cho initialization và type information

2. **Concrete Prototypes**:
   - `SensorComponentPrototype`: Clone sensor configurations
   - `NavigationComponentPrototype`: Clone navigation setups
   - `RobotConfigurationPrototype`: Clone complete robot configs

3. **Prototype Registry**:
   - Central storage cho prototypes
   - File-based persistence
   - Runtime registration support

4. **Factory Integration**:
   - `RobotFactory` sử dụng prototypes
   - Fleet creation capabilities
   - Customization support

#### 5. Ưu điểm trong ROS2

1. **Performance Benefits**:
   - Faster object creation than constructors
   - Reuse of expensive initialization
   - Memory efficiency với shared configurations

2. **Flexibility**:
   - Runtime prototype registration
   - Dynamic robot creation
   - Easy configuration variants

3. **Scalability**:
   - Support cho robot fleets
   - Efficient cloning
   - Resource sharing

#### 6. Các trường hợp sử dụng trong ROS2

1. **Robot Fleet Management**:
```cpp
class FleetManager {
    std::vector<std::unique_ptr<RobotComponentPrototype>> fleet_;
    
public:
    void deployFleet(const std::string& base_prototype, int size) {
        auto& registry = RobotPrototypeRegistry::getInstance();
        
        for (int i = 0; i < size; ++i) {
            auto robot = registry.createFromPrototype(base_prototype);
            robot->setName("robot_" + std::to_string(i));
            
            // Customize for specific role
            customizeForRole(robot.get(), i);
            
            fleet_.push_back(std::move(robot));
        }
    }
    
private:
    void customizeForRole(RobotComponentPrototype* robot, int role_id) {
        // Customize robot based on fleet role
    }
};
```

2. **Sensor Configuration Templates**:
```cpp
class SensorTemplateManager {
public:
    void createSensorTemplates() {
        auto& registry = RobotPrototypeRegistry::getInstance();
        
        // Indoor navigation sensors
        createIndoorSensorTemplate(registry);
        
        // Outdoor navigation sensors
        createOutdoorSensorTemplate(registry);
        
        // Industrial manipulation sensors
        createIndustrialSensorTemplate(registry);
    }
    
private:
    void createIndoorSensorTemplate(RobotPrototypeRegistry& registry) {
        SensorComponentPrototype::SensorConfig config;
        config.sensor_type = "indoor_lidar";
        config.topic_name = "/indoor_scan";
        config.frame_id = "indoor_lidar_frame";
        config.frequency = 10.0;
        config.parameters["range_max"] = 10.0;
        config.parameters["angle_resolution"] = 0.5;
        
        auto prototype = std::make_unique<SensorComponentPrototype>(config);
        registry.registerPrototype("indoor_sensor_template", std::move(prototype));
    }
};
```

3. **Mission Template System**:
```cpp
class MissionTemplateManager {
    struct MissionTemplate {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::map<std::string, double> mission_params;
        std::string mission_type;
    };
    
    std::map<std::string, MissionTemplate> templates_;
    
public:
    void registerMissionTemplate(const std::string& name, const MissionTemplate& mission) {
        templates_[name] = mission;
    }
    
    std::unique_ptr<NavigationComponentPrototype> createMissionFromTemplate(
        const std::string& template_name) {
        
        auto it = templates_.find(template_name);
        if (it == templates_.end()) {
            return nullptr;
        }
        
        NavigationComponentPrototype::NavigationConfig config;
        config.planner_type = "mission_planner";
        config.controller_type = "mission_controller";
        config.waypoints = it->second.waypoints;
        
        for (const auto& [key, value] : it->second.mission_params) {
            config.planner_params[key] = value;
        }
        
        return std::make_unique<NavigationComponentPrototype>(config);
    }
};
```

#### 7. Best Practices trong ROS2

1. **Deep vs Shallow Copy**:
```cpp
class SmartPrototype : public RobotComponentPrototype {
    std::shared_ptr<ExpensiveResource> shared_resource_;
    std::unique_ptr<ConfigData> config_data_;
    
public:
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        auto cloned = std::make_unique<SmartPrototype>();
        
        // Shared resource - shallow copy
        cloned->shared_resource_ = shared_resource_;
        
        // Configuration data - deep copy
        cloned->config_data_ = std::make_unique<ConfigData>(*config_data_);
        
        return std::move(cloned);
    }
};
```

2. **Lazy Initialization**:
```cpp
class LazyPrototype : public RobotComponentPrototype {
    mutable std::unique_ptr<ExpensiveComponent> component_;
    
public:
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        auto cloned = std::make_unique<LazyPrototype>();
        // Don't copy expensive component until needed
        return std::move(cloned);
    }
    
    ExpensiveComponent* getComponent() const {
        if (!component_) {
            component_ = std::make_unique<ExpensiveComponent>();
        }
        return component_.get();
    }
};
```

3. **Version Management**:
```cpp
class VersionedPrototype : public RobotComponentPrototype {
    int version_;
    std::string compatibility_level_;
    
public:
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        auto cloned = std::make_unique<VersionedPrototype>(*this);
        
        // Update version for clone
        cloned->version_ = version_ + 1;
        
        return std::move(cloned);
    }
    
    bool isCompatibleWith(const VersionedPrototype& other) const {
        return compatibility_level_ == other.compatibility_level_;
    }
};
```

#### 8. Mở rộng và tùy chỉnh

1. **Dynamic Prototype Loading**:
```cpp
class DynamicPrototypeLoader {
public:
    void loadPrototypeFromPlugin(const std::string& plugin_path) {
        // Load shared library
        void* handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
        
        if (!handle) {
            throw std::runtime_error("Cannot load plugin: " + std::string(dlerror()));
        }
        
        // Get factory function
        typedef RobotComponentPrototype* (*create_prototype_t)();
        create_prototype_t create_prototype = 
            (create_prototype_t) dlsym(handle, "create_prototype");
        
        if (!create_prototype) {
            dlclose(handle);
            throw std::runtime_error("Cannot find create_prototype function");
        }
        
        // Create and register prototype
        auto prototype = std::unique_ptr<RobotComponentPrototype>(create_prototype());
        RobotPrototypeRegistry::getInstance().registerPrototype(
            prototype->getName(), std::move(prototype));
    }
};
```

2. **Prototype Composition**:
```cpp
class CompositePrototype : public RobotComponentPrototype {
    std::vector<std::unique_ptr<RobotComponentPrototype>> child_prototypes_;
    
public:
    void addChildPrototype(std::unique_ptr<RobotComponentPrototype> child) {
        child_prototypes_.push_back(std::move(child));
    }
    
    std::unique_ptr<RobotComponentPrototype> clone() const override {
        auto cloned = std::make_unique<CompositePrototype>();
        
        // Clone all children
        for (const auto& child : child_prototypes_) {
            cloned->addChildPrototype(child->clone());
        }
        
        return std::move(cloned);
    }
    
    bool initialize(const rclcpp::Node::SharedPtr& node) override {
        // Initialize all children
        for (const auto& child : child_prototypes_) {
            if (!child->initialize(node)) {
                return false;
            }
        }
        return true;
    }
};
```

3. **Prototype Caching**:
```cpp
class PrototypeCache {
    std::map<std::string, std::weak_ptr<RobotComponentPrototype>> cache_;
    std::mutex cache_mutex_;
    
public:
    std::shared_ptr<RobotComponentPrototype> getCachedPrototype(
        const std::string& name) {
        
        std::lock_guard<std::mutex> lock(cache_mutex_);
        
        auto it = cache_.find(name);
        if (it != cache_.end()) {
            if (auto cached = it->second.lock()) {
                return cached;
            } else {
                cache_.erase(it);
            }
        }
        
        // Create new prototype
        auto& registry = RobotPrototypeRegistry::getInstance();
        auto prototype = registry.createFromPrototype(name);
        
        auto shared_proto = std::shared_ptr<RobotComponentPrototype>(
            prototype.release());
        cache_[name] = shared_proto;
        
        return shared_proto;
    }
};
```

#### 9. Testing

1. **Prototype Tests**:
```cpp
TEST(PrototypeTest, CloneTest) {
    // Create original sensor
    SensorComponentPrototype::SensorConfig config;
    config.sensor_type = "test_lidar";
    config.topic_name = "/test_scan";
    config.frame_id = "test_frame";
    config.frequency = 15.0;
    
    SensorComponentPrototype original(config);
    original.setName("original_sensor");
    
    // Clone sensor
    auto cloned = original.clone();
    
    // Verify clone is different object
    EXPECT_NE(&original, cloned.get());
    
    // Verify configurations are equal
    EXPECT_EQ(original.getConfiguration().size(), cloned->getConfiguration().size());
    
    // Verify names are different
    EXPECT_NE(original.getName(), cloned->getName());
}
```

2. **Registry Tests**:
```cpp
TEST(RegistryTest, PrototypeRegistrationTest) {
    auto& registry = RobotPrototypeRegistry::getInstance();
    
    // Create test prototype
    SensorComponentPrototype::SensorConfig config;
    config.sensor_type = "test_sensor";
    config.topic_name = "/test_topic";
    config.frame_id = "test_frame";
    config.frequency = 10.0;
    
    auto prototype = std::make_unique<SensorComponentPrototype>(config);
    
    // Register prototype
    registry.registerPrototype("test_prototype", std::move(prototype));
    
    // Verify registration
    EXPECT_TRUE(registry.hasPrototype("test_prototype"));
    
    // Create from prototype
    auto created = registry.createFromPrototype("test_prototype");
    EXPECT_NE(created, nullptr);
    EXPECT_EQ(created->getType(), "SENSOR");
}
```

3. **Integration Tests**:
```cpp
TEST(IntegrationTest, RobotFleetCreationTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    RobotManager manager(node);
    
    // Create fleet
    EXPECT_TRUE(manager.createFleet("test_fleet", "test_robot_prototype", 3));
    
    // Verify fleet creation
    auto active_robots = manager.getActiveRobots();
    EXPECT_EQ(active_robots.size(), 1);  // Fleet is one entity
    
    // Get fleet robot
    auto fleet = manager.getRobot("test_fleet");
    EXPECT_NE(fleet, nullptr);
    EXPECT_EQ(fleet->getType(), "ROBOT_CONFIG");
}
```

#### 10. Kết luận

Prototype Pattern là một mẫu thiết kế rất hiệu quả trong ROS2 robotics, đặc biệt hữu ích cho:

1. **Performance Optimization**:
   - Faster object creation through cloning
   - Reuse của expensive initialization
   - Memory efficient cho large fleets

2. **Configuration Management**:
   - Template-based robot configurations
   - Easy variant creation
   - Standardized component setups

3. **Scalability**:
   - Efficient fleet deployment
   - Dynamic prototype registration
   - Runtime configuration changes

4. **Flexibility**:
   - Support cho multiple robot types
   - Component composition
   - Plugin-based extensions

Trong robotics, Prototype Pattern đặc biệt quan trọng vì:
- Robots thường có configurations phức tạp
- Fleet operations cần many similar robots
- Sensor và component setups có thể được reused
- Runtime reconfiguration là common requirement

Pattern này giúp tạo ra một hệ thống quản lý robot linh hoạt, hiệu quả và dễ maintain trong môi trường ROS2.
