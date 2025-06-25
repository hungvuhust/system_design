## BUILDER PATTERN TRONG ROS2

#### 1. Giới thiệu đơn giản
Builder Pattern cho phép xây dựng các đối tượng phức tạp từng bước một cách có cấu trúc. Trong ROS2, pattern này đặc biệt hữu ích cho:

- Tạo robot configurations phức tạp
- Xây dựng launch files động
- Tạo sensor fusion pipelines
- Cấu hình navigation stacks
- Khởi tạo multi-robot systems

#### 2. Định nghĩa chi tiết
Builder Pattern tách biệt việc xây dựng đối tượng phức tạp khỏi cách biểu diễn của nó, cho phép cùng một quá trình xây dựng có thể tạo ra các biểu diễn khác nhau.

#### Các thành phần chính:
1. **Product**:
   - Đối tượng phức tạp cần tạo
   - Chứa nhiều components

2. **Builder Interface**:
   - Định nghĩa các bước xây dựng
   - Abstract interface cho builders

3. **Concrete Builders**:
   - Implements các bước cụ thể
   - Tạo product variants khác nhau

4. **Director**:
   - Điều khiển quá trình xây dựng
   - Sử dụng builder interface

#### 3. Ví dụ thực tế trong ROS2

```cpp
// 1. Product - Robot Configuration
class RobotConfiguration {
public:
    struct SensorConfig {
        std::string type;
        std::string topic;
        std::string frame_id;
        double frequency;
        std::map<std::string, double> parameters;
    };
    
    struct NavigationConfig {
        std::string global_planner;
        std::string local_planner;
        std::string costmap_type;
        std::map<std::string, double> parameters;
    };
    
    struct HardwareConfig {
        std::string driver_type;
        std::string device_path;
        std::map<std::string, std::string> hardware_params;
    };
    
    void addSensor(const SensorConfig& sensor) {
        sensors_.push_back(sensor);
    }
    
    void setNavigation(const NavigationConfig& nav) {
        navigation_ = nav;
    }
    
    void setHardware(const HardwareConfig& hw) {
        hardware_ = hw;
    }
    
    void setRobotDescription(const std::string& urdf_path) {
        robot_description_path_ = urdf_path;
    }
    
    void addLaunchFile(const std::string& package, const std::string& file) {
        launch_files_.push_back({package, file});
    }
    
    // Getters
    const std::vector<SensorConfig>& getSensors() const { return sensors_; }
    const NavigationConfig& getNavigation() const { return navigation_; }
    const HardwareConfig& getHardware() const { return hardware_; }
    const std::string& getRobotDescription() const { return robot_description_path_; }
    const std::vector<std::pair<std::string, std::string>>& getLaunchFiles() const { 
        return launch_files_; 
    }
    
    // Validation
    bool isValid() const {
        return !sensors_.empty() && 
               !robot_description_path_.empty() && 
               !hardware_.driver_type.empty();
    }
    
    // Generate launch content
    std::string generateLaunchContent() const {
        std::stringstream ss;
        ss << "<?xml version=\"1.0\"?>\n";
        ss << "<launch>\n";
        
        // Robot description
        ss << "  <param name=\"robot_description\" textfile=\"" 
           << robot_description_path_ << "\" />\n";
        
        // Hardware driver
        ss << "  <node pkg=\"" << hardware_.driver_type << "\" type=\"driver_node\" name=\"hardware_driver\">\n";
        for (const auto& param : hardware_.hardware_params) {
            ss << "    <param name=\"" << param.first << "\" value=\"" << param.second << "\" />\n";
        }
        ss << "  </node>\n";
        
        // Sensors
        for (const auto& sensor : sensors_) {
            ss << "  <node pkg=\"" << sensor.type << "\" type=\"sensor_node\" name=\"" << sensor.type << "_node\">\n";
            ss << "    <param name=\"topic\" value=\"" << sensor.topic << "\" />\n";
            ss << "    <param name=\"frame_id\" value=\"" << sensor.frame_id << "\" />\n";
            ss << "    <param name=\"frequency\" value=\"" << sensor.frequency << "\" />\n";
            for (const auto& param : sensor.parameters) {
                ss << "    <param name=\"" << param.first << "\" value=\"" << param.second << "\" />\n";
            }
            ss << "  </node>\n";
        }
        
        // Navigation
        if (!navigation_.global_planner.empty()) {
            ss << "  <include file=\"$(find nav2_bringup)/launch/navigation_launch.py\">\n";
            ss << "    <arg name=\"global_planner\" value=\"" << navigation_.global_planner << "\" />\n";
            ss << "    <arg name=\"local_planner\" value=\"" << navigation_.local_planner << "\" />\n";
            ss << "  </include>\n";
        }
        
        ss << "</launch>\n";
        return ss.str();
    }

private:
    std::vector<SensorConfig> sensors_;
    NavigationConfig navigation_;
    HardwareConfig hardware_;
    std::string robot_description_path_;
    std::vector<std::pair<std::string, std::string>> launch_files_;
};

// 2. Builder Interface
class RobotConfigurationBuilder {
public:
    virtual ~RobotConfigurationBuilder() = default;
    
    virtual RobotConfigurationBuilder& setSensorSuite() = 0;
    virtual RobotConfigurationBuilder& setNavigationStack() = 0;
    virtual RobotConfigurationBuilder& setHardwareInterface() = 0;
    virtual RobotConfigurationBuilder& setRobotModel() = 0;
    virtual RobotConfigurationBuilder& addCustomComponents() = 0;
    
    virtual std::unique_ptr<RobotConfiguration> build() = 0;
    
protected:
    std::unique_ptr<RobotConfiguration> configuration_;
    
    void reset() {
        configuration_ = std::make_unique<RobotConfiguration>();
    }
};

// 3. Concrete Builders
class TurtleBotBuilder : public RobotConfigurationBuilder {
public:
    TurtleBotBuilder() {
        reset();
    }
    
    RobotConfigurationBuilder& setSensorSuite() override {
        // Add LiDAR
        RobotConfiguration::SensorConfig lidar;
        lidar.type = "rplidar_ros";
        lidar.topic = "/scan";
        lidar.frame_id = "laser_frame";
        lidar.frequency = 10.0;
        lidar.parameters["angle_compensate"] = 1.0;
        lidar.parameters["serial_port"] = 0;  // /dev/ttyUSB0
        configuration_->addSensor(lidar);
        
        // Add Camera
        RobotConfiguration::SensorConfig camera;
        camera.type = "usb_cam";
        camera.topic = "/camera/image_raw";
        camera.frame_id = "camera_frame";
        camera.frequency = 30.0;
        camera.parameters["video_device"] = 0;  // /dev/video0
        camera.parameters["image_width"] = 640.0;
        camera.parameters["image_height"] = 480.0;
        configuration_->addSensor(camera);
        
        // Add IMU
        RobotConfiguration::SensorConfig imu;
        imu.type = "imu_node";
        imu.topic = "/imu/data";
        imu.frame_id = "imu_frame";
        imu.frequency = 100.0;
        imu.parameters["calibration_time"] = 10.0;
        configuration_->addSensor(imu);
        
        return *this;
    }
    
    RobotConfigurationBuilder& setNavigationStack() override {
        RobotConfiguration::NavigationConfig nav;
        nav.global_planner = "NavfnPlanner";
        nav.local_planner = "DWBLocalPlanner";
        nav.costmap_type = "costmap_2d";
        nav.parameters["controller_frequency"] = 20.0;
        nav.parameters["planner_patience"] = 5.0;
        nav.parameters["oscillation_timeout"] = 10.0;
        nav.parameters["oscillation_distance"] = 0.5;
        
        configuration_->setNavigation(nav);
        return *this;
    }
    
    RobotConfigurationBuilder& setHardwareInterface() override {
        RobotConfiguration::HardwareConfig hw;
        hw.driver_type = "turtlebot3_bringup";
        hw.device_path = "/dev/ttyACM0";
        hw.hardware_params["baud"] = "115200";
        hw.hardware_params["timeout"] = "1000";
        hw.hardware_params["model"] = "burger";
        
        configuration_->setHardware(hw);
        return *this;
    }
    
    RobotConfigurationBuilder& setRobotModel() override {
        configuration_->setRobotDescription(
            "$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf");
        return *this;
    }
    
    RobotConfigurationBuilder& addCustomComponents() override {
        // Add teleop
        configuration_->addLaunchFile("turtlebot3_teleop", "turtlebot3_teleop_key.launch");
        
        // Add visualization
        configuration_->addLaunchFile("turtlebot3_bringup", "turtlebot3_rviz.launch");
        
        return *this;
    }
    
    std::unique_ptr<RobotConfiguration> build() override {
        if (!configuration_->isValid()) {
            throw std::runtime_error("Invalid TurtleBot configuration");
        }
        
        auto result = std::move(configuration_);
        reset();  // Reset for next build
        return result;
    }
};

class IndustrialRobotBuilder : public RobotConfigurationBuilder {
public:
    IndustrialRobotBuilder() {
        reset();
    }
    
    RobotConfigurationBuilder& setSensorSuite() override {
        // Add Force/Torque Sensor
        RobotConfiguration::SensorConfig ft_sensor;
        ft_sensor.type = "robotiq_ft_sensor";
        ft_sensor.topic = "/ft_sensor/wrench";
        ft_sensor.frame_id = "ft_sensor_frame";
        ft_sensor.frequency = 125.0;
        ft_sensor.parameters["calibration_matrix"] = 1.0;
        ft_sensor.parameters["bias_compensation"] = 1.0;
        configuration_->addSensor(ft_sensor);
        
        // Add Vision System
        RobotConfiguration::SensorConfig vision;
        vision.type = "industrial_camera";
        vision.topic = "/camera/image_raw";
        vision.frame_id = "camera_frame";
        vision.frequency = 60.0;
        vision.parameters["exposure_time"] = 5000.0;
        vision.parameters["gain"] = 1.0;
        vision.parameters["trigger_mode"] = 1.0;
        configuration_->addSensor(vision);
        
        // Add Joint State Sensors
        RobotConfiguration::SensorConfig joint_states;
        joint_states.type = "joint_state_publisher";
        joint_states.topic = "/joint_states";
        joint_states.frame_id = "base_link";
        joint_states.frequency = 50.0;
        configuration_->addSensor(joint_states);
        
        return *this;
    }
    
    RobotConfigurationBuilder& setNavigationStack() override {
        // Industrial robots typically don't use mobile navigation
        // Instead, they use motion planning
        RobotConfiguration::NavigationConfig nav;
        nav.global_planner = "OMPL";
        nav.local_planner = "Pilz";
        nav.costmap_type = "collision_detection";
        nav.parameters["planning_time"] = 5.0;
        nav.parameters["max_velocity_scaling"] = 0.5;
        nav.parameters["max_acceleration_scaling"] = 0.3;
        
        configuration_->setNavigation(nav);
        return *this;
    }
    
    RobotConfigurationBuilder& setHardwareInterface() override {
        RobotConfiguration::HardwareConfig hw;
        hw.driver_type = "ur_robot_driver";
        hw.device_path = "192.168.1.100";  // IP address for industrial robots
        hw.hardware_params["robot_ip"] = "192.168.1.100";
        hw.hardware_params["kinematics_config"] = "/etc/ur_cal/ur5e_calibration.yaml";
        hw.hardware_params["use_tool_communication"] = "true";
        hw.hardware_params["tool_voltage"] = "24";
        
        configuration_->setHardware(hw);
        return *this;
    }
    
    RobotConfigurationBuilder& setRobotModel() override {
        configuration_->setRobotDescription(
            "$(find ur_description)/urdf/ur5e_robot.urdf.xacro");
        return *this;
    }
    
    RobotConfigurationBuilder& addCustomComponents() override {
        // Add MoveIt planning
        configuration_->addLaunchFile("ur5e_moveit_config", "ur5e_moveit_planning_execution.launch");
        
        // Add safety monitoring
        configuration_->addLaunchFile("ur_robot_driver", "ur5e_safety_monitor.launch");
        
        // Add tool control
        configuration_->addLaunchFile("robotiq_2f_gripper_control", "robotiq_action_server.launch");
        
        return *this;
    }
    
    std::unique_ptr<RobotConfiguration> build() override {
        if (!configuration_->isValid()) {
            throw std::runtime_error("Invalid Industrial Robot configuration");
        }
        
        auto result = std::move(configuration_);
        reset();
        return result;
    }
};

class DroneBuilder : public RobotConfigurationBuilder {
public:
    DroneBuilder() {
        reset();
    }
    
    RobotConfigurationBuilder& setSensorSuite() override {
        // Add GPS
        RobotConfiguration::SensorConfig gps;
        gps.type = "nmea_navsat_driver";
        gps.topic = "/gps/fix";
        gps.frame_id = "gps_frame";
        gps.frequency = 10.0;
        gps.parameters["port"] = 0;  // /dev/ttyUSB0
        gps.parameters["baud"] = 4800.0;
        configuration_->addSensor(gps);
        
        // Add IMU
        RobotConfiguration::SensorConfig imu;
        imu.type = "imu_node";
        imu.topic = "/imu/data";
        imu.frame_id = "imu_frame";
        imu.frequency = 200.0;
        imu.parameters["calibration_samples"] = 1000.0;
        configuration_->addSensor(imu);
        
        // Add Camera for visual odometry
        RobotConfiguration::SensorConfig camera;
        camera.type = "usb_cam";
        camera.topic = "/camera/image_raw";
        camera.frame_id = "camera_frame";
        camera.frequency = 30.0;
        camera.parameters["video_device"] = 0;
        camera.parameters["image_width"] = 1280.0;
        camera.parameters["image_height"] = 720.0;
        configuration_->addSensor(camera);
        
        // Add Barometer
        RobotConfiguration::SensorConfig baro;
        baro.type = "baro_node";
        baro.topic = "/barometer/pressure";
        baro.frame_id = "base_link";
        baro.frequency = 50.0;
        configuration_->addSensor(baro);
        
        return *this;
    }
    
    RobotConfigurationBuilder& setNavigationStack() override {
        RobotConfiguration::NavigationConfig nav;
        nav.global_planner = "PX4_mission_planner";
        nav.local_planner = "position_controller";
        nav.costmap_type = "3d_costmap";
        nav.parameters["max_velocity_horizontal"] = 5.0;
        nav.parameters["max_velocity_vertical"] = 3.0;
        nav.parameters["position_tolerance"] = 0.5;
        nav.parameters["yaw_tolerance"] = 0.1;
        
        configuration_->setNavigation(nav);
        return *this;
    }
    
    RobotConfigurationBuilder& setHardwareInterface() override {
        RobotConfiguration::HardwareConfig hw;
        hw.driver_type = "mavros";
        hw.device_path = "/dev/ttyACM0";
        hw.hardware_params["fcu_url"] = "/dev/ttyACM0:57600";
        hw.hardware_params["gcs_url"] = "udp://:14550@127.0.0.1:14557";
        hw.hardware_params["target_system_id"] = "1";
        hw.hardware_params["target_component_id"] = "1";
        
        configuration_->setHardware(hw);
        return *this;
    }
    
    RobotConfigurationBuilder& setRobotModel() override {
        configuration_->setRobotDescription(
            "$(find drone_description)/urdf/quadrotor.urdf.xacro");
        return *this;
    }
    
    RobotConfigurationBuilder& addCustomComponents() override {
        // Add MAVROS
        configuration_->addLaunchFile("mavros", "px4.launch");
        
        // Add visual odometry
        configuration_->addLaunchFile("vio_estimator", "vio_estimator.launch");
        
        // Add mission control
        configuration_->addLaunchFile("drone_control", "mission_control.launch");
        
        return *this;
    }
    
    std::unique_ptr<RobotConfiguration> build() override {
        if (!configuration_->isValid()) {
            throw std::runtime_error("Invalid Drone configuration");
        }
        
        auto result = std::move(configuration_);
        reset();
        return result;
    }
};

// 4. Director
class RobotConfigurationDirector {
public:
    void setBuilder(std::unique_ptr<RobotConfigurationBuilder> builder) {
        builder_ = std::move(builder);
    }
    
    std::unique_ptr<RobotConfiguration> buildBasicRobot() {
        if (!builder_) {
            throw std::runtime_error("No builder set");
        }
        
        return builder_->setSensorSuite()
                      .setHardwareInterface()
                      .setRobotModel()
                      .build();
    }
    
    std::unique_ptr<RobotConfiguration> buildFullyEquippedRobot() {
        if (!builder_) {
            throw std::runtime_error("No builder set");
        }
        
        return builder_->setSensorSuite()
                      .setNavigationStack()
                      .setHardwareInterface()
                      .setRobotModel()
                      .addCustomComponents()
                      .build();
    }
    
    std::unique_ptr<RobotConfiguration> buildCustomRobot(
        const std::function<RobotConfigurationBuilder&(RobotConfigurationBuilder&)>& customizer) {
        
        if (!builder_) {
            throw std::runtime_error("No builder set");
        }
        
        return customizer(*builder_).build();
    }

private:
    std::unique_ptr<RobotConfigurationBuilder> builder_;
};

// 5. ROS2 Integration
class RobotLauncher {
public:
    RobotLauncher(const rclcpp::Node::SharedPtr& node) : node_(node) {}
    
    bool launchRobot(const RobotConfiguration& config) {
        try {
            // Validate configuration
            if (!config.isValid()) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid robot configuration");
                return false;
            }
            
            // Generate launch file
            std::string launch_content = config.generateLaunchContent();
            std::string launch_file_path = "/tmp/robot_config.launch";
            
            // Write launch file
            std::ofstream launch_file(launch_file_path);
            if (!launch_file.is_open()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create launch file");
                return false;
            }
            launch_file << launch_content;
            launch_file.close();
            
            // Launch robot
            RCLCPP_INFO(node_->get_logger(), "Launching robot with configuration:");
            RCLCPP_INFO(node_->get_logger(), "  Sensors: %zu", config.getSensors().size());
            RCLCPP_INFO(node_->get_logger(), "  Navigation: %s", 
                config.getNavigation().global_planner.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Hardware: %s", 
                config.getHardware().driver_type.c_str());
            
            // Start launch process
            return startLaunchProcess(launch_file_path);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to launch robot: %s", e.what());
            return false;
        }
    }

private:
    bool startLaunchProcess(const std::string& launch_file) {
        // In real implementation, this would start the ROS2 launch process
        std::string command = "ros2 launch " + launch_file;
        RCLCPP_INFO(node_->get_logger(), "Executing: %s", command.c_str());
        
        // For demo purposes, just return true
        return true;
    }
    
    rclcpp::Node::SharedPtr node_;
};

// 6. Usage Example
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("robot_configuration_node");
    
    try {
        // Create director
        RobotConfigurationDirector director;
        
        // Example 1: Build TurtleBot
        RCLCPP_INFO(node->get_logger(), "Building TurtleBot configuration...");
        director.setBuilder(std::make_unique<TurtleBotBuilder>());
        auto turtlebot_config = director.buildFullyEquippedRobot();
        
        // Launch TurtleBot
        RobotLauncher launcher(node);
        launcher.launchRobot(*turtlebot_config);
        
        // Example 2: Build Industrial Robot
        RCLCPP_INFO(node->get_logger(), "Building Industrial Robot configuration...");
        director.setBuilder(std::make_unique<IndustrialRobotBuilder>());
        auto industrial_config = director.buildFullyEquippedRobot();
        
        // Example 3: Build Custom Drone
        RCLCPP_INFO(node->get_logger(), "Building Custom Drone configuration...");
        director.setBuilder(std::make_unique<DroneBuilder>());
        auto drone_config = director.buildCustomRobot([](RobotConfigurationBuilder& builder) -> RobotConfigurationBuilder& {
            return builder.setSensorSuite()
                         .setHardwareInterface()
                         .setRobotModel();
            // Skip navigation and custom components for basic drone
        });
        
        // Example 4: Runtime configuration based on parameters
        std::string robot_type = node->declare_parameter<std::string>("robot_type", "turtlebot");
        
        if (robot_type == "turtlebot") {
            director.setBuilder(std::make_unique<TurtleBotBuilder>());
        } else if (robot_type == "industrial") {
            director.setBuilder(std::make_unique<IndustrialRobotBuilder>());
        } else if (robot_type == "drone") {
            director.setBuilder(std::make_unique<DroneBuilder>());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Unknown robot type: %s", robot_type.c_str());
            return 1;
        }
        
        auto runtime_config = director.buildFullyEquippedRobot();
        launcher.launchRobot(*runtime_config);
        
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

1. **Product (RobotConfiguration)**:
   - Chứa tất cả thông tin cấu hình robot
   - Sensors, navigation, hardware, model
   - Validation và generation logic

2. **Builder Interface**:
   - Định nghĩa các bước xây dựng chuẩn
   - Fluent interface pattern
   - Method chaining

3. **Concrete Builders**:
   - TurtleBotBuilder: Mobile robot với LiDAR, camera
   - IndustrialRobotBuilder: Manipulator với force sensor
   - DroneBuilder: UAV với GPS, IMU, camera

4. **Director**:
   - Điều khiển quá trình xây dựng
   - Predefined build sequences
   - Custom build support

#### 5. Ưu điểm trong ROS2

1. **Flexibility**:
   - Dễ dàng tạo robot configurations khác nhau
   - Runtime configuration switching
   - Modular component assembly

2. **Maintainability**:
   - Clear separation of concerns
   - Reusable components
   - Standardized interfaces

3. **Extensibility**:
   - Dễ thêm robot types mới
   - Plugin architecture support
   - Configuration validation

#### 6. Các trường hợp sử dụng trong ROS2

1. **Robot Fleet Management**:
```cpp
class FleetConfigurationBuilder : public RobotConfigurationBuilder {
    std::vector<std::unique_ptr<RobotConfiguration>> fleet_;
    
public:
    FleetConfigurationBuilder& addRobot(const std::string& type) {
        // Add robot to fleet
        return *this;
    }
    
    FleetConfigurationBuilder& setFleetCommunication() {
        // Configure inter-robot communication
        return *this;
    }
    
    std::unique_ptr<FleetConfiguration> buildFleet() {
        // Return complete fleet configuration
        return std::make_unique<FleetConfiguration>(std::move(fleet_));
    }
};
```

2. **Dynamic Reconfiguration**:
```cpp
class ReconfigurableRobotBuilder : public RobotConfigurationBuilder {
public:
    RobotConfigurationBuilder& addSensorConditionally(
        const std::string& condition) {
        
        if (evaluateCondition(condition)) {
            // Add sensor based on runtime condition
        }
        return *this;
    }
    
    RobotConfigurationBuilder& setAdaptiveNavigation() {
        // Configure navigation based on environment
        return *this;
    }
};
```

3. **Multi-Environment Configurations**:
```cpp
class EnvironmentAwareBuilder : public RobotConfigurationBuilder {
    enum Environment { INDOOR, OUTDOOR, UNDERWATER, SPACE };
    Environment env_;
    
public:
    EnvironmentAwareBuilder(Environment env) : env_(env) {}
    
    RobotConfigurationBuilder& setSensorSuite() override {
        switch (env_) {
            case INDOOR:
                addIndoorSensors();
                break;
            case OUTDOOR:
                addOutdoorSensors();
                break;
            case UNDERWATER:
                addUnderwaterSensors();
                break;
            case SPACE:
                addSpaceSensors();
                break;
        }
        return *this;
    }
};
```

#### 7. Best Practices trong ROS2

1. **Configuration Validation**:
```cpp
class ValidatedBuilder : public RobotConfigurationBuilder {
    bool validateSensorCompatibility() {
        // Check sensor compatibility
        return true;
    }
    
    bool validateHardwareRequirements() {
        // Check hardware requirements
        return true;
    }
    
public:
    std::unique_ptr<RobotConfiguration> build() override {
        if (!validateSensorCompatibility()) {
            throw std::runtime_error("Sensor compatibility check failed");
        }
        
        if (!validateHardwareRequirements()) {
            throw std::runtime_error("Hardware requirements not met");
        }
        
        return std::move(configuration_);
    }
};
```

2. **Resource Management**:
```cpp
class ResourceAwareBuilder : public RobotConfigurationBuilder {
    struct ResourceLimits {
        double max_cpu_usage;
        double max_memory_usage;
        double max_network_bandwidth;
    };
    
    ResourceLimits limits_;
    
public:
    RobotConfigurationBuilder& setResourceLimits(const ResourceLimits& limits) {
        limits_ = limits;
        return *this;
    }
    
    RobotConfigurationBuilder& optimizeForResources() {
        // Adjust configuration based on resource limits
        return *this;
    }
};
```

3. **Error Handling**:
```cpp
class RobustBuilder : public RobotConfigurationBuilder {
public:
    RobotConfigurationBuilder& setSensorSuite() override {
        try {
            // Try to configure primary sensors
            configurePrimarySensors();
        } catch (const std::exception& e) {
            RCLCPP_WARN(logger_, "Primary sensors failed, using fallback: %s", e.what());
            configureFallbackSensors();
        }
        return *this;
    }
    
private:
    void configurePrimarySensors() {
        // Configure preferred sensors
    }
    
    void configureFallbackSensors() {
        // Configure fallback sensors
    }
    
    rclcpp::Logger logger_ = rclcpp::get_logger("RobustBuilder");
};
```

#### 8. Mở rộng và tùy chỉnh

1. **Plugin System**:
```cpp
class PluginBuilder : public RobotConfigurationBuilder {
    std::map<std::string, std::shared_ptr<ComponentPlugin>> plugins_;
    
public:
    RobotConfigurationBuilder& loadPlugin(const std::string& name) {
        auto plugin = loadComponentPlugin(name);
        plugins_[name] = plugin;
        return *this;
    }
    
    RobotConfigurationBuilder& configurePlugin(
        const std::string& name, const YAML::Node& config) {
        
        if (auto it = plugins_.find(name); it != plugins_.end()) {
            it->second->configure(config);
        }
        return *this;
    }
};
```

2. **Template-based Builders**:
```cpp
template<typename RobotType>
class TemplateRobotBuilder : public RobotConfigurationBuilder {
public:
    RobotConfigurationBuilder& setSpecializedComponents() {
        // Use template specialization for robot-specific components
        RobotType::configureSensors(*this);
        RobotType::configureNavigation(*this);
        return *this;
    }
};

// Specializations
struct TurtleBotType {
    static void configureSensors(RobotConfigurationBuilder& builder) {
        // TurtleBot-specific sensor configuration
    }
    
    static void configureNavigation(RobotConfigurationBuilder& builder) {
        // TurtleBot-specific navigation configuration
    }
};
```

3. **Configuration from Files**:
```cpp
class FileBasedBuilder : public RobotConfigurationBuilder {
public:
    RobotConfigurationBuilder& loadFromYAML(const std::string& file_path) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            
            // Parse sensors
            if (config["sensors"]) {
                for (const auto& sensor_config : config["sensors"]) {
                    addSensorFromYAML(sensor_config);
                }
            }
            
            // Parse navigation
            if (config["navigation"]) {
                setNavigationFromYAML(config["navigation"]);
            }
            
            // Parse hardware
            if (config["hardware"]) {
                setHardwareFromYAML(config["hardware"]);
            }
            
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to load configuration: " + std::string(e.what()));
        }
        
        return *this;
    }

private:
    void addSensorFromYAML(const YAML::Node& sensor_node) {
        RobotConfiguration::SensorConfig sensor;
        sensor.type = sensor_node["type"].as<std::string>();
        sensor.topic = sensor_node["topic"].as<std::string>();
        sensor.frame_id = sensor_node["frame_id"].as<std::string>();
        sensor.frequency = sensor_node["frequency"].as<double>();
        
        if (sensor_node["parameters"]) {
            for (const auto& param : sensor_node["parameters"]) {
                sensor.parameters[param.first.as<std::string>()] = 
                    param.second.as<double>();
            }
        }
        
        configuration_->addSensor(sensor);
    }
};
```

#### 9. Testing

1. **Builder Tests**:
```cpp
TEST(BuilderTest, TurtleBotBuilderTest) {
    TurtleBotBuilder builder;
    
    auto config = builder.setSensorSuite()
                        .setNavigationStack()
                        .setHardwareInterface()
                        .setRobotModel()
                        .build();
    
    ASSERT_TRUE(config->isValid());
    EXPECT_EQ(config->getSensors().size(), 3);  // LiDAR, Camera, IMU
    EXPECT_EQ(config->getNavigation().global_planner, "NavfnPlanner");
    EXPECT_EQ(config->getHardware().driver_type, "turtlebot3_bringup");
}
```

2. **Director Tests**:
```cpp
TEST(DirectorTest, BuilderSelectionTest) {
    RobotConfigurationDirector director;
    
    // Test TurtleBot
    director.setBuilder(std::make_unique<TurtleBotBuilder>());
    auto turtlebot = director.buildFullyEquippedRobot();
    EXPECT_EQ(turtlebot->getHardware().driver_type, "turtlebot3_bringup");
    
    // Test Industrial Robot
    director.setBuilder(std::make_unique<IndustrialRobotBuilder>());
    auto industrial = director.buildFullyEquippedRobot();
    EXPECT_EQ(industrial->getHardware().driver_type, "ur_robot_driver");
}
```

3. **Integration Tests**:
```cpp
TEST(IntegrationTest, LaunchConfigurationTest) {
    auto node = std::make_shared<rclcpp::Node>("test_node");
    RobotLauncher launcher(node);
    
    TurtleBotBuilder builder;
    auto config = builder.setSensorSuite()
                        .setHardwareInterface()
                        .setRobotModel()
                        .build();
    
    EXPECT_TRUE(launcher.launchRobot(*config));
}
```

#### 10. Kết luận

Builder Pattern là một mẫu thiết kế cực kỳ hữu ích trong ROS2 robotics, đặc biệt cho việc tạo các cấu hình robot phức tạp. Pattern này mang lại:

1. **Flexibility trong Configuration**:
   - Dễ dàng tạo nhiều robot types khác nhau
   - Runtime configuration switching
   - Modular component assembly

2. **Code Organization**:
   - Clear separation giữa construction logic
   - Reusable components
   - Standardized interfaces

3. **Maintainability**:
   - Dễ thêm robot types mới
   - Configuration validation
   - Error handling

4. **Integration với ROS2**:
   - Launch file generation
   - Parameter management
   - Node lifecycle management

Builder Pattern đặc biệt phù hợp cho các hệ thống robotics phức tạp cần quản lý nhiều components, sensors, và configurations khác nhau. Trong ví dụ trên, chúng ta đã thấy cách pattern này giúp xây dựng một hệ thống configuration linh hoạt và mạnh mẽ cho nhiều loại robot khác nhau.
