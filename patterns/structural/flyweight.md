# Flyweight Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Flyweight Pattern là một mẫu thiết kế thuộc nhóm Structural Pattern, giúp giảm thiểu việc sử dụng bộ nhớ bằng cách chia sẻ các trạng thái chung giữa nhiều đối tượng thay vì lưu trữ chúng trong từng đối tượng.

Trong ROS2 và robotics, Flyweight Pattern thường được sử dụng để:
- Quản lý dữ liệu cảm biến từ nhiều robot
- Chia sẻ các tham số cấu hình chung
- Tối ưu hóa bộ nhớ trong point cloud processing
- Quản lý các shared resources trong multi-robot system

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần tạo nhiều đối tượng có các thuộc tính giống nhau, ví dụ:
- Nhiều robot cùng loại trong swarm robotics
- Các điểm trong point cloud với cùng thuộc tính
- Các particle trong particle filter
- Các node trong occupancy grid map

Việc lưu trữ riêng các thuộc tính chung này sẽ dẫn đến:
- Sử dụng bộ nhớ không hiệu quả
- Tăng thời gian xử lý
- Giảm hiệu suất hệ thống

## 3. Giải pháp

Flyweight Pattern giải quyết vấn đề bằng cách:
1. Tách các thuộc tính thành intrinsic (shared) và extrinsic (unique)
2. Tạo factory để quản lý và chia sẻ các flyweight objects
3. Sử dụng caching để tái sử dụng các đối tượng

## 4. Ví dụ thực tế: Point Cloud Processing

```cpp
// Intrinsic state - shared properties
class PointProperties {
public:
    PointProperties(uint8_t r, uint8_t g, uint8_t b, std::string semantic_label)
        : color_r_(r), color_g_(g), color_b_(b)
        , semantic_label_(semantic_label) {}

    // Getters
    uint8_t getR() const { return color_r_; }
    uint8_t getG() const { return color_g_; }
    uint8_t getB() const { return color_b_; }
    const std::string& getLabel() const { return semantic_label_; }

private:
    uint8_t color_r_, color_g_, color_b_;
    std::string semantic_label_;
};

// Flyweight factory
class PointPropertiesFactory {
public:
    std::shared_ptr<PointProperties> getPointProperties(
        uint8_t r, uint8_t g, uint8_t b, const std::string& label) {
        
        // Create key for properties
        std::string key = std::to_string(r) + "_" + 
                         std::to_string(g) + "_" + 
                         std::to_string(b) + "_" + 
                         label;

        // Check if properties already exist
        auto it = properties_.find(key);
        if (it != properties_.end()) {
            return it->second;
        }

        // Create new properties if not found
        auto props = std::make_shared<PointProperties>(r, g, b, label);
        properties_[key] = props;
        return props;
    }

    size_t getCacheSize() const {
        return properties_.size();
    }

private:
    std::unordered_map<std::string, std::shared_ptr<PointProperties>> properties_;
};

// Point class with extrinsic state
class Point {
public:
    Point(double x, double y, double z,
          std::shared_ptr<PointProperties> properties)
        : x_(x), y_(y), z_(z)
        , properties_(properties) {}

    // Getters
    double getX() const { return x_; }
    double getY() const { return y_; }
    double getZ() const { return z_; }
    const PointProperties& getProperties() const { return *properties_; }

private:
    // Extrinsic state - unique for each point
    double x_, y_, z_;
    // Intrinsic state - shared between points
    std::shared_ptr<PointProperties> properties_;
};

// Point cloud using flyweight pattern
class OptimizedPointCloud {
public:
    OptimizedPointCloud() : factory_(std::make_unique<PointPropertiesFactory>()) {}

    void addPoint(double x, double y, double z,
                 uint8_t r, uint8_t g, uint8_t b,
                 const std::string& label) {
        auto props = factory_->getPointProperties(r, g, b, label);
        points_.emplace_back(x, y, z, props);
    }

    size_t getNumPoints() const {
        return points_.size();
    }

    size_t getNumUniqueProperties() const {
        return factory_->getCacheSize();
    }

    void processPoints() {
        for (const auto& point : points_) {
            RCLCPP_DEBUG(logger_, "Processing point (%.2f, %.2f, %.2f) with label %s",
                        point.getX(), point.getY(), point.getZ(),
                        point.getProperties().getLabel().c_str());
            // Process point...
        }
    }

private:
    std::vector<Point> points_;
    std::unique_ptr<PointPropertiesFactory> factory_;
    rclcpp::Logger logger_ = rclcpp::get_logger("OptimizedPointCloud");
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Flyweight Pattern trong một ROS2 node xử lý point cloud:

```cpp
class PointCloudProcessorNode : public rclcpp::Node {
public:
    PointCloudProcessorNode() : Node("point_cloud_processor") {
        // Khởi tạo point cloud
        point_cloud_ = std::make_unique<OptimizedPointCloud>();

        // Subscribe to point cloud topic
        point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", 10,
            std::bind(&PointCloudProcessorNode::pointCloudCallback, this, std::placeholders::_1));

        // Timer để in thống kê
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PointCloudProcessorNode::publishStats, this));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Process points using flyweight pattern
        for (const auto& pcl_point : pcl_cloud) {
            point_cloud_->addPoint(
                pcl_point.x, pcl_point.y, pcl_point.z,
                pcl_point.r, pcl_point.g, pcl_point.b,
                determineLabel(pcl_point));
        }

        // Process the optimized point cloud
        point_cloud_->processPoints();
    }

    void publishStats() {
        RCLCPP_INFO(get_logger(),
            "Point cloud stats:\n"
            "  Total points: %zu\n"
            "  Unique properties: %zu",
            point_cloud_->getNumPoints(),
            point_cloud_->getNumUniqueProperties());
    }

    std::string determineLabel(const pcl::PointXYZRGB& point) {
        // Simplified label determination based on height
        if (point.z > 1.5) return "high";
        if (point.z > 0.5) return "medium";
        return "low";
    }

    std::unique_ptr<OptimizedPointCloud> point_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};
```

## 6. Lợi ích

1. **Tiết kiệm bộ nhớ**: Giảm dung lượng bộ nhớ bằng cách chia sẻ dữ liệu
2. **Hiệu suất**: Giảm thời gian tạo đối tượng và truy cập dữ liệu
3. **Quản lý tốt hơn**: Tập trung quản lý các thuộc tính chung
4. **Scalability**: Dễ dàng mở rộng với số lượng lớn đối tượng

## 7. Khi nào sử dụng

- Khi ứng dụng cần tạo số lượng lớn đối tượng
- Khi nhiều đối tượng chia sẻ thuộc tính giống nhau
- Khi cần tối ưu hóa việc sử dụng bộ nhớ
- Khi các thuộc tính chung ít thay đổi

## 8. Lưu ý

1. Thiết kế Flyweight:
   - Phân biệt rõ intrinsic và extrinsic state
   - Đảm bảo flyweight objects là immutable
   - Cẩn thận với thread safety trong factory

2. Quản lý bộ nhớ:
   - Cân nhắc kích thước cache trong factory
   - Xử lý việc giải phóng bộ nhớ khi không cần thiết
   - Theo dõi memory usage

3. Trong ROS2:
   - Sử dụng shared_ptr để quản lý flyweight objects
   - Cẩn thận với real-time constraints
   - Xem xét trade-off giữa memory và CPU usage 