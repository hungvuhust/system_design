# Iterator Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Iterator Pattern là một behavioral pattern cho phép duyệt qua các phần tử của một collection mà không cần biết về cấu trúc bên trong của nó. Trong ROS2 và robotics, pattern này thường được sử dụng cho:

- Duyệt qua sensor data
- Xử lý point clouds
- Quản lý waypoints
- Lập kế hoạch đường đi
- Quản lý robot fleet
- Xử lý task queue

## 2. Vấn đề

Trong robotics, chúng ta thường gặp các tình huống cần:
- Truy cập tuần tự vào dữ liệu sensor
- Duyệt qua các điểm trong trajectory
- Xử lý từng điểm trong point cloud
- Quản lý nhiều robots
- Thực thi các tasks theo thứ tự
- Duyệt qua map data

## 3. Giải pháp

Iterator Pattern giải quyết các vấn đề trên bằng cách:
1. Tách logic duyệt khỏi cấu trúc dữ liệu
2. Cung cấp interface thống nhất để duyệt
3. Hỗ trợ nhiều cách duyệt khác nhau
4. Cho phép truy cập tuần tự mà không cần biết implementation

## 4. Ví dụ thực tế: Point Cloud Processing System

```cpp
// Point cloud data structure
struct Point {
    double x, y, z;
    std::vector<uint8_t> intensity;
    std::vector<uint8_t> color;
};

// Iterator interface
template<typename T>
class Iterator {
public:
    virtual ~Iterator() = default;
    virtual bool hasNext() const = 0;
    virtual T& next() = 0;
    virtual void reset() = 0;
};

// Collection interface
template<typename T>
class Collection {
public:
    virtual ~Collection() = default;
    virtual std::unique_ptr<Iterator<T>> createIterator() = 0;
    virtual void add(const T& item) = 0;
    virtual size_t size() const = 0;
};

// Concrete point cloud collection
class PointCloud : public Collection<Point> {
public:
    std::unique_ptr<Iterator<Point>> createIterator() override {
        return std::make_unique<PointCloudIterator>(*this);
    }

    void add(const Point& point) override {
        points_.push_back(point);
    }

    size_t size() const override {
        return points_.size();
    }

    // Additional point cloud specific methods
    void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg) {
        // Convert ROS message to internal format
        points_.clear();
        
        // Get field offsets
        size_t x_offset = 0, y_offset = 4, z_offset = 8;
        size_t intensity_offset = 16, color_offset = 20;
        
        // Extract points
        for (size_t i = 0; i < msg.width * msg.height; ++i) {
            Point point;
            
            // Extract position
            memcpy(&point.x, &msg.data[i * msg.point_step + x_offset], sizeof(float));
            memcpy(&point.y, &msg.data[i * msg.point_step + y_offset], sizeof(float));
            memcpy(&point.z, &msg.data[i * msg.point_step + z_offset], sizeof(float));
            
            // Extract intensity if available
            if (msg.fields.end() != std::find_if(msg.fields.begin(), msg.fields.end(),
                [](const sensor_msgs::msg::PointField& field) {
                    return field.name == "intensity";
                })) {
                uint8_t intensity;
                memcpy(&intensity, &msg.data[i * msg.point_step + intensity_offset], sizeof(uint8_t));
                point.intensity.push_back(intensity);
            }
            
            // Extract color if available
            if (msg.fields.end() != std::find_if(msg.fields.begin(), msg.fields.end(),
                [](const sensor_msgs::msg::PointField& field) {
                    return field.name == "rgb";
                })) {
                for (int j = 0; j < 3; ++j) {
                    uint8_t color;
                    memcpy(&color, &msg.data[i * msg.point_step + color_offset + j], sizeof(uint8_t));
                    point.color.push_back(color);
                }
            }
            
            points_.push_back(point);
        }
    }

    sensor_msgs::msg::PointCloud2 toROSMsg() const {
        sensor_msgs::msg::PointCloud2 msg;
        
        // Set basic header
        msg.header.frame_id = "base_link";
        msg.height = 1;
        msg.width = points_.size();
        
        // Define fields
        sensor_msgs::msg::PointField x_field;
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;
        msg.fields.push_back(x_field);
        
        sensor_msgs::msg::PointField y_field;
        y_field.name = "y";
        y_field.offset = 4;
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;
        msg.fields.push_back(y_field);
        
        sensor_msgs::msg::PointField z_field;
        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;
        msg.fields.push_back(z_field);
        
        // Add intensity and color fields if available
        if (!points_.empty() && !points_[0].intensity.empty()) {
            sensor_msgs::msg::PointField intensity_field;
            intensity_field.name = "intensity";
            intensity_field.offset = 16;
            intensity_field.datatype = sensor_msgs::msg::PointField::UINT8;
            intensity_field.count = 1;
            msg.fields.push_back(intensity_field);
        }
        
        if (!points_.empty() && !points_[0].color.empty()) {
            sensor_msgs::msg::PointField rgb_field;
            rgb_field.name = "rgb";
            rgb_field.offset = 20;
            rgb_field.datatype = sensor_msgs::msg::PointField::UINT8;
            rgb_field.count = 3;
            msg.fields.push_back(rgb_field);
        }
        
        // Set point step and row step
        msg.point_step = 32;  // Adjust based on fields
        msg.row_step = msg.point_step * msg.width;
        
        // Allocate memory for data
        msg.data.resize(msg.row_step * msg.height);
        
        // Copy points
        for (size_t i = 0; i < points_.size(); ++i) {
            const auto& point = points_[i];
            
            // Copy position
            memcpy(&msg.data[i * msg.point_step + 0], &point.x, sizeof(float));
            memcpy(&msg.data[i * msg.point_step + 4], &point.y, sizeof(float));
            memcpy(&msg.data[i * msg.point_step + 8], &point.z, sizeof(float));
            
            // Copy intensity if available
            if (!point.intensity.empty()) {
                memcpy(&msg.data[i * msg.point_step + 16], &point.intensity[0], sizeof(uint8_t));
            }
            
            // Copy color if available
            if (!point.color.empty()) {
                for (size_t j = 0; j < 3; ++j) {
                    memcpy(&msg.data[i * msg.point_step + 20 + j], &point.color[j], sizeof(uint8_t));
                }
            }
        }
        
        return msg;
    }

private:
    std::vector<Point> points_;

    // Iterator implementation
    class PointCloudIterator : public Iterator<Point> {
    public:
        explicit PointCloudIterator(const PointCloud& cloud)
            : cloud_(cloud), current_(0) {}

        bool hasNext() const override {
            return current_ < cloud_.points_.size();
        }

        Point& next() override {
            return const_cast<Point&>(cloud_.points_[current_++]);
        }

        void reset() override {
            current_ = 0;
        }

    private:
        const PointCloud& cloud_;
        size_t current_;
    };

    friend class PointCloudIterator;
};

// Filtered iterator example
class FilteredPointCloudIterator : public Iterator<Point> {
public:
    using FilterFunction = std::function<bool(const Point&)>;

    FilteredPointCloudIterator(const PointCloud& cloud, FilterFunction filter)
        : cloud_(cloud)
        , filter_(std::move(filter))
        , current_(0) {
        findNext();
    }

    bool hasNext() const override {
        return current_ < cloud_.size();
    }

    Point& next() override {
        Point& point = const_cast<Point&>(cloud_.points_[current_]);
        current_++;
        findNext();
        return point;
    }

    void reset() override {
        current_ = 0;
        findNext();
    }

private:
    void findNext() {
        while (current_ < cloud_.size() && !filter_(cloud_.points_[current_])) {
            current_++;
        }
    }

    const PointCloud& cloud_;
    FilterFunction filter_;
    size_t current_;
};

// Region iterator example
class RegionPointCloudIterator : public Iterator<Point> {
public:
    RegionPointCloudIterator(const PointCloud& cloud,
                            double min_x, double max_x,
                            double min_y, double max_y,
                            double min_z, double max_z)
        : cloud_(cloud)
        , min_x_(min_x), max_x_(max_x)
        , min_y_(min_y), max_y_(max_y)
        , min_z_(min_z), max_z_(max_z)
        , current_(0) {
        findNext();
    }

    bool hasNext() const override {
        return current_ < cloud_.size();
    }

    Point& next() override {
        Point& point = const_cast<Point&>(cloud_.points_[current_]);
        current_++;
        findNext();
        return point;
    }

    void reset() override {
        current_ = 0;
        findNext();
    }

private:
    void findNext() {
        while (current_ < cloud_.size()) {
            const auto& point = cloud_.points_[current_];
            if (point.x >= min_x_ && point.x <= max_x_ &&
                point.y >= min_y_ && point.y <= max_y_ &&
                point.z >= min_z_ && point.z <= max_z_) {
                break;
            }
            current_++;
        }
    }

    const PointCloud& cloud_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    size_t current_;
};
```

## 5. Sử dụng trong ROS2

Ví dụ về cách sử dụng Iterator Pattern trong một ROS2 node:

```cpp
class PointCloudProcessorNode : public rclcpp::Node {
public:
    PointCloudProcessorNode() : Node("point_cloud_processor") {
        // Create subscribers and publishers
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_cloud", 10,
            std::bind(&PointCloudProcessorNode::cloudCallback, this, std::placeholders::_1));

        filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "filtered_cloud", 10);

        region_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "region_cloud", 10);

        // Define processing parameters
        declare_parameter("min_intensity", 50);
        declare_parameter("region.min_x", -1.0);
        declare_parameter("region.max_x", 1.0);
        declare_parameter("region.min_y", -1.0);
        declare_parameter("region.max_y", 1.0);
        declare_parameter("region.min_z", 0.0);
        declare_parameter("region.max_z", 2.0);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert to our point cloud format
        PointCloud cloud;
        cloud.fromROSMsg(*msg);

        // Process using different iterators
        processWithFilter(cloud);
        processRegion(cloud);
    }

    void processWithFilter(const PointCloud& input_cloud) {
        // Create filtered cloud
        PointCloud filtered_cloud;
        
        // Get intensity threshold
        int min_intensity = get_parameter("min_intensity").as_int();

        // Create filtered iterator
        auto filter = [min_intensity](const Point& p) {
            return !p.intensity.empty() && p.intensity[0] >= min_intensity;
        };
        
        FilteredPointCloudIterator it(input_cloud, filter);

        // Process points
        while (it.hasNext()) {
            filtered_cloud.add(it.next());
        }

        // Publish filtered cloud
        auto filtered_msg = filtered_cloud.toROSMsg();
        filtered_msg.header = msg->header;
        filtered_pub_->publish(filtered_msg);
    }

    void processRegion(const PointCloud& input_cloud) {
        // Create region cloud
        PointCloud region_cloud;
        
        // Get region parameters
        double min_x = get_parameter("region.min_x").as_double();
        double max_x = get_parameter("region.max_x").as_double();
        double min_y = get_parameter("region.min_y").as_double();
        double max_y = get_parameter("region.max_y").as_double();
        double min_z = get_parameter("region.min_z").as_double();
        double max_z = get_parameter("region.max_z").as_double();

        // Create region iterator
        RegionPointCloudIterator it(input_cloud,
            min_x, max_x, min_y, max_y, min_z, max_z);

        // Process points
        while (it.hasNext()) {
            region_cloud.add(it.next());
        }

        // Publish region cloud
        auto region_msg = region_cloud.toROSMsg();
        region_msg.header = msg->header;
        region_pub_->publish(region_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr region_pub_;
};
```

## 6. Lợi ích

1. **Separation of Concerns**:
   - Tách logic duyệt khỏi cấu trúc dữ liệu
   - Dễ thêm iterators mới
   - Clean code organization

2. **Flexibility**:
   - Nhiều cách duyệt khác nhau
   - Dễ thay đổi cách duyệt
   - Không ảnh hưởng đến collection

3. **Simplicity**:
   - Interface đơn giản
   - Dễ implement
   - Dễ sử dụng

## 7. Khi nào sử dụng

- Cần duyệt qua collection phức tạp
- Muốn nhiều cách duyệt khác nhau
- Cần tách logic duyệt khỏi collection
- Xử lý dữ liệu theo stream
- Cần filter hoặc transform data khi duyệt
- Làm việc với large datasets

## 8. Lưu ý

1. Thiết kế:
   - Xác định rõ iterator interface
   - Cân nhắc performance
   - Handle edge cases
   - Consider thread safety

2. Implementation:
   - Memory management
   - Error handling
   - Iterator invalidation
   - Performance optimization

3. Trong ROS2:
   - Message conversion overhead
   - Real-time considerations
   - Resource management
   - Thread safety 