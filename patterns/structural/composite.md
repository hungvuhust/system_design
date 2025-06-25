# Composite Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Composite Pattern là một mẫu thiết kế cấu trúc (Structural Design Pattern) cho phép bạn kết hợp các đối tượng thành các cấu trúc cây để biểu diễn các hệ thống phân cấp một phần-toàn bộ (part-whole hierarchies). Composite cho phép client xử lý các đối tượng riêng lẻ và các thành phần của đối tượng một cách thống nhất.

Trong ROS2 và robotics, Composite Pattern rất hữu ích để quản lý các cấu trúc phức tạp như một robot được tạo thành từ nhiều bộ phận, mỗi bộ phận lại có thể có các bộ phận con. Ví dụ, một cánh tay robot có thể được coi là một "composite" bao gồm các khớp (joints), và các thanh nối (links), trong khi mỗi khớp lại là một đối tượng riêng lẻ.

**Ví dụ:**
- Một robot di động (mobile robot) là một composite của hệ thống truyền động (drive system), hệ thống cảm biến (sensor system), và hệ thống tính toán (computation system).
- Hệ thống cảm biến lại là một composite của nhiều cảm biến riêng lẻ như LiDAR, camera, IMU.

## 2. Cấu trúc

Composite Pattern bao gồm các thành phần sau:

- **Component:**
  - Là một interface hoặc abstract class cho tất cả các đối tượng trong composition.
  - Khai báo các phương thức chung cho cả `Leaf` (đối tượng riêng lẻ) và `Composite` (đối tượng chứa các đối tượng khác).
  - Ví dụ: `RobotComponent` với các phương thức như `initialize()`, `shutdown()`, `get_status()`.

- **Leaf:**
  - Là các đối tượng riêng lẻ trong composition, không có con.
  - Implement các phương thức của `Component`.
  - Ví dụ: `LidarSensor`, `CameraSensor`, `MotorController`.

- **Composite:**
  - Là đối tượng chứa các `Component` con (có thể là `Leaf` hoặc `Composite` khác).
  - Implement các phương thức của `Component`, thường bằng cách ủy thác (delegate) cho các con của nó.
  - Cung cấp các phương thức để quản lý các con, như `add()`, `remove()`, `get_child()`.
  - Ví dụ: `SensorSystem` (chứa các `LidarSensor`, `CameraSensor`), `RobotPlatform` (chứa `SensorSystem`, `DriveSystem`).

- **Client:**
  - Tương tác với các đối tượng trong composition thông qua interface `Component`.
  - Client không cần phân biệt giữa `Leaf` và `Composite`, giúp đơn giản hóa code.

## 3. Ví dụ trong C++ (ROS2)

Hãy xem xét một ví dụ về việc mô hình hóa một robot di động tự hành (AMR - Autonomous Mobile Robot) sử dụng Composite Pattern trong C++.

### 3.1. Component Interface (`robot_component.hpp`)

```cpp
#pragma once

class RobotComponent {
protected:
    std::string name_;
    RobotComponent* parent_ = nullptr;

public:
    RobotComponent(const std::string& name) : name_(name) {}
    virtual ~RobotComponent() = default;

    void set_parent(RobotComponent* parent) {
        parent_ = parent;
    }

    RobotComponent* get_parent() const {
        return parent_;
    }

    virtual void add(std::unique_ptr<RobotComponent> component) {}
    virtual void remove(RobotComponent* component) {}
    virtual bool is_composite() const { return false; }

    virtual void initialize() = 0;
    virtual void shutdown() = 0;
    virtual std::string get_status() = 0;
    
    std::string get_name() const { return name_; }
};
```

### 3.2. Leaf Components (`leaf_components.hpp`)

```cpp
#pragma once

// Leaf: LidarSensor
class LidarSensor : public RobotComponent {
public:
    using RobotComponent::RobotComponent; // Inherit constructor

    void initialize() override {
        std::cout << "Initializing LiDAR sensor: " << name_ << std::endl;
    }

    void shutdown() override {
        std::cout << "Shutting down LiDAR sensor: " << name_ << std::endl;
    }

    std::string get_status() override {
        return "LiDAR " + name_ + ": OK";
    }
};

// Leaf: CameraSensor
class CameraSensor : public RobotComponent {
public:
    using RobotComponent::RobotComponent;

    void initialize() override {
        std::cout << "Initializing Camera sensor: " << name_ << std::endl;
    }

    void shutdown() override {
        std::cout << "Shutting down Camera sensor: " << name_ << std::endl;
    }

    std::string get_status() override {
        return "Camera " + name_ + ": OK";
    }
};

// Leaf: MotorController
class MotorController : public RobotComponent {
public:
    using RobotComponent::RobotComponent;

    void initialize() override {
        std::cout << "Initializing Motor Controller: " << name_ << std::endl;
    }

    void shutdown() override {
        std::cout << "Shutting down Motor Controller: " << name_ << std::endl;
    }

    std::string get_status() override {
        return "Motor Controller " + name_ + ": Running";
    }
};
```

### 3.3. Composite Component (`composite_component.hpp`)

```cpp
#pragma once

class Composite : public RobotComponent {
protected:
    std::vector<std::unique_ptr<RobotComponent>> children_;

public:
    using RobotComponent::RobotComponent;

    void add(std::unique_ptr<RobotComponent> component) override {
        component->set_parent(this);
        children_.push_back(std::move(component));
    }

    void remove(RobotComponent* component) override {
        children_.erase(
            std::remove_if(children_.begin(), children_.end(), 
                [&](const std::unique_ptr<RobotComponent>& p) {
                    return p.get() == component;
                }),
            children_.end()
        );
    }

    bool is_composite() const override {
        return true;
    }

    void initialize() override {
        std::cout << "Initializing composite component: " << name_ << std::endl;
        for (const auto& child : children_) {
            child->initialize();
        }
    }

    void shutdown() override {
        std::cout << "Shutting down composite component: " << name_ << std::endl;
        for (const auto& child : children_) {
            child->shutdown();
        }
    }

    std::string get_status() override {
        std::string result = "Composite " + name_ + ": [";
        for (size_t i = 0; i < children_.size(); ++i) {
            result += children_[i]->get_status();
            if (i < children_.size() - 1) {
                result += ", ";
            }
        }
        result += "]";
        return result;
    }
};
```

### 3.4. Client Code (`main.cpp`)

```cpp
int main() {
    // 1. Build the robot's component tree
    auto robot = std::make_unique<Composite>("MyAMR");

    // Sensor System
    auto sensor_system = std::make_unique<Composite>("SensorSystem");
    sensor_system->add(std::make_unique<LidarSensor>("Lidar_Front"));
    sensor_system->add(std::make_unique<CameraSensor>("Camera_Stereo"));

    // Drive System
    auto drive_system = std::make_unique<Composite>("DriveSystem");
    drive_system->add(std::make_unique<MotorController>("Motor_Left"));
    drive_system->add(std::make_unique<MotorController>("Motor_Right"));

    // Add subsystems to the robot
    robot->add(std::move(sensor_system));
    robot->add(std::move(drive_system));

    // 2. Interact with the robot
    std::cout << "--- Initializing Robot ---" << std::endl;
    robot->initialize();

    std::cout << "\n--- Getting Robot Status ---" << std::endl;
    std::cout << robot->get_status() << std::endl;

    std::cout << "\n--- Shutting Down Robot ---" << std::endl;
    robot->shutdown();

    return 0;
}
```

## 4. Best Practices

- **Interface Segregation:** Giữ cho `Component` interface ở mức tối thiểu. Các phương thức quản lý con (`add`, `remove`) có thể được đặt trong `Composite` class thay vì `Component` interface để tránh làm "ô nhiễm" `Leaf` class (Leaf không cần các phương thức này). Tuy nhiên, việc đặt chúng trong `Component` giúp client code đơn giản hơn (không cần kiểm tra kiểu).
- **Parent Pointers:** Cung cấp một con trỏ (pointer) từ `Component` đến `Composite` cha của nó. Điều này hữu ích cho việc duyệt cây hoặc thực hiện các hành động yêu cầu ngữ cảnh của cha.
- **ROS2 Integration:**
  - Mỗi `Composite` hoặc `Leaf` có thể là một ROS2 Node hoặc quản lý một tập các Node.
  - Sử dụng ROS2 parameters để cấu hình các `Component` (ví dụ: tên topic, tần số publish).
  - Sử dụng ROS2 services hoặc actions để kích hoạt các hành động trên `Component` (ví dụ: `start_scan` cho LiDAR).
- **Lifecycle Management:** Tích hợp với ROS2 Lifecycle Nodes. Các phương thức `initialize()` và `shutdown()` có thể được ánh xạ tới các transition `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup` của Lifecycle Node.

## 5. Mở rộng

- **Shared Leafs:** Để tiết kiệm bộ nhớ, các `Leaf` có thể được chia sẻ giữa nhiều `Composite` nếu chúng không có trạng thái riêng.
- **Ordering:** Trong một số trường hợp, thứ tự của các con trong `Composite` là quan trọng. `Composite` class nên sử dụng một cấu trúc dữ liệu có thứ tự (như list) để lưu trữ các con.

## 6. Testing

- **Unit Test:**
  - Test các `Leaf` một cách độc lập.
  - Test các `Composite` bằng cách sử dụng các mock `Component` để kiểm tra xem nó có ủy thác các cuộc gọi một cách chính xác hay không.
- **Integration Test:**
  - Xây dựng một cây `Composite` đơn giản và kiểm tra xem toàn bộ hệ thống có hoạt động như mong đợi không.
  - Test trong môi trường ROS2 để đảm bảo giao tiếp giữa các node là chính xác.

## 7. Use Cases trong Robotics

- **Robot Arm:** Một cánh tay robot là một `Composite` của các `Joint` và `Link`. Mỗi `Joint` có thể là một `Composite` khác nếu nó phức tạp (ví dụ: khớp 2 bậc tự do).
- **Simulation Models (URDF/SDF):** Cấu trúc của một robot trong file URDF hoặc SDF có thể được biểu diễn bằng Composite Pattern. `<robot>` là `Composite` gốc, `<link>` và `<joint>` là các `Component`.
- **Task Planning:** Một nhiệm vụ phức tạp (ví dụ: "lấy và đặt") có thể được chia thành các nhiệm vụ con (`Composite`). Các nhiệm vụ nguyên thủy (ví dụ: "di chuyển đến vị trí", "đóng kẹp") là các `Leaf`.
- **Software Architecture:** Cấu trúc của một hệ thống phần mềm robot có thể được tổ chức bằng Composite. Ví dụ, một `NavigationSystem` là một `Composite` của `PathPlanner`, `LocalPlanner`, và `RecoveryBehavior`.
