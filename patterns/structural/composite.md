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

## 3. Ví dụ trong ROS2

Hãy xem xét một ví dụ về việc mô hình hóa một robot di động tự hành (AMR - Autonomous Mobile Robot) sử dụng Composite Pattern.

### 3.1. Component Interface

Đầu tiên, chúng ta định nghĩa một `Component` chung cho tất cả các bộ phận của robot.

```python
# robot_component.py
from abc import ABC, abstractmethod

class RobotComponent(ABC):
    """Component Interface"""
    def __init__(self, name):
        self._name = name
        self._parent = None

    def set_parent(self, parent):
        self._parent = parent

    def get_parent(self):
        return self._parent

    def add(self, component: 'RobotComponent'):
        pass

    def remove(self, component: 'RobotComponent'):
        pass

    def is_composite(self):
        return False

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def shutdown(self):
        pass

    @abstractmethod
    def get_status(self):
        pass
```

### 3.2. Leaf Components

Các `Leaf` là các bộ phận cụ thể của robot.

```python
# leaf_components.py
from robot_component import RobotComponent

class LidarSensor(RobotComponent):
    """Leaf"""
    def initialize(self):
        print(f"Initializing LiDAR sensor: {self._name}")
        # Code để khởi tạo kết nối với LiDAR, đăng ký ROS2 publisher/subscriber

    def shutdown(self):
        print(f"Shutting down LiDAR sensor: {self._name}")

    def get_status(self):
        return f"LiDAR {self._name}: OK"

class CameraSensor(RobotComponent):
    """Leaf"""
    def initialize(self):
        print(f"Initializing Camera sensor: {self._name}")

    def shutdown(self):
        print(f"Shutting down Camera sensor: {self._name}")

    def get_status(self):
        return f"Camera {self._name}: OK"

class MotorController(RobotComponent):
    """Leaf"""
    def initialize(self):
        print(f"Initializing Motor Controller: {self._name}")

    def shutdown(self):
        print(f"Shutting down Motor Controller: {self._name}")

    def get_status(self):
        return f"Motor Controller {self._name}: Running"
```

### 3.3. Composite Components

Các `Composite` là các hệ thống con, chứa các `Leaf` hoặc các `Composite` khác.

```python
# composite_components.py
from typing import List
from robot_component import RobotComponent

class Composite(RobotComponent):
    """Composite"""
    def __init__(self, name):
        super().__init__(name)
        self._children: List[RobotComponent] = []

    def add(self, component: RobotComponent):
        self._children.append(component)
        component.set_parent(self)

    def remove(self, component: RobotComponent):
        self._children.remove(component)
        component.set_parent(None)

    def is_composite(self):
        return True

    def initialize(self):
        print(f"Initializing composite component: {self._name}")
        for child in self._children:
            child.initialize()

    def shutdown(self):
        print(f"Shutting down composite component: {self._name}")
        for child in self._children:
            child.shutdown()

    def get_status(self):
        results = []
        for child in self._children:
            results.append(child.get_status())
        return f"Composite {self._name}: [{', '.join(results)}]"

# Cụ thể hóa các Composite
class SensorSystem(Composite):
    """Composite for sensors"""
    pass

class DriveSystem(Composite):
    """Composite for drive system"""
    pass

class AutonomousMobileRobot(Composite):
    """The top-level composite"""
    pass
```

### 3.4. Client Code

Client sẽ xây dựng cây cấu trúc robot và tương tác với nó.

```python
# main.py
from leaf_components import LidarSensor, CameraSensor, MotorController
from composite_components import SensorSystem, DriveSystem, AutonomousMobileRobot

if __name__ == "__main__":
    # 1. Xây dựng cây cấu trúc robot
    robot = AutonomousMobileRobot("MyAMR")

    # Hệ thống cảm biến
    sensor_system = SensorSystem("SensorSystem")
    sensor_system.add(LidarSensor("Lidar_Front"))
    sensor_system.add(CameraSensor("Camera_Stereo"))

    # Hệ thống truyền động
    drive_system = DriveSystem("DriveSystem")
    drive_system.add(MotorController("Motor_Left"))
    drive_system.add(MotorController("Motor_Right"))

    # Thêm các hệ thống con vào robot
    robot.add(sensor_system)
    robot.add(drive_system)

    # 2. Tương tác với robot
    print("--- Initializing Robot ---")
    robot.initialize()

    print("
--- Getting Robot Status ---")
    print(robot.get_status())

    print("
--- Shutting Down Robot ---")
    robot.shutdown()
```

### Kết quả chạy:
```
--- Initializing Robot ---
Initializing composite component: MyAMR
Initializing composite component: SensorSystem
Initializing LiDAR sensor: Lidar_Front
Initializing Camera sensor: Camera_Stereo
Initializing composite component: DriveSystem
Initializing Motor Controller: Motor_Left
Initializing Motor Controller: Motor_Right

--- Getting Robot Status ---
Composite MyAMR: [Composite SensorSystem: [LiDAR Lidar_Front: OK, Camera Camera_Stereo: OK], Composite DriveSystem: [Motor Controller Motor_Left: Running, Motor Controller Motor_Right: Running]]

--- Shutting Down Robot ---
Shutting down composite component: MyAMR
Shutting down composite component: SensorSystem
Shutting down LiDAR sensor: Lidar_Front
Shutting down Camera sensor: Camera_Stereo
Shutting down composite component: DriveSystem
Shutting down Motor Controller: Motor_Left
Shutting down Motor Controller: Motor_Right
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
