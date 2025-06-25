# Decorator Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Decorator là một mẫu thiết kế cấu trúc (Structural Design Pattern) cho phép bạn thêm các hành vi mới vào các đối tượng một cách linh hoạt bằng cách đặt chúng bên trong các đối tượng "wrapper" đặc biệt. Wrapper này chứa cùng một interface với đối tượng gốc.

Trong ROS2 và robotics, Decorator Pattern cực kỳ hữu ích để mở rộng chức năng của một component mà không cần thay đổi code của nó. Đây là một giải pháp thay thế cho việc kế thừa (subclassing) để mở rộng chức năng.

**Ví dụ:**
- Gói một `PathPlanner` cơ bản với một `TimingDecorator` để đo thời gian thực thi.
- Thêm một `SafetyDecorator` vào một `MotorController` để kiểm tra các giới hạn an toàn trước khi gửi lệnh.
- Gói một `SensorPublisher` với một `FilterDecorator` để lọc nhiễu từ dữ liệu cảm biến trước khi publish.

## 2. Cấu trúc

Decorator Pattern có các thành phần sau:

- **Component:**
  - Là một interface chung cho cả đối tượng gốc (concrete component) và các decorator.
  - Ví dụ: `PathPlanner` interface với phương thức `plan()`.

- **Concrete Component:**
  - Là class của đối tượng gốc mà chúng ta muốn thêm chức năng.
  - Implement interface `Component`.
  - Ví dụ: `AStarPlanner`, `RRTPlanner`.

- **Decorator:**
  - Là một abstract class đóng vai trò là base class cho các concrete decorator.
  - Chứa một tham chiếu đến một đối tượng `Component` (có thể là concrete component hoặc một decorator khác).
  - Ủy thác (delegate) tất cả các lời gọi đến đối tượng `Component` mà nó gói.

- **Concrete Decorator:**
  - Là các class wrapper cụ thể, implement chức năng bổ sung.
  - Chúng thực thi hành vi của mình trước hoặc sau khi ủy thác lời gọi đến đối tượng được gói.
  - Ví dụ: `TimingDecorator`, `LoggingDecorator`, `SafetyCheckDecorator`.

- **Client:**
  - Có thể gói các component nhiều lần với các decorator khác nhau.

## 3. Ví dụ trong ROS2

Giả sử chúng ta có một hệ thống hoạch định đường đi (`PathPlanner`) và muốn thêm chức năng đo thời gian và ghi log mà không sửa đổi các planner gốc.

### 3.1. Component Interface

```python
# path_planner.py
from abc import ABC, abstractmethod
from typing import List, Tuple

class PathPlanner(ABC):
    """Component Interface"""
    @abstractmethod
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        pass
```

### 3.2. Concrete Component

Đây là planner gốc, thực hiện thuật toán A*.

```python
# concrete_planners.py
from path_planner import PathPlanner
from typing import List, Tuple

class AStarPlanner(PathPlanner):
    """Concrete Component"""
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        print(f"Planning path from {start} to {goal} using A* algorithm.")
        # Giả lập một thuật toán phức tạp
        path = [start, (start[0], goal[1]), goal]
        print("Path found.")
        return path
```

### 3.3. Decorator Base Class

```python
# planner_decorator.py
from path_planner import PathPlanner
from typing import List, Tuple

class PlannerDecorator(PathPlanner):
    """Decorator Base Class"""
    _planner: PathPlanner = None

    def __init__(self, planner: PathPlanner) -> None:
        self._planner = planner

    @property
    def planner(self) -> PathPlanner:
        return self._planner

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        return self._planner.plan(start, goal)
```

### 3.4. Concrete Decorators

Các decorator này thêm chức năng đo thời gian và ghi log.

```python
# concrete_decorators.py
from planner_decorator import PlannerDecorator
from typing import List, Tuple
import time

class TimingDecorator(PlannerDecorator):
    """Concrete Decorator for timing"""
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        start_time = time.time()
        result = super().plan(start, goal)
        end_time = time.time()
        print(f"[TimingDecorator] Planning took {end_time - start_time:.4f} seconds.")
        return result

class LoggingDecorator(PlannerDecorator):
    """Concrete Decorator for logging"""
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        print(f"[LoggingDecorator] Calling planner for path from {start} to {goal}.")
        result = super().plan(start, goal)
        print(f"[LoggingDecorator] Planner finished execution. Path has {len(result)} points.")
        return result
```

### 3.5. Client Code

Client có thể kết hợp các decorator một cách linh hoạt.

```python
# main.py
from concrete_planners import AStarPlanner
from concrete_decorators import TimingDecorator, LoggingDecorator

if __name__ == "__main__":
    # 1. Planner gốc
    simple_planner = AStarPlanner()
    print("--- Simple Planner ---")
    path1 = simple_planner.plan((0, 0), (10, 10))
    print(f"Resulting path: {path1}")

    # 2. Gói planner với TimingDecorator
    timed_planner = TimingDecorator(simple_planner)
    print("\n--- Timed Planner ---")
    path2 = timed_planner.plan((0, 0), (20, 20))
    print(f"Resulting path: {path2}")

    # 3. Gói planner với cả hai decorator
    # Thứ tự gói quan trọng: Logging -> Timing -> Planner
    logged_timed_planner = LoggingDecorator(TimingDecorator(AStarPlanner()))
    print("\n--- Logged and Timed Planner ---")
    path3 = logged_timed_planner.plan((5, 5), (25, 25))
    print(f"Resulting path: {path3}")
```

### 3.6. Ví dụ trong C++ (ROS2)

Tương tự, chúng ta có thể implement Decorator pattern trong C++ sử dụng smart pointers để quản lý ownership.

#### Component Interface (`path_planner.hpp`)
```cpp
#pragma once
#include <vector>
#include <utility>
#include <memory>
#include <iostream>

// Component Interface
class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    virtual std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal) = 0;
};
```

#### Concrete Component (`astar_planner.hpp`)
```cpp
#pragma once
#include "path_planner.hpp"

// Concrete Component
class AStarPlanner : public PathPlanner {
public:
    std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal) override {
        std::cout << "Planning path from (" << start.first << "," << start.second 
                  << ") to (" << goal.first << "," << goal.second << ") using A* algorithm." << std::endl;
        // Dummy implementation
        std::vector<std::pair<int, int>> path = {start, {start.first, goal.second}, goal};
        std::cout << "Path found." << std::endl;
        return path;
    }
};
```

#### Decorator Base Class (`planner_decorator.hpp`)
```cpp
#pragma once
#include "path_planner.hpp"

// Decorator Base Class
class PlannerDecorator : public PathPlanner {
protected:
    std::unique_ptr<PathPlanner> planner_;

public:
    PlannerDecorator(std::unique_ptr<PathPlanner> planner) : planner_(std::move(planner)) {}

    std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal) override {
        return planner_->plan(start, goal);
    }
};
```

#### Concrete Decorators (`concrete_decorators.hpp`)
```cpp
#pragma once
#include "planner_decorator.hpp"
#include <chrono>

// Concrete Decorator for timing
class TimingDecorator : public PlannerDecorator {
public:
    using PlannerDecorator::PlannerDecorator; // Inherit constructor

    std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal) override {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto result = planner_->plan(start, goal);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        std::cout << "[TimingDecorator] Planning took " << duration.count() << " ms." << std::endl;
        return result;
    }
};

// Concrete Decorator for logging
class LoggingDecorator : public PlannerDecorator {
public:
    using PlannerDecorator::PlannerDecorator; // Inherit constructor

    std::vector<std::pair<int, int>> plan(const std::pair<int, int>& start, const std::pair<int, int>& goal) override {
        std::cout << "[LoggingDecorator] Calling planner for path from (" << start.first << "," << start.second 
                  << ") to (" << goal.first << "," << goal.second << ")." << std::endl;
        auto result = planner_->plan(start, goal);
        std::cout << "[LoggingDecorator] Planner finished execution. Path has " << result.size() << " points." << std::endl;
        return result;
    }
};
```

#### Client Code (`main.cpp`)
```cpp
#include "astar_planner.hpp"
#include "concrete_decorators.hpp"

void print_path(const std::vector<std::pair<int, int>>& path) {
    std::cout << "Resulting path: ";
    for (const auto& p : path) {
        std::cout << "(" << p.first << "," << p.second << ") ";
    }
    std::cout << std::endl;
}

int main() {
    // 1. Planner gốc
    auto simple_planner = std::make_unique<AStarPlanner>();
    std::cout << "--- Simple Planner ---" << std::endl;
    auto path1 = simple_planner->plan({0, 0}, {10, 10});
    print_path(path1);

    // 2. Gói planner với TimingDecorator
    auto timed_planner = std::make_unique<TimingDecorator>(std::make_unique<AStarPlanner>());
    std::cout << "\n--- Timed Planner ---" << std::endl;
    auto path2 = timed_planner->plan({0, 0}, {20, 20});
    print_path(path2);

    // 3. Gói planner với cả hai decorator
    // Thứ tự gói quan trọng: Logging -> Timing -> Planner
    auto logged_timed_planner = std::make_unique<LoggingDecorator>(
        std::make_unique<TimingDecorator>(
            std::make_unique<AStarPlanner>()
        )
    );
    std::cout << "\n--- Logged and Timed Planner ---" << std::endl;
    auto path3 = logged_timed_planner->plan({5, 5}, {25, 25});
    print_path(path3);

    return 0;
}
```

## 4. Best Practices

- **Single Responsibility:** Mỗi decorator chỉ nên thêm một chức năng duy nhất. Điều này giúp chúng dễ dàng được kết hợp và tái sử dụng.
- **Interface Consistency:** Decorator phải tuân thủ cùng một interface với đối tượng mà nó gói. Client không nên cần biết nó đang làm việc với một decorator hay một component gốc.
- **Order of Decoration:** Lưu ý rằng thứ tự áp dụng các decorator có thể quan trọng. Ví dụ, `Logging(Timing(Planner))` sẽ khác với `Timing(Logging(Planner))` về output log.
- **ROS2 Integration:**
  - Sử dụng Decorator để gói các ROS2 client, server, publisher, subscriber.
  - Ví dụ: một decorator cho một `rclpy.Publisher` có thể tự động đếm số lượng message được gửi hoặc kiểm tra tính hợp lệ của message trước khi publish.

## 5. Mở rộng

- **Decorator và Factory:** Kết hợp Decorator với Factory Pattern để tạo ra các đối tượng đã được trang trí sẵn dựa trên một cấu hình. Ví dụ, một factory có thể trả về một planner đã được gói với logging và timing nếu chế độ debug được bật.

## 6. Testing

- **Unit Test:** Test từng `ConcreteDecorator` một cách độc lập. Sử dụng mock object cho `Component` được gói để chỉ kiểm tra chức năng của decorator.
- **Integration Test:** Test các sự kết hợp khác nhau của các decorator để đảm bảo chúng hoạt động tốt với nhau và với concrete component.

## 7. Use Cases trong Robotics

- **Safety Layers:** Gói một `ActuatorInterface` với một `SafetyDecorator` để kiểm tra các lệnh điều khiển có nằm trong giới hạn an toàn không (vận tốc, gia tốc, lực tối đa) trước khi gửi đến phần cứng.
- **Data Transformation:** Một decorator quanh một `SensorInterface` có thể chuyển đổi dữ liệu (ví dụ: từ đơn vị raw sang đơn vị SI, từ hệ tọa độ camera sang hệ tọa độ robot) trước khi cung cấp cho các phần còn lại của hệ thống.
- **Caching:** Một decorator có thể thêm chức năng caching cho các hoạt động tốn kém. Ví dụ, một `IKServerDecorator` có thể cache kết quả của các truy vấn Inverse Kinematics cho các vị trí end-effector đã được yêu cầu gần đây.
- **Fault Tolerance:** Một decorator có thể thêm logic thử lại (retry) hoặc xử lý lỗi cho một service client. Nếu một lời gọi service thất bại, decorator có thể tự động thử lại một vài lần trước khi báo lỗi.
