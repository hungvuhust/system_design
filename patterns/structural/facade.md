# Facade Pattern trong ROS2 và Robotics

## 1. Giới thiệu

Facade là một mẫu thiết kế cấu trúc (Structural Design Pattern) cung cấp một interface đơn giản hóa cho một thư viện, một framework, hoặc bất kỳ một tập hợp các class phức tạp nào khác.

Trong ROS2 và robotics, hệ thống thường bao gồm nhiều subsystem phức tạp (ví dụ: navigation, manipulation, perception). Mỗi subsystem lại có thể bao gồm nhiều ROS2 nodes, topics, services, và actions. Facade Pattern giúp che giấu sự phức tạp này đằng sau một API đơn giản, dễ sử dụng.

**Ví dụ:**
- Cung cấp một phương thức `robot.navigate_to(pose)` thay vì yêu cầu người dùng phải tương tác trực tiếp với planner, controller, và map server.
- Cung cấp một hàm `robot.pick_object(object_id)` thay vì phải quản lý chuỗi các hành động của MoveIt2.

## 2. Cấu trúc

- **Facade:**
  - Là class cung cấp interface đơn giản cho client.
  - Biết về các subsystem mà nó cần để thực hiện yêu cầu của client.
  - Ủy thác (delegate) các lời gọi từ client đến các đối tượng phù hợp trong subsystem.

- **Subsystem Classes:**
  - Là các class implement chức năng phức tạp của hệ thống.
  - Chúng không biết về sự tồn tại của Facade. Chúng hoạt động độc lập và có thể được sử dụng trực tiếp bởi client nếu cần.

- **Client:**
  - Sử dụng Facade để tương tác với subsystem một cách đơn giản.
  - Client không cần biết về sự phức tạp bên trong của subsystem.

## 3. Ví dụ trong ROS2

Hãy tạo một Facade cho một hệ thống navigation đơn giản. Hệ thống này bao gồm 3 subsystem:
1.  `LocalizationSystem`: Cung cấp vị trí hiện tại của robot.
2.  `PlanningSystem`: Tìm một đường đi từ điểm A đến điểm B.
3.  `ControlSystem`: Điều khiển robot di chuyển theo đường đi đã hoạch định.

### 3.1. Subsystem Classes

Đây là các class giả lập cho các subsystem phức tạp.

```python
# subsystem.py
from typing import List, Tuple

class LocalizationSystem:
    """Một phần của subsystem phức tạp"""
    def get_current_pose(self) -> Tuple[int, int]:
        print("[Localization] Getting current robot pose.")
        return (0, 0)

class PlanningSystem:
    """Một phần của subsystem phức tạp"""
    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        print(f"[Planning] Planning path from {start} to {goal}.")
        return [start, (start[0], goal[1]), goal]

class ControlSystem:
    """Một phần của subsystem phức tạp"""
    def execute_path(self, path: List[Tuple[int, int]]):
        print(f"[Control] Executing path: {path}")
        for point in path:
            print(f"[Control] Moving to {point}...")
        print("[Control] Goal reached.")
```

### 3.2. Navigation Facade

Facade này cung cấp một phương thức `navigate_to_goal` duy nhất.

```python
# navigation_facade.py
from subsystem import LocalizationSystem, PlanningSystem, ControlSystem
from typing import Tuple

class NavigationFacade:
    """Facade"""
    def __init__(self):
        self._localization = LocalizationSystem()
        self._planning = PlanningSystem()
        self._control = ControlSystem()

    def navigate_to_goal(self, goal: Tuple[int, int]):
        """Interface đơn giản hóa"""
        print(f"--- Starting navigation to {goal} ---")
        # 1. Lấy vị trí hiện tại
        current_pose = self._localization.get_current_pose()

        # 2. Hoạch định đường đi
        path = self._planning.plan_path(current_pose, goal)

        # 3. Thực thi đường đi
        self._control.execute_path(path)
        print("--- Navigation finished ---")
```

### 3.3. Client Code

Client chỉ cần tương tác với Facade.

```python
# main.py
from navigation_facade import NavigationFacade

if __name__ == "__main__":
    # Client chỉ cần biết về NavigationFacade
    navigation = NavigationFacade()

    # Thực hiện một tác vụ phức tạp bằng một lời gọi hàm duy nhất
    navigation.navigate_to_goal((10, 20))
```

### Kết quả chạy:
```
--- Starting navigation to (10, 20) ---
[Localization] Getting current robot pose.
[Planning] Planning path from (0, 0) to (10, 20).
[Control] Executing path: [(0, 0), (0, 20), (10, 20)]
[Control] Moving to (0, 0)...
[Control] Moving to (0, 20)...
[Control] Moving to (10, 20)...
[Control] Goal reached.
--- Navigation finished ---
```

## 4. Best Practices

- **Don't Block Access:** Facade nên đơn giản hóa việc truy cập, nhưng không nên là cách duy nhất. Cho phép các client "nâng cao" có thể truy cập trực tiếp vào các subsystem nếu họ cần chức năng phức tạp hơn.
- **Single Responsibility:** Một Facade nên đại diện cho một subsystem logic duy nhất. Tránh tạo ra các "God Object" Facade quản lý mọi thứ trong hệ thống.
- **ROS2 Integration:**
  - Facade có thể là một ROS2 Node, quản lý các client cho các service và action của subsystem.
  - Cung cấp một action server (ví dụ: `NavigateToPose.action`) làm Facade cho một chuỗi các hoạt động phức tạp.

## 5. Mở rộng

- **Multiple Facades:** Có thể có nhiều Facade cho cùng một subsystem, mỗi Facade cung cấp một tập hợp các chức năng đơn giản hóa cho các loại client khác nhau.

## 6. Testing

- **Integration Test:** Test Facade bằng cách kiểm tra xem nó có gọi đúng các subsystem theo đúng thứ tự và với đúng tham số hay không. Bạn có thể sử dụng mock object cho các subsystem để cô lập Facade trong quá trình test.

## 7. Use Cases trong Robotics

- **Navigation Stack (Nav2):** Cung cấp một action server `NavigateToPose` làm facade cho toàn bộ quá trình navigation, bao gồm global planning, local planning, và recovery behaviors.
- **Manipulation Stack (MoveIt2):** `MoveGroupInterface` là một facade cho việc hoạch định và thực thi chuyển động của cánh tay robot. Nó che giấu sự phức tạp của việc tương tác với planning scene, inverse kinematics solvers, và trajectory controllers.
- **Robot Startup/Shutdown:** Một `SystemManager` node có thể hoạt động như một facade để khởi tạo, cấu hình và tắt tất cả các driver và các node phần mềm khác của robot một cách có trật tự.
- **Perception Pipeline:** Một facade có thể đơn giản hóa việc lấy thông tin đối tượng từ một pipeline perception phức tạp. Ví dụ, một phương thức `detect_objects()` có thể ẩn đi các bước như lấy ảnh, tiền xử lý, chạy model object detection, và hậu xử lý kết quả.
