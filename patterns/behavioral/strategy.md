# STRATEGY PATTERN TRONG ROS2

## 1. Strategy Pattern là gì?

Strategy Pattern là một mẫu thiết kế hành vi cho phép định nghĩa một nhóm các thuật toán, đóng gói từng thuật toán và làm cho chúng có thể hoán đổi cho nhau. Thay vì cài đặt thuật toán cố định, bạn có thể chọn thuật toán phù hợp tại runtime.

## 2. Ứng dụng trong Robotics

Strategy Pattern cực kỳ phổ biến trong robotics. Nó cho phép robot thay đổi hành vi dựa trên ngữ cảnh hoặc môi trường hiện tại. Ví dụ:

*   **Lập kế hoạch đường đi:** Robot có thể chuyển đổi giữa các thuật toán lập kế hoạch đường đi khác nhau (A*, DWA, RRT) tùy vào môi trường (trống, chướng ngại vật, động/tĩnh).
*   **Gắp vật:** Cánh tay robot có thể dùng các chiến lược gắp khác nhau dựa trên hình dạng, kích thước, chất liệu vật thể.
*   **Sensor Fusion:** Hệ thống có thể chuyển đổi giữa các thuật toán hợp nhất cảm biến (Kalman Filter, Particle Filter) dựa trên độ nhiễu hoặc tình trạng cảm biến.

Trong ROS2, pluginlib là công cụ mạnh để triển khai Strategy Pattern, cho phép nạp các thuật toán như plugin tại runtime.

## 3. Ví dụ C++

```cpp
// Interface Strategy
class PathPlannerStrategy {
public:
    virtual ~PathPlannerStrategy() {}
    virtual void planPath(double startX, double startY, double endX, double endY) = 0;
};

// Strategy cụ thể A
class AStarPlanner : public PathPlannerStrategy {
public:
    void planPath(double startX, double startY, double endX, double endY) override {
        // Thuật toán A*
    }
};

// Strategy cụ thể B
class RRTPlanner : public PathPlannerStrategy {
public:
    void planPath(double startX, double startY, double endX, double endY) override {
        // Thuật toán RRT
    }
};

// Context
class NavigationContext {
private:
    PathPlannerStrategy* planner;
public:
    void setPlanner(PathPlannerStrategy* newPlanner) {
        planner = newPlanner;
    }
    void navigate() {
        if (planner) {
            planner->planPath(0, 0, 100, 100);
        }
    }
};
```

## 4. Best Practices

*   **Dựa trên interface:** Context nên phụ thuộc vào interface strategy, không phụ thuộc vào các class cụ thể.
*   **Stateless:** Nên thiết kế strategy không có trạng thái, mọi dữ liệu cần thiết truyền qua tham số.
*   **Sử dụng pluginlib:** Trong ROS2, nên dùng pluginlib để nạp strategy động.

## 5. Mở rộng và Biến thể

*   **Kết hợp nhiều strategy:** Có thể tạo strategy là sự kết hợp của nhiều strategy con.
*   **Strategy Factory:** Có thể dùng factory để tạo và chọn strategy phù hợp dựa trên cấu hình hoặc điều kiện runtime.

## 6. Testing

*   **Unit Test:** Test từng strategy cụ thể độc lập.
*   **Test Context:** Test context với mock strategy để đảm bảo gọi đúng.
*   **Integration Test:** Test context với tất cả strategy để đảm bảo hoạt động đúng.

## 7. Kết luận

Strategy Pattern là một mẫu thiết kế quan trọng trong ROS2, đặc biệt hữu ích cho việc quản lý các thuật toán và chiến lược khác nhau trong robotics. Pattern này giúp hệ thống linh hoạt, dễ mở rộng, dễ bảo trì và tối ưu cho các ứng dụng cần thay đổi thuật toán runtime.