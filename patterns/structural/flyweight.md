# Flyweight Pattern in ROS2

## What is the Flyweight Pattern?

The Flyweight pattern is a structural design pattern that minimizes memory usage by sharing as much data as possible with other similar objects. It is useful when you need to create a large number of objects that have some shared state. The pattern separates the intrinsic (shared) state from the extrinsic (unique) state.

## Use Case in Robotics

In a robotics application, you might have a large number of identical sensors or actuators. For example, a robot might have a grid of proximity sensors. Each sensor has some common properties (model, range, field of view) and some unique properties (ID, position on the robot). The Flyweight pattern can be used to share the common properties among all sensor objects, reducing memory consumption.

## C++ Example

Here is a C++ example of the Flyweight pattern for representing robot sensors.

```cpp
// Flyweight interface
class SensorFlyweight {
public:
    virtual ~SensorFlyweight() {}
    virtual void display(int sensorID, double position) const = 0;
};

// Concrete Flyweight
class ProximitySensor : public SensorFlyweight {
private:
    std::string model;
    double range;
    double fov;

public:
    ProximitySensor(const std::string& model, double range, double fov)
        : model(model), range(range), fov(fov) {}

    void display(int sensorID, double position) const override {
        // Display sensor information
    }
};

// Flyweight Factory
class SensorFlyweightFactory {
private:
    std::map<std::string, SensorFlyweight*> flyweights;

public:
    ~SensorFlyweightFactory() {
        for (auto pair : flyweights) {
            delete pair.second;
        }
        flyweights.clear();
    }

    SensorFlyweight* getFlyweight(const std::string& key, const std::string& model, double range, double fov) {
        if (flyweights.find(key) == flyweights.end()) {
            flyweights[key] = new ProximitySensor(model, range, fov);
        }
        return flyweights[key];
    }

    void listFlyweights() const {
        // List all flyweights
    }
};

// Context
class Sensor {
private:
    int id;
    double position;
    const SensorFlyweight* flyweight;

public:
    Sensor(int id, double position, const SensorFlyweight* flyweight)
        : id(id), position(position), flyweight(flyweight) {}

    void display() const {
        flyweight->display(id, position);
    }
};
```

## Best Practices

*   **Immutability:** The intrinsic state in the flyweight object should be immutable.
*   **Factory:** Use a factory to manage the creation and sharing of flyweight objects.
*   **Garbage Collection:** Be mindful of the lifecycle of flyweight objects, especially in C++ where you need to manage memory manually.

## Extensions and Variations

*   **Shared vs. Unshared Flyweights:** You can have a mix of shared and unshared flyweight objects.
*   **Stateful Flyweights:** While the intrinsic state is typically immutable, you can have flyweights with state that can be modified, but this should be done with care.

## Testing

*   **Unit Testing:** Test the flyweight factory to ensure that it correctly creates and shares flyweights.
*   **Integration Testing:** Test the interaction between the context objects and the flyweight objects.
