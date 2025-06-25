# Proxy Pattern in ROS2

## What is the Proxy Pattern?

The Proxy pattern provides a surrogate or placeholder for another object to control access to it. It is used to add a level of indirection to the object, allowing you to perform operations before or after the request gets to the original object. This can be useful for lazy initialization, access control, logging, etc.

## Use Case in Robotics

In a robotics system, you might have a resource that is expensive to load, such as a large map of the environment. You can use a proxy to delay the loading of the map until it is actually needed. The proxy object will have the same interface as the map object, but it will only load the map from disk the first time a map-related operation is called.

Another use case is a remote proxy for a ROS2 service or action client, where the proxy handles the communication details, making it seem like a local object call.

## C++ Example

Here is a C++ example of the Proxy pattern for lazy loading of a navigation map.

```cpp
// Subject interface
class Map {
public:
    virtual ~Map() {}
    virtual void display() const = 0;
    virtual bool isObstacle(double x, double y) const = 0;
};

// Real Subject
class NavigationMap : public Map {
private:
    std::string mapData;
    void loadMap(const std::string& mapFile) {
        // Expensive operation: load map data from a file
    }

public:
    NavigationMap(const std::string& mapFile) {
        loadMap(mapFile);
    }

    void display() const override {
        // Display the map
    }

    bool isObstacle(double x, double y) const override {
        // Check for obstacles
        return false;
    }
};

// Proxy
class MapProxy : public Map {
private:
    NavigationMap* realMap;
    std::string mapFile;

public:
    MapProxy(const std::string& mapFile) : realMap(nullptr), mapFile(mapFile) {}

    ~MapProxy() {
        delete realMap;
    }

    void display() const override {
        if (!realMap) {
            // const_cast is used here to modify the realMap pointer in a const method
            // This is a common pattern in proxy implementations for lazy loading
            const_cast<MapProxy*>(this)->realMap = new NavigationMap(mapFile);
        }
        realMap->display();
    }

    bool isObstacle(double x, double y) const override {
        if (!realMap) {
            const_cast<MapProxy*>(this)->realMap = new NavigationMap(mapFile);
        }
        return realMap->isObstacle(x, y);
    }
};
```

## Best Practices

*   **Interface Identity:** The proxy and the real subject should implement the same interface.
*   **Lifecycle Management:** The proxy should be responsible for creating and deleting the real subject.
*   **Transparency:** The client should not need to know whether it is interacting with the proxy or the real subject.

## Extensions and Variations

*   **Virtual Proxy:** Delays the creation of an object until it is needed (as in the example).
*   **Protection Proxy:** Controls access to an object based on permissions.
*   **Remote Proxy:** Provides a local representative for an object in a different address space (common in ROS2).

## Testing

*   **Unit Testing:** Test the proxy's logic (e.g., lazy loading, access control) without the real subject.
*   **Integration Testing:** Test the interaction between the proxy and the real subject.
