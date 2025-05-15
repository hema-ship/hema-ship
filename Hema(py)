import time
import random

# Simulate LiDAR sensor
class LidarSensor:
    def get_data(self):
        return random.choice(["clear", "obstacle"])

# Simulate GPS sensor
class GPSSensor:
    def get_position(self):
        return (round(random.uniform(0, 10), 2), round(random.uniform(0, 10), 2))

# Simulate Camera sensor
class CameraSensor:
    def detect_objects(self):
        return random.choice(["none", "object"])

# Emergency stop mechanism
class EmergencyStop:
    def __init__(self):
        self.engaged = False

    def activate(self):
        self.engaged = True
        print("[SAFETY] Emergency Stop Activated!")

# Path planner
class PathPlanner:
    def generate_path(self, start, goal):
        print(f"[NAV] Planning path from {start} to {goal}...")
        path = []
        x, y = start
        goal_x, goal_y = goal
        while round(x, 2) != goal_x or round(y, 2) != goal_y:
            if round(x, 2) != goal_x:
                x += 1 if x < goal_x else -1
            if round(y, 2) != goal_y:
                y += 1 if y < goal_y else -1
            path.append((round(x, 2), round(y, 2)))
        return path

# Obstacle detection using LiDAR
class ObstacleAvoidance:
    def __init__(self, lidar_sensor):
        self.lidar = lidar_sensor

    def detect_obstacle(self):
        status = self.lidar.get_data()
        print(f"[SENSORS] LiDAR Status: {status}")
        return status == "obstacle"

# Main robot coordinator
class RobotCoordinator:
    def __init__(self):
        self.lidar = LidarSensor()
        self.gps = GPSSensor()
        self.camera = CameraSensor()
        self.safety = EmergencyStop()
        self.planner = PathPlanner()
        self.avoidance = ObstacleAvoidance(self.lidar)

    def handle_object(self):
        print("[ACTION] Handling detected object...")
        time.sleep(1)

    def execute_mission(self):
        print("[SYSTEM] Starting mission...\n")
        start = self.gps.get_position()
        goal = (start[0] + 5, start[1] + 5)
        path = self.planner.generate_path(start, goal)

        for position in path:
            if self.avoidance.detect_obstacle():
                print("[AVOIDANCE] Obstacle detected. Replanning path...")
                path = self.planner.generate_path(position, goal)
                continue

            object_status = self.camera.detect_objects()
            if object_status == "object":
                print("[CAMERA] Object detected.")
                self.handle_object()

            print(f"[MOVE] Navigating to {position}")
            time.sleep(0.5)

            # Randomly trigger emergency stop (5% chance)
            if random.random() < 0.05:
                self.safety.activate()
                break

        if not self.safety.engaged:
            print("\n[SYSTEM] Mission completed successfully.")
        else:
            print("\n[SYSTEM] Mission aborted due to safety protocol.")

# Entry point
if __name__ == "__main__":
    coordinator = RobotCoordinator()
    coordinator.execute_mission()
