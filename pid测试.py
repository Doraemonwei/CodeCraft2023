import math
import numpy as np


class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def move(self, v, w, dt):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output


def follow_trajectory(trajectory, robot, v_pid, w_pid, dt):
    """
    Follow a trajectory of (x, y) points using a robot and PID controllers for
    linear and angular velocity.
    """
    index = 0
    while index < len(trajectory):
        x_target, y_target = trajectory[index]
        distance_error = math.sqrt((x_target - robot.x) ** 2 + (y_target - robot.y) ** 2)
        if distance_error < 0.4:
            index += 1
            continue
        angle_to_target = math.atan2(y_target - robot.y, x_target - robot.x)
        angle_error = angle_to_target - robot.theta

        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        v = min(v_pid.control(distance_error, dt), 6.6)
        w = min(w_pid.control(angle_error, dt), 3.4)

        robot.move(v, w, dt)


if __name__ == '__main__':
    robot = Robot(0, 0, 0)
    v_pid = PIDController(1, 0.01, 0.05)
    w_pid = PIDController(1, 0.01, 0.05)
    trajectory = [(0, 1), (0, 2), (0, 3), (0, 4)]
    dt = 0.1
    follow_trajectory(trajectory, robot, v_pid, w_pid, dt)
