import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
from math import fabs

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

PRED_STEERING_FACTOR = 1.0


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,  # YawController
                 steer_ratio,  # YawController
                 max_lat_accel,  # YawController
                 max_steer_angle):  # YawController

        self.prev_time = rospy.get_time()
        self.velocity_pid = PID(kp=0.2, ki=0.00009, kd=1.7, mn=-max_steer_angle, mx=max_steer_angle)

        self.steer_pid = PID(kp=0.2, ki=0.00009, kd=1.7,
                             mn=-max_steer_angle, mx=max_steer_angle)
        self.max_steer_angle = max_steer_angle
        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=2.0,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

    def control(self,
                dbw_enabled,
                cte,
                linear_velocity,
                angular_velocity,
                current_velocity):

        throttle = 0.3
        brake = 0.0
        steer = 0.0

        if dbw_enabled:
            current_time = rospy.get_time()
            sample_time = current_time - self.prev_time
            self.prev_time = current_time

            predictive_steer = self.yaw_controller.get_steering(linear_velocity=linear_velocity,
                                                                angular_velocity=angular_velocity,
                                                                current_velocity=current_velocity)
            corrective_steer = self.steer_pid.step(cte, sample_time)

            steer = 0.3 * corrective_steer + 0.2 * predictive_steer

            rospy.logwarn('steer = %f, cte = %f, sample_time = %f, corrective_steer = %f, predictive_steer = %f',
                          steer, cte, sample_time, corrective_steer, predictive_steer)

            steer = corrective_steering + PRED_STEERING_FACTOR * predictive_steering

            rospy.logwarn('steer = %f, cte = %f, sample_time = %f', steer, cte, sample_time)
        else:
            self.steer_pid.reset()
            self.prev_time = rospy.get_time()

<<<<<<< Updated upstream
=======
        # throttle = 1.0 - 0.9 * self.max_steer_angle / 100 * fabs(steer)

>>>>>>> Stashed changes
        return throttle, brake, steer
