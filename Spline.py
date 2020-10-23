from __future__ import annotations

import math

import Math


class Spline:
    # the number of samples for integration
    num_samples = 100000
    # ax^5 + bx^4 +cx^3 + dx^2 + ex
    a = 0.0
    b = 0.0
    c = 0.0
    d = 0.0
    e = 0.0

    y_offset = 0.0

    x_offset = 0.0

    knot_distance = 0.0
    theta_offset = 0.0
    arc_length = 0.0

    def __init__(self):
        self.arc_length = -1

    @staticmethod
    def almost_equal(x, y):
        return abs(x - y) < 1e-6

    @staticmethod
    def reticulate_spline(x0: float, y0: float, theta0: float,
                          x1: float, y1: float, theta1: float):
        print("reticulating splines...")
        result = Spline()
        result.x_offset = x0
        result.y_offset = y0

        x1_hat = math.sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2))

        if x1_hat == 0:
            return None

        result.knot_distance = x1_hat
        result.theta_offset = math.atan2(y1 - y0, x1 - x0)

        theta0_hat = Math.get_difference_in_angle_radians(result.theta_offset, theta0)
        theta1_hat = Math.get_difference_in_angle_radians(result.theta_offset, theta1)

        # Currently doesnt support vertical slope, (meaning a 90 degrees off the straight line between p0 and p1
        if Spline.almost_equal(abs(theta0_hat), math.pi / 2) or Spline.almost_equal(abs(theta1_hat), math.pi / 2):
            return None

        # Currently doesnt support turning more than 90 degrees in a single path
        if abs(Math.get_difference_in_angle_radians(theta0_hat, theta1_hat)) >= math.pi / 2:
            return None

        yp0_hat = math.tan(theta0_hat)
        yp1_hat = math.tan(theta1_hat)

        result.a = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat)
        result.b = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat)
        result.c = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat)
        result.d = 0
        result.e = yp0_hat

        return result

    def derivative_at(self, percentage: float):
        percentage = max(min(percentage, 1), 0)

        x_hat = percentage * self.knot_distance
        yp_hat = (5 * self.a * x_hat + 4 * self.b) * x_hat * x_hat * x_hat \
                 + 3 * self.c * x_hat * x_hat + 2 * self.d * x_hat + self.e

        return yp_hat

    def second_derivative_at(self, percentage: float):
        percentage = max(min(percentage, 1), 0)

        x_hat = percentage * self.knot_distance
        ypp_hat = (20 * self.a * x_hat + 12 * self.b) * x_hat * x_hat + 6 * self.c * x_hat + 2 * self.d
        return ypp_hat

    def angle_at(self, percentage: float):
        percentage = max(min(percentage, 1), 0)
        angle = Math.bound_angle_0_to_2pi_radians(math.atan(self.derivative_at(percentage)) + self.theta_offset)
        return angle

    def calculate_length(self):
        if self.arc_length >= 0:
            return self.arc_length

        num_samples = self.num_samples
        arc_length = 0.0

        integrand = math.sqrt(1 + self.derivative_at(0) * self.derivative_at(0)) / num_samples
        last_integrand = integrand

        for i in range(1, num_samples + 1):
            t = float(i) / num_samples
            dydt = self.derivative_at(t)
            integrand = math.sqrt(1 + dydt * dydt) / num_samples
            arc_length += (integrand + last_integrand) / 2.0
            last_integrand = integrand
        self.arc_length = self.knot_distance * arc_length

        return self.arc_length

    def get_percentage_for_distance(self, distance: float):
        num_samples = self.num_samples
        arc_length = 0.0
        t = 0.0
        last_arc_length = 0.0
        dydt = 0.0

        integrand = math.sqrt(1 + self.derivative_at(0) * self.derivative_at(0)) / num_samples
        last_integrand = integrand
        distance /= self.knot_distance
        for i in range(1, num_samples + 1):
            t = float(i) / num_samples
            dydt = self.derivative_at(t)
            integrand = math.sqrt(1 + dydt * dydt) / num_samples
            arc_length += (integrand + last_integrand) / 2.0
            if arc_length > distance:
                break

            last_integrand = integrand
            last_arc_length = arc_length

        interpolated = t
        if arc_length != last_arc_length:
            interpolated += ((distance - last_arc_length) / (arc_length - last_arc_length) - 1) / float(num_samples)
        return interpolated

    def get_xy(self, percentage: float) -> []:
        result = [0.0] * 2
        percentage = max(min(percentage, 1), 0)

        x_hat = percentage * self.knot_distance
        y_hat = (self.a * x_hat + self.b) * x_hat * x_hat * x_hat * x_hat \
                + self.c * x_hat * x_hat * x_hat + self.d * x_hat * x_hat + self.e * x_hat

        cos_theta = math.cos(self.theta_offset)
        sin_theta = math.sin(self.theta_offset)

        result[0] = x_hat * cos_theta - y_hat * sin_theta + self.x_offset
        result[1] = x_hat * sin_theta + y_hat * cos_theta + self.y_offset

        return result
