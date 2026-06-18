import math
import os

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

from .pid import PID, clamp, wrap_to_pi
from futuraps_perception.srv import GetClosestGrid


class PlantFollowerPID(Node):
    def __init__(self):
        super().__init__("pid_controller")

        # ---- General params ----
        self.declare_parameter("reference_distance", 0.6)
        self.declare_parameter(
            "bearing_ref", -math.pi / 2
        )  # -pi/2 for plant on right, +pi/2 for left
        self.declare_parameter("v_nominal", 0.25)
        self.declare_parameter("v_max", 0.35)
        self.declare_parameter("w_max", 0.8)
        self.declare_parameter("straight_only", False)
        self.declare_parameter("min_distance_stop", 0.3)
        self.declare_parameter("slow_down_on_heading_error", False)
        self.declare_parameter("heading_slowdown_gain", 0.8)

        self.declare_parameter("control_rate_hz", 20.0)

        # Service timing:
        # - request_period_s controls how often we try to send a new request
        # - result_max_age_s controls how old a cached result can be before we stop
        self.declare_parameter(
            "request_period_s", 0.10
        )  # 10 Hz requests (lighter than 20 Hz control)
        self.declare_parameter("result_max_age_s", 0.50)  # seconds

        self.declare_parameter("perception_service", "/get_closest_grid")
        self.declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter(
            "cmd_vel_unstamped_topic", "/diff_drive_controller/cmd_vel_unstamped"
        )
        self.declare_parameter("publish_unstamped_cmd_vel", True)
        self.declare_parameter("odom_topic", "/diff_drive_controller/odom")
        self.declare_parameter(
            "cmd_vel_frame_id", "odom"
        )  # keep to match your CLI test
        self.declare_parameter("debug_csv_path", "/tmp/platform_pid_debug.csv")

        # ---- PID params ----
        hp = self.declare_parameter("heading_pid.kp", 1.2).value
        hi = self.declare_parameter("heading_pid.ki", 0.0).value
        hd = self.declare_parameter("heading_pid.kd", 0.05).value
        h_imax = self.declare_parameter("heading_pid.i_max", 0.6).value

        dp = self.declare_parameter("distance_pid.kp", 0.8).value
        di = self.declare_parameter("distance_pid.ki", 0.0).value
        dd = self.declare_parameter("distance_pid.kd", 0.0).value
        d_imax = self.declare_parameter("distance_pid.i_max", 0.4).value
        self.declare_parameter("distance_pid.output_sign", 1.0)

        self.pid_heading = PID(hp, hi, hd, h_imax)
        self.pid_dist = PID(dp, di, dd, d_imax)

        # ---- Grid request params ----
        # XZ grid in base_link, and lateral acceptance via y_left/right_max.
        self.declare_parameter("grid.cell_size", 0.3)
        self.declare_parameter("grid.rows", 5)
        self.declare_parameter("grid.cols", 5)
        self.declare_parameter("grid.x0", -0.15)
        self.declare_parameter("grid.z0", 0.2)
        self.declare_parameter("grid.y_left_max", 2.0)
        self.declare_parameter("grid.y_right_max", 2.0)
        self.declare_parameter("grid.side", 0)  # 0 both, 1 left-only, -1 right-only
        self.declare_parameter("grid.front_percentile", 0.01)
        self.declare_parameter("grid.min_points_per_cell", 10)
        self.declare_parameter("grid.conf_min", 0.0)

        # ---- State ----
        self.enabled = False

        # Async service bookkeeping
        self._grid_future = None
        self._last_grid_res = None
        self._last_grid_stamp = None
        self._last_request_time = None
        self._last_odom = None
        self._debug_sample_count = 0
        self._grid_request_count = 0
        self._grid_response_count = 0

        # Throttle warning logs
        self._last_warn_time = 0.0
        self._last_grid_diag_time = 0.0

        # ---- Publisher ----
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.publish_unstamped_cmd_vel = bool(
            self.get_parameter("publish_unstamped_cmd_vel").value
        )
        if self.publish_unstamped_cmd_vel:
            cmd_vel_unstamped_topic = self.get_parameter(
                "cmd_vel_unstamped_topic"
            ).value
            self.cmd_unstamped_pub = self.create_publisher(
                Twist, cmd_vel_unstamped_topic, 10
            )
            self.get_logger().info(
                "Publishing platform commands as TwistStamped on "
                f"{cmd_vel_topic} and Twist on {cmd_vel_unstamped_topic}"
            )
        else:
            self.cmd_unstamped_pub = None
            self.get_logger().info(
                f"Publishing platform commands as TwistStamped on {cmd_vel_topic}"
            )

        # ---- Odometry feedback for plotting/debug ----
        odom_topic = self.get_parameter("odom_topic").value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 20)

        # ---- Enable service ----
        self.enable_srv = self.create_service(SetBool, "~/enable", self.on_enable)

        # ---- Perception client ----
        srv_name = self.get_parameter("perception_service").value
        self.cp_client = self.create_client(GetClosestGrid, srv_name)
        self.get_logger().info(f"Using GetClosestGrid service: {srv_name}")

        # ---- Control loop timer ----
        rate = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_tick)

        self._debug_file = self._open_debug_csv()

        self._log_grid_config()
        self.get_logger().info("PlantFollowerPID ready. Call ~/enable to start/stop.")

    # -------------------- Control enable/disable --------------------

    def on_enable(self, req, resp):
        self.enabled = bool(req.data)

        self.pid_heading.reset()
        self.pid_dist.reset()

        # Clear cached perception so we don't use stale results after re-enable
        self._grid_future = None
        self._last_grid_res = None
        self._last_grid_stamp = None
        self._last_request_time = None
        self._grid_request_count = 0
        self._grid_response_count = 0

        if not self.enabled:
            self.publish_cmd(0.0, 0.0)
        else:
            self._log_grid_config()

        self.get_logger().info(
            "PID controller %s via ~/enable service"
            % ("enabled" if self.enabled else "disabled")
        )

        resp.success = True
        resp.message = "enabled" if self.enabled else "disabled"
        return resp

    # -------------------- ROS publish helpers --------------------

    def on_odom(self, msg):
        self._last_odom = msg

    def publish_cmd(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("cmd_vel_frame_id").value
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

        if self.cmd_unstamped_pub is not None:
            unstamped_msg = Twist()
            unstamped_msg.linear.x = float(v)
            unstamped_msg.angular.z = float(w)
            self.cmd_unstamped_pub.publish(unstamped_msg)

    def _open_debug_csv(self):
        path = self.get_parameter("debug_csv_path").value
        if not path:
            return None

        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            f = open(path, "w", encoding="utf-8")
            f.write(
                "t,enabled,x,y,yaw,v_odom,w_odom,v_cmd,w_cmd,"
                "closest_x,closest_y,distance,bearing,heading_error,distance_error\n"
            )
            f.flush()
            self.get_logger().info(f"Writing platform PID debug CSV to {path}")
            return f
        except OSError as exc:
            self.get_logger().warn(
                f"Could not open platform PID debug CSV '{path}': {exc}"
            )
            return None

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_values(self):
        if self._last_odom is None:
            return (math.nan, math.nan, math.nan, math.nan, math.nan)

        pose = self._last_odom.pose.pose
        twist = self._last_odom.twist.twist
        return (
            pose.position.x,
            pose.position.y,
            self._yaw_from_quaternion(pose.orientation),
            twist.linear.x,
            twist.angular.z,
        )

    def log_debug_sample(
        self,
        v_cmd,
        w_cmd,
        closest_x=math.nan,
        closest_y=math.nan,
        distance=math.nan,
        bearing=math.nan,
        heading_error=math.nan,
        distance_error=math.nan,
    ):
        if self._debug_file is None:
            return

        x, y, yaw, v_odom, w_odom = self._odom_values()
        t = self.get_clock().now().nanoseconds * 1e-9
        self._debug_file.write(
            f"{t:.9f},{int(self.enabled)},"
            f"{x:.9f},{y:.9f},{yaw:.9f},{v_odom:.9f},{w_odom:.9f},"
            f"{v_cmd:.9f},{w_cmd:.9f},"
            f"{closest_x:.9f},{closest_y:.9f},{distance:.9f},{bearing:.9f},"
            f"{heading_error:.9f},{distance_error:.9f}\n"
        )

        self._debug_sample_count += 1
        if self._debug_sample_count % 20 == 0:
            self._debug_file.flush()

    def _warn_throttled(self, text: str, period_s: float = 1.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_warn_time >= period_s:
            self.get_logger().warn(text)
            self._last_warn_time = now

    def _grid_diag_throttled(self, text: str, period_s: float = 1.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_grid_diag_time >= period_s:
            self.get_logger().info(text)
            self._last_grid_diag_time = now

    def _log_grid_config(self):
        cell_size = float(self.get_parameter("grid.cell_size").value)
        rows = int(self.get_parameter("grid.rows").value)
        cols = int(self.get_parameter("grid.cols").value)
        x0 = float(self.get_parameter("grid.x0").value)
        z0 = float(self.get_parameter("grid.z0").value)
        y_right_max = float(self.get_parameter("grid.y_right_max").value)
        y_left_max = float(self.get_parameter("grid.y_left_max").value)
        self.get_logger().info(
            "PID closest-grid config: cell=%.3f rows=%d cols=%d "
            "x=[%.3f, %.3f] z=[%.3f, %.3f] y=[-%.3f, %.3f] "
            "side=%d front_percentile=%.3f min_points=%d conf_min=%.3f"
            % (
                cell_size,
                rows,
                cols,
                x0,
                x0 + cols * cell_size,
                z0,
                z0 + rows * cell_size,
                y_right_max,
                y_left_max,
                int(self.get_parameter("grid.side").value),
                float(self.get_parameter("grid.front_percentile").value),
                int(self.get_parameter("grid.min_points_per_cell").value),
                float(self.get_parameter("grid.conf_min").value),
            )
        )

    # -------------------- Main timer --------------------

    def control_tick(self):
        if not self.enabled:
            return

        # 1) Send / poll async perception
        self._maybe_send_grid_request()
        self._poll_grid_future()

        # 2) Extract best (x,y) from last response
        closest = self._extract_closest_from_last_result(
            max_age_s=float(self.get_parameter("result_max_age_s").value)
        )
        if closest is None:
            self.publish_cmd(0.0, 0.0)
            self.log_debug_sample(0.0, 0.0)
            return

        x, y = closest  # in base_link meters

        # Using lateral distance for wall-following
        distance = abs(y)
        bearing = math.atan2(y, x)

        if distance < float(self.get_parameter("min_distance_stop").value):
            self._warn_throttled("Too close to plant -> stopping", period_s=0.5)
            self.publish_cmd(0.0, 0.0)
            self.log_debug_sample(0.0, 0.0, x, y, distance, bearing)
            return

        bearing_ref = float(self.get_parameter("bearing_ref").value)
        d_ref = float(self.get_parameter("reference_distance").value)

        heading_error = wrap_to_pi(bearing - bearing_ref)
        distance_error = distance - d_ref

        # 3) Controller
        if bool(self.get_parameter("straight_only").value):
            w = 0.0
        else:
            distance_output_sign = float(
                self.get_parameter("distance_pid.output_sign").value
            )
            w = self.pid_heading.step(heading_error, self.dt) + (
                distance_output_sign * self.pid_dist.step(distance_error, self.dt)
            )
            w_max = float(self.get_parameter("w_max").value)
            w = clamp(w, -w_max, w_max)

        v_nom = float(self.get_parameter("v_nominal").value)
        v_max = float(self.get_parameter("v_max").value)

        v = v_nom
        if bool(self.get_parameter("slow_down_on_heading_error").value):
            slowdown_gain = float(self.get_parameter("heading_slowdown_gain").value)
            slowdown = max(0.0, 1.0 - slowdown_gain * min(1.0, abs(heading_error)))
            v *= slowdown
        v = clamp(v, 0.0, v_max)

        self.publish_cmd(v, w)
        self.log_debug_sample(
            v,
            w,
            x,
            y,
            distance,
            bearing,
            heading_error,
            distance_error,
        )

    # -------------------- Perception async plumbing --------------------

    def _maybe_send_grid_request(self):
        # Only allow one in-flight request
        if self._grid_future is not None:
            return

        # Rate-limit requests
        req_period = float(self.get_parameter("request_period_s").value)
        now = self.get_clock().now()
        if self._last_request_time is not None:
            age = (now - self._last_request_time).nanoseconds * 1e-9
            if age < req_period:
                return

        if not self.cp_client.service_is_ready():
            self._warn_throttled("GetClosestGrid service not ready")
            return

        req = GetClosestGrid.Request()
        req.cell_size = float(self.get_parameter("grid.cell_size").value)
        req.rows = int(self.get_parameter("grid.rows").value)
        req.cols = int(self.get_parameter("grid.cols").value)
        req.x0 = float(self.get_parameter("grid.x0").value)
        req.z0 = float(self.get_parameter("grid.z0").value)
        req.y_left_max = float(self.get_parameter("grid.y_left_max").value)
        req.y_right_max = float(self.get_parameter("grid.y_right_max").value)
        req.side = int(self.get_parameter("grid.side").value)
        req.front_percentile = float(self.get_parameter("grid.front_percentile").value)
        req.min_points_per_cell = int(
            self.get_parameter("grid.min_points_per_cell").value
        )

        self._grid_future = self.cp_client.call_async(req)
        self._grid_future.add_done_callback(self._on_grid_response)
        self._last_request_time = now
        self._grid_request_count += 1
        self._grid_diag_throttled(
            f"Sent GetClosestGrid request #{self._grid_request_count}"
        )

    def _on_grid_response(self, future):
        if future is not self._grid_future:
            return

        self._grid_future = None

        try:
            res = future.result()
        except Exception as exc:
            self._warn_throttled(f"GetClosestGrid future failed: {exc}", period_s=1.0)
            return

        if res is None:
            self._warn_throttled("GetClosestGrid returned None")
            return

        self._last_grid_res = res
        self._last_grid_stamp = self.get_clock().now()
        self._grid_response_count += 1

        # Debug once per second: how many found cells?
        found_count = sum(1 for f in res.found if f)
        valid_dist = [
            float(d)
            for d, found in zip(res.dist, res.found)
            if found and math.isfinite(float(d))
        ]
        min_dist = min(valid_dist) if valid_dist else float("nan")
        self._grid_diag_throttled(
            f"GetClosestGrid response #{self._grid_response_count}: "
            f"found={found_count}/{len(res.found)} min_found_dist={min_dist:.3f}",
            period_s=1.0,
        )

    def _poll_grid_future(self):
        # Responses are handled by _on_grid_response(). This method is retained
        # so the control loop's sequence stays explicit.
        return

    def _extract_closest_from_last_result(self, max_age_s: float):
        if self._last_grid_res is None or self._last_grid_stamp is None:
            self._warn_throttled("No GetClosestGrid data yet", period_s=1.0)
            return None

        age = (self.get_clock().now() - self._last_grid_stamp).nanoseconds * 1e-9
        if age > max_age_s:
            self._warn_throttled(
                f"GetClosestGrid data too old ({age:.2f}s) -> stopping", period_s=1.0
            )
            return None

        res = self._last_grid_res
        conf_min = float(self.get_parameter("grid.conf_min").value)

        best_idx = None
        best_dist = None

        n = len(res.found)
        for k in range(n):
            if not res.found[k]:
                continue
            if k < len(res.confidence) and res.confidence[k] < conf_min:
                continue

            d = res.dist[k]
            if best_dist is None or d < best_dist:
                best_dist = d
                best_idx = k

        if best_idx is None:
            # Not necessarily an error; just means nothing found in the grid
            self._warn_throttled(
                "No valid cells found in GetClosestGrid response", period_s=1.0
            )
            return None

        return float(res.x[best_idx]), float(res.y[best_idx])


def main():
    rclpy.init()
    node = PlantFollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
