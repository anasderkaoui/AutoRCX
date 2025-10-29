```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time

# GPIO pin definitions (BOARD numbering)
PIN_CLK = 11  # CLK channel pin
PIN_DT  = 12  # DT channel pin

# Encoder / wheel parameters
PULSES_PER_REV   = 60.0      # Encoder pulses per wheel revolution (20 PPR × 3 shaft turns)
WHEEL_CIRC_MM    = 385.725   # Wheel circumference in mm (with gear ratio)
DEBOUNCE_S       = 200e-6    # Debounce interval in seconds (200 μs)
EMA_ALPHA        = 0.2       # Exponential moving average factor for speed smoothing
PUBLISH_INTERVAL = 0.1       # Publish interval in seconds

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PIN_DT,  GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Shared state (updated in ISR)
        self.pulse_count     = 0
        self.last_clk_state  = GPIO.input(PIN_CLK)
        self.last_time_edge  = time.monotonic()

        # Distance & speed
        self.total_distance_mm  = 0.0
        self.raw_speed_mm_s     = 0.0
        self.smooth_speed_mm_s  = 0.0
        self.last_publish_time  = time.monotonic()
        self.last_count_snapshot = 0

        # ROS publishers
        self.dist_pub  = self.create_publisher(Float32, '/encoder_distance', 10)
        self.speed_pub = self.create_publisher(Float32, '/encoder_speed', 10)

        # Interrupt on CLK only (both edges)
        GPIO.add_event_detect(
            PIN_CLK,
            GPIO.BOTH,
            callback=self._handle_edge,
            bouncetime=int(DEBOUNCE_S * 1000) or 1
        )

        # Timer for compute & publish
        self.create_timer(PUBLISH_INTERVAL, self._on_timer)

    def _handle_edge(self, channel):
        now = time.monotonic()
        # Debounce
        if (now - self.last_time_edge) < DEBOUNCE_S:
            return
        self.last_time_edge = now

        clk = GPIO.input(PIN_CLK)
        dt  = GPIO.input(PIN_DT)

        # Only process on CLK state change
        if clk != self.last_clk_state:
            # Determine direction: if CLK != DT then forward (+1), else backward (-1)
            delta = +1 if (clk != dt) else -1
            self.pulse_count += delta
            self.last_clk_state = clk

    def _on_timer(self):
        now = time.monotonic() # we're sure time never goes back
        dt = now - self.last_publish_time
        if dt <= 0:
            return

        # Snapshot pulse count
        count = self.pulse_count

        # Compute total distance (mm)
        self.total_distance_mm = (count / PULSES_PER_REV) * WHEEL_CIRC_MM

        # Compute raw speed over interval (mm/s)
        delta = count - self.last_count_snapshot
        self.last_count_snapshot = count
        self.raw_speed_mm_s = ((delta / PULSES_PER_REV) * WHEEL_CIRC_MM) / dt

        # EMA smoothing
        # self.smooth_speed_mm_s = (
        #     EMA_ALPHA * self.raw_speed_mm_s +
        #     (1 - EMA_ALPHA) * self.smooth_speed_mm_s
        # )

        # Convert to meters
        dist_m = self.total_distance_mm / 1000.0
        speed_raw_m_s = self.raw_speed_mm_s / 1000.0
        speed_smooth_m_s = self.smooth_speed_mm_s / 1000.0

        # Publish distance
        dist_msg = Float32()
        dist_msg.data = dist_m
        self.dist_pub.publish(dist_msg)

        # Publish speed (smoothed)
        speed_msg = Float32()
        speed_msg.data = speed_smooth_m_s
        self.speed_pub.publish(speed_msg)

        # (Optional) print for debugging
        # self.get_logger().debug(
        #     f"Distance: {dist_m:.4f} m | Raw: {speed_raw_m_s:.4f} m/s | "
        #     f"Smoothed: {speed_smooth_m_s:.4f} m/s"
        # )

        self.last_publish_time = now

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# With 4x decoding !

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import Jetson.GPIO as GPIO
# import time
# from threading import Lock
# from math import pi

# # GPIO pin definitions (BOARD numbering)
# PIN_CLK = 11  # CLK channel pin
# PIN_DT  = 12  # DT channel pin

# # Encoder / wheel parameters
# PPR = 20.0              # Pulses per encoder revolution (physical PPR)
# GEAR_RATIO = 1.93       # Encoder revolutions per wheel revolution
# DECODING_FACTOR = 4     # Quadrature decoding factor (4 edges per pulse)
# PULSES_PER_REV = PPR * GEAR_RATIO * DECODING_FACTOR  # Total counts per wheel rev
# WHEEL_RADIUS_M = 0.051  # Wheel radius in meters
# WHEEL_CIRC_MM = 2 * pi * WHEEL_RADIUS_M * 1000      # Wheel circumference in mm

# DEBOUNCE_S       = 200e-6   # Debounce interval in seconds (200 μs)
# EMA_ALPHA        = 0.2      # EMA factor for speed smoothing
# PUBLISH_INTERVAL = 0.1      # Publish interval in seconds

# # Gray-code transition table (4× decoding)
# TRANS = {
#     (0,1): +1, (1,3): +1, (3,2): +1, (2,0): +1,
#     (0,2): -1, (2,3): -1, (3,1): -1, (1,0): -1
# }

# class EncoderNode(Node):
#     def __init__(self):
#         super().__init__('encoder_node')

#         # GPIO init
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(PIN_CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#         GPIO.setup(PIN_DT,  GPIO.IN, pull_up_down=GPIO.PUD_UP)

#         # Shared state
#         self.pulse_count = 0
#         self.last_state = ((GPIO.input(PIN_CLK) << 1) | GPIO.input(PIN_DT)) & 0x3
#         self.last_time_edge = time.monotonic()
#         self.lock = Lock()

#         # Distance & speed
#         self.total_distance_mm   = 0.0
#         self.raw_speed_mm_s      = 0.0
#         self.smooth_speed_mm_s   = 0.0
#         self.last_publish_time   = time.monotonic()
#         self.last_count_snapshot = 0

#         # ROS publishers
#         self.dist_pub      = self.create_publisher(Float32, '/encoder_distance', 10)
#         self.raw_speed_pub = self.create_publisher(Float32, '/encoder_raw_speed', 10)
#         self.speed_pub     = self.create_publisher(Float32, '/encoder_speed', 10)

#         # Interrupt attachments (both edges)
#         GPIO.add_event_detect(PIN_CLK, GPIO.BOTH, callback=self._handle_edge, bouncetime=0)
#         GPIO.add_event_detect(PIN_DT,  GPIO.BOTH, callback=self._handle_edge, bouncetime=0)

#         # Timer for compute & publish
#         self.create_timer(PUBLISH_INTERVAL, self._on_timer)

#     def _handle_edge(self, channel):
#         now = time.monotonic()
#         if (now - self.last_time_edge) < DEBOUNCE_S:
#             return
#         self.last_time_edge = now

#         # Gray code state machine
#         a = GPIO.input(PIN_CLK)
#         b = GPIO.input(PIN_DT)
#         new_state = ((a << 1) | b) & 0x3
#         step = TRANS.get((self.last_state, new_state), 0)

#         if step:
#             with self.lock:
#                 self.pulse_count += step
#         self.last_state = new_state

#     def _on_timer(self):
#         now = time.monotonic()
#         dt = now - self.last_publish_time
#         if dt <= 0:
#             return

#         # snapshot
#         with self.lock:
#             count = self.pulse_count

#         # Distance (mm)
#         self.total_distance_mm = (count / PULSES_PER_REV) * WHEEL_CIRC_MM

#         # Raw speed over interval (mm/s)
#         delta = count - self.last_count_snapshot
#         self.last_count_snapshot = count
#         self.raw_speed_mm_s = ((delta / PULSES_PER_REV) * WHEEL_CIRC_MM) / dt

#         # EMA smoothing
#         self.smooth_speed_mm_s = (
#             EMA_ALPHA * self.raw_speed_mm_s +
#             (1 - EMA_ALPHA) * self.smooth_speed_mm_s
#         )

#         # Convert to meters
#         dist_m            = self.total_distance_mm / 1000.0
#         speed_raw_m_s     = self.raw_speed_mm_s / 1000.0

#         if self.smooth_speed_mm_s == 0.0:
#             speed_smooth_m_s = 0.0
#         else:
#             speed_smooth_m_s = self.smooth_speed_mm_s / 1000.0

#         # Publish distance
#         dist_msg = Float32()
#         dist_msg.data = dist_m
#         self.dist_pub.publish(dist_msg)

#         # Publish raw speed
#         raw_msg = Float32()
#         raw_msg.data = speed_raw_m_s
#         self.raw_speed_pub.publish(raw_msg)

#         # Publish smoothed speed
#         smooth_msg = Float32()
#         smooth_msg.data = speed_smooth_m_s
#         self.speed_pub.publish(smooth_msg)

#         self.last_publish_time = now

#     def destroy_node(self):
#         GPIO.cleanup()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = EncoderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
```
