"""Expose the original Reeman move_base stop duration to the dashboard."""

from pathlib import Path
import tempfile
import threading

import rospy
from dynamic_reconfigure.msg import Config, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure


PARAMETER = "stop_move_time"
MIN_SECONDS, MAX_SECONDS = 0.0, 20.0


class ObstacleWait:
    def __init__(self, path):
        self.path = path
        self.current = None
        self.lock = threading.Lock()
        self.configure = rospy.ServiceProxy("/move_base/set_parameters", Reconfigure)
        self.restoring = path.is_file()
        self.subscriber = rospy.Subscriber("/move_base/parameter_updates", Config,
                                           self.on_configuration, queue_size=1)
        if self.restoring:
            threading.Thread(target=self.restore, daemon=True).start()

    @staticmethod
    def seconds(value):
        if type(value) not in (float, int) or not MIN_SECONDS <= value <= MAX_SECONDS:
            raise ValueError("Thời gian chờ phải nằm trong khoảng 0–20 giây")
        return float(value)

    def on_configuration(self, config):
        values = {parameter.name: parameter.value for parameter in config.doubles}
        if PARAMETER not in values:
            return None
        actual = {"seconds": values[PARAMETER], "min_seconds": MIN_SECONDS,
                  "max_seconds": MAX_SECONDS,
                  "recovery_seconds": values.get("controller_patience")}
        self.current = actual
        return actual

    def status(self):
        return None if self.restoring else self.current

    def apply(self, seconds):
        reply = self.configure(Config(doubles=[DoubleParameter(name=PARAMETER, value=seconds)]))
        actual = self.on_configuration(reply.config)
        if actual is None:
            raise RuntimeError("Reeman chưa xác nhận thời gian chờ")
        return actual

    def restore(self):
        try:
            seconds = self.seconds(float(self.path.read_text()))
            self.configure.wait_for_service(timeout=45)
            with self.lock:
                self.apply(seconds)
        except (OSError, ValueError, RuntimeError, rospy.ROSException) as error:
            rospy.logwarn("Could not restore Reeman obstacle wait: %s", error)
        finally:
            self.restoring = False

    def set(self, value):
        seconds = self.seconds(value)
        with self.lock:
            if self.status() is None:
                raise RuntimeError("Đang chờ move_base nạp cấu hình")
            actual = self.apply(seconds)
            try:
                with tempfile.NamedTemporaryFile(mode="w", dir=self.path.parent, delete=False) as stream:
                    pending = Path(stream.name)
                    try:
                        stream.write(f"{actual['seconds']}\n")
                        stream.flush()
                        pending.replace(self.path)
                    finally:
                        pending.unlink(missing_ok=True)
            except OSError as error:
                raise RuntimeError("Đã áp dụng thời gian chờ, nhưng chưa lưu được cho lần mở sau") from error
            return actual
