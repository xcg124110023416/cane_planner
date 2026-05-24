#!/usr/bin/env python3

import math

import rospy
from std_msgs.msg import Bool, Float64MultiArray, String


class MpcEventMonitor:
    def __init__(self):
        self.reason_topic = rospy.get_param("~stop_reason_topic", "/mpc/stop_reason")
        self.advice_topic = rospy.get_param("~stop_advice_topic", "/mpc/stop_advice")
        self.metrics_topic = rospy.get_param("~debug_metrics_topic", "/mpc/debug_metrics")
        self.warn_active_duration = rospy.get_param("~warn_active_duration", 8.0)
        self.update_rate = rospy.get_param("~update_rate", 5.0)
        self.print_reason_updates = rospy.get_param("~print_reason_updates", False)

        self.current_reason = "OK"
        self.current_advice = False
        self.reported_reason = "OK"
        self.active = False
        self.active_start = None
        self.last_warn_time = rospy.Time(0)
        self.latest_metrics = None

        self.stop_count = 0
        self.reason_counts = {}
        self.total_stop_duration = 0.0

        rospy.Subscriber(self.metrics_topic, Float64MultiArray, self.metrics_callback, queue_size=10)
        rospy.Subscriber(self.advice_topic, Bool, self.advice_callback, queue_size=10)
        rospy.Subscriber(self.reason_topic, String, self.reason_callback, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / max(0.1, self.update_rate)), self.timer_callback)

        rospy.on_shutdown(self.print_summary)
        rospy.loginfo("MPC event monitor listening: %s, %s, %s",
                      self.reason_topic, self.advice_topic, self.metrics_topic)

    def metrics_callback(self, msg):
        self.latest_metrics = msg.data

    def advice_callback(self, msg):
        self.current_advice = msg.data

    def reason_callback(self, msg):
        self.current_reason = msg.data if msg.data else "OK"

    def timer_callback(self, _event):
        self.update_state()

    def update_state(self):
        now = rospy.Time.now()
        reason = self.current_reason
        should_stop = self.current_advice or reason != "OK"
        display_reason = reason if reason != "OK" else "STOP_ADVICE"

        if should_stop and not self.active:
            self.active = True
            self.active_start = now
            self.last_warn_time = now
            self.reported_reason = display_reason
            self.stop_count += 1
            self.add_reason_count(display_reason)
            print("[{:.2f}s] STOP entered: {} {}".format(
                now.to_sec(), display_reason, self.metrics_text()))
        elif (should_stop and self.active and self.print_reason_updates and
              display_reason != self.reported_reason):
            self.add_reason_count(display_reason)
            print("[{:.2f}s] STOP reason updated: {} -> {} {}".format(
                now.to_sec(), self.reported_reason, display_reason, self.metrics_text()))
            self.reported_reason = display_reason
        elif not should_stop and self.active:
            duration = 0.0
            if self.active_start is not None:
                duration = (now - self.active_start).to_sec()
            self.total_stop_duration += max(0.0, duration)
            print("[{:.2f}s] STOP released: duration={:.2f}s {}".format(
                now.to_sec(), duration, self.metrics_text()))
            self.active = False
            self.active_start = None
            self.last_warn_time = rospy.Time(0)
            self.reported_reason = "OK"

        if self.active and self.warn_active_duration > 0.0 and self.active_start is not None:
            active_duration = (now - self.active_start).to_sec()
            since_warn = (now - self.last_warn_time).to_sec()
            if active_duration >= self.warn_active_duration and since_warn >= self.warn_active_duration:
                print("[{:.2f}s] STOP still active: duration={:.2f}s reason={} {}".format(
                    now.to_sec(), active_duration, self.reported_reason, self.metrics_text()))
                self.last_warn_time = now

    def metrics_text(self):
        data = self.latest_metrics
        if data is None or len(data) < 10:
            return "metrics=NA"

        valid = self.fmt(data[1])
        clearance = self.fmt(data[3])
        ttc = self.fmt(data[4])
        plan_valid = int(data[8]) if len(data) > 8 else -1
        risk_scale = self.fmt(data[9]) if len(data) > 9 else "NA"
        best_clearance = self.fmt(data[10]) if len(data) > 10 else "NA"
        best_ttc = self.fmt(data[11]) if len(data) > 11 else "NA"
        return ("valid={} best_clearance={} best_ttc={} "
                "global_clearance={} global_ttc={} plan_valid={} risk_scale={}").format(
            valid, best_clearance, best_ttc, clearance, ttc, plan_valid, risk_scale)

    @staticmethod
    def fmt(value):
        if value is None or not math.isfinite(value):
            return "NA"
        return "{:.2f}".format(value)

    def add_reason_count(self, reason):
        if reason == "OK":
            return
        self.reason_counts[reason] = self.reason_counts.get(reason, 0) + 1

    def print_summary(self):
        if self.active and self.active_start is not None:
            now = rospy.Time.now()
            self.total_stop_duration += max(0.0, (now - self.active_start).to_sec())

        if self.stop_count == 0:
            print("MPC event summary: no stop/yield events observed.")
            return

        reason_text = ", ".join(
            "{}={}".format(reason, count)
            for reason, count in sorted(self.reason_counts.items()))
        print("MPC event summary: stops={} total_stop_duration={:.2f}s reasons=[{}]".format(
            self.stop_count, self.total_stop_duration, reason_text))


if __name__ == "__main__":
    rospy.init_node("mpc_event_monitor")
    MpcEventMonitor()
    rospy.spin()
