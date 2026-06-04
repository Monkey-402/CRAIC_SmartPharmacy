#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""订阅 JudgementReport，按 CRAIC 规则经 TCP 发送 JSON 至裁判软件。"""

import json
import socket
import threading

import rospy
from move_nav.msg import JudgementReport
from std_msgs.msg import Bool


class JudgementTcpSender(object):
    def __init__(self):
        rospy.init_node("judgement_tcp_sender", anonymous=False)

        self.server_ip = rospy.get_param("~server_ip", "192.168.1.100")
        self.server_port = int(rospy.get_param("~server_port", 8888))
        self.send_rate = float(rospy.get_param("~send_rate", 1.5))
        self.topic = rospy.get_param("~input_topic", "/judgement/report")
        self.append_newline = rospy.get_param("~append_newline", True)
        self.connected_topic = rospy.get_param(
            "~connected_topic", "/judgement/peer_connected"
        )
        self.connect_retry_interval = float(
            rospy.get_param("~connect_retry_interval", 2.0)
        )

        self._connected_pub = rospy.Publisher(
            self.connected_topic, Bool, queue_size=1, latch=True
        )
        self._publish_connected(False)

        self._lock = threading.Lock()
        self._latest = None
        self._last_report_time = None
        self._sock = None
        # 主控在 home/standby 停止发布后，超过该时间不再向裁判 TCP 发旧数据
        self._max_report_age_sec = float(
            rospy.get_param("~max_report_age_sec", max(2.0 / self.send_rate, 1.0))
        )

        self._sub = rospy.Subscriber(
            self.topic, JudgementReport, self._report_cb, queue_size=10
        )
        self._send_timer = rospy.Timer(
            rospy.Duration(1.0 / self.send_rate), self._send_timer_cb
        )
        self._connect_timer = rospy.Timer(
            rospy.Duration(self.connect_retry_interval), self._connect_retry_cb
        )

        rospy.loginfo(
            "judgement_tcp_sender: topic=%s -> %s:%d @ %.2f Hz "
            "(connect_retry=%.1fs)",
            self.topic,
            self.server_ip,
            self.server_port,
            self.send_rate,
            self.connect_retry_interval,
        )

    def _report_cb(self, msg):
        with self._lock:
            self._latest = msg
            self._last_report_time = rospy.Time.now()

    def _to_payload(self, msg):
        if len(msg.odom) < 2:
            rospy.logwarn_throttle(5.0, "odom 至少需要 2 个元素 [x, y]")
            return None

        data = {
            "id": msg.id,
            "speed": msg.speed,
            "odom": [msg.odom[0], msg.odom[1]],
            "task": msg.task,
            "CV1": msg.CV1,
            "CV2": msg.CV2,
        }
        text = json.dumps(data, separators=(",", ":"), ensure_ascii=False)
        if self.append_newline:
            text += "\n"
        return text.encode("utf-8")

    def _publish_connected(self, connected):
        msg = Bool()
        msg.data = bool(connected)
        self._connected_pub.publish(msg)

    def _connect(self):
        if self._sock is not None:
            return True
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self.server_ip, self.server_port))
            sock.settimeout(None)
            self._sock = sock
            self._publish_connected(True)
            rospy.loginfo("裁判 TCP 已连接 %s:%d", self.server_ip, self.server_port)
            return True
        except socket.error as exc:
            rospy.logwarn_throttle(5.0, "连接裁判软件失败: %s", exc)
            self._close_socket(publish=False)
            return False

    def _connect_retry_cb(self, _event):
        """home/standby 无上报时也周期性尝试连接，供主控开赛前等待 peer_connected。"""
        if self._sock is not None:
            return
        self._connect()

    def _close_socket(self, publish=True):
        had_sock = self._sock is not None
        if self._sock is not None:
            try:
                self._sock.close()
            except socket.error:
                pass
            self._sock = None
        if had_sock and publish:
            self._publish_connected(False)
            rospy.loginfo("裁判 TCP 已断开")

    def _send_timer_cb(self, _event):
        with self._lock:
            if self._latest is None or self._last_report_time is None:
                return
            age = (rospy.Time.now() - self._last_report_time).to_sec()
            if age > self._max_report_age_sec:
                # home/standby 主控停发后：不再发旧 JSON，但保持 TCP（避免 connect_retry 抖连）
                return
            msg = self._latest

        payload = self._to_payload(msg)
        if payload is None:
            return

        if self._sock is None and not self._connect():
            return

        try:
            self._sock.sendall(payload)
        except socket.error as exc:
            rospy.logwarn("发送失败，将重连: %s", exc)
            self._close_socket()

    def shutdown(self):
        self._send_timer.shutdown()
        self._connect_timer.shutdown()
        self._close_socket()


if __name__ == "__main__":
    node = JudgementTcpSender()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
