#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""订阅 JudgementReport，按 CRAIC 规则经 TCP 发送 JSON 至裁判软件。"""

import json
import socket
import threading

import rospy
from move_nav.msg import JudgementReport


class JudgementTcpSender(object):
    def __init__(self):
        rospy.init_node("judgement_tcp_sender", anonymous=False)

        self.server_ip = rospy.get_param("~server_ip", "192.168.1.100")
        self.server_port = int(rospy.get_param("~server_port", 8888))
        self.send_rate = float(rospy.get_param("~send_rate", 1.5))
        self.topic = rospy.get_param("~input_topic", "/judgement/report")
        self.append_newline = rospy.get_param("~append_newline", True)

        self._lock = threading.Lock()
        self._latest = None
        self._sock = None

        self._sub = rospy.Subscriber(
            self.topic, JudgementReport, self._report_cb, queue_size=10
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.send_rate), self._send_timer_cb
        )

        rospy.loginfo(
            "judgement_tcp_sender: topic=%s -> %s:%d @ %.2f Hz",
            self.topic,
            self.server_ip,
            self.server_port,
            self.send_rate,
        )

    def _report_cb(self, msg):
        with self._lock:
            self._latest = msg

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

    def _connect(self):
        if self._sock is not None:
            return True
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self.server_ip, self.server_port))
            sock.settimeout(None)
            self._sock = sock
            rospy.loginfo("已连接裁判软件 %s:%d", self.server_ip, self.server_port)
            return True
        except socket.error as exc:
            rospy.logwarn_throttle(5.0, "连接裁判软件失败: %s", exc)
            self._close_socket()
            return False

    def _close_socket(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except socket.error:
                pass
            self._sock = None

    def _send_timer_cb(self, _event):
        with self._lock:
            if self._latest is None:
                return
            msg = self._latest

        payload = self._to_payload(msg)
        if payload is None:
            return

        if not self._connect():
            return

        try:
            self._sock.sendall(payload)
        except socket.error as exc:
            rospy.logwarn("发送失败，将重连: %s", exc)
            self._close_socket()

    def shutdown(self):
        self._timer.shutdown()
        self._close_socket()


if __name__ == "__main__":
    node = JudgementTcpSender()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
