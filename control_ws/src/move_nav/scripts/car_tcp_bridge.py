#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""双车 TCP 桥接：ROS Int32 <-> 对端整数，不依赖跨车 ROS 通信。"""

import socket
import threading

import rospy
from std_msgs.msg import Int32


class CarTcpBridge(object):
    def __init__(self):
        rospy.init_node("car_tcp_bridge", anonymous=False)

        self.role = rospy.get_param("~role", "client").lower()
        self.peer_ip = rospy.get_param("~peer_ip", "192.168.1.102")
        self.port = int(rospy.get_param("~port", 9000))
        self.send_topic = rospy.get_param("~send_topic", "/car_link/send")
        self.recv_topic = rospy.get_param("~recv_topic", "/car_link/recv")
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))

        self._sock = None
        self._sock_lock = threading.Lock()
        self._recv_buffer = ""
        self._stop = threading.Event()

        self._pub = rospy.Publisher(self.recv_topic, Int32, queue_size=10)
        self._sub = rospy.Subscriber(
            self.send_topic, Int32, self._send_cb, queue_size=10
        )

        if self.role == "server":
            self._conn_thread = threading.Thread(target=self._server_loop)
            rospy.loginfo(
                "car_tcp_bridge [server]: listen :%d, send=%s, recv=%s",
                self.port,
                self.send_topic,
                self.recv_topic,
            )
        elif self.role == "client":
            self._conn_thread = threading.Thread(target=self._client_loop)
            rospy.loginfo(
                "car_tcp_bridge [client]: connect %s:%d, send=%s, recv=%s",
                self.peer_ip,
                self.port,
                self.send_topic,
                self.recv_topic,
            )
        else:
            raise rospy.ROSInitException("role 必须是 server 或 client")

        self._conn_thread.daemon = True
        self._conn_thread.start()

    def _close_socket_unlocked(self):
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            try:
                self._sock.close()
            except socket.error:
                pass
            self._sock = None
        self._recv_buffer = ""

    def _close_socket(self):
        with self._sock_lock:
            self._close_socket_unlocked()

    def _set_socket(self, sock):
        with self._sock_lock:
            self._close_socket_unlocked()
            self._sock = sock
            self._recv_buffer = ""

    def _send_cb(self, msg):
        line = "{0}\n".format(int(msg.data)).encode("utf-8")
        with self._sock_lock:
            sock = self._sock
        if sock is None:
            rospy.logwarn_throttle(5.0, "未连接对端，丢弃发送: %d", msg.data)
            return
        try:
            sock.sendall(line)
            rospy.logdebug("已发送: %d", msg.data)
        except socket.error as exc:
            rospy.logwarn("发送失败: %s", exc)
            self._close_socket()

    def _handle_received_line(self, line):
        line = line.strip()
        if not line:
            return
        try:
            value = int(line)
        except ValueError:
            rospy.logwarn("收到非法数据: %r", line)
            return
        self._pub.publish(Int32(data=value))
        rospy.loginfo("收到对端: %d -> %s", value, self.recv_topic)

    def _recv_loop(self, sock):
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    rospy.logwarn("对端断开连接")
                    break
                self._recv_buffer += chunk.decode("utf-8", errors="replace")
                while "\n" in self._recv_buffer:
                    line, self._recv_buffer = self._recv_buffer.split("\n", 1)
                    self._handle_received_line(line)
            except socket.error as exc:
                rospy.logwarn("接收失败: %s", exc)
                break
        self._close_socket()

    def _server_loop(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("", self.port))
        server_sock.listen(1)
        server_sock.settimeout(1.0)

        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                conn, addr = server_sock.accept()
            except socket.timeout:
                continue
            except socket.error as exc:
                rospy.logwarn_throttle(5.0, "监听失败: %s", exc)
                rospy.sleep(self.reconnect_interval)
                continue

            rospy.loginfo("对端已连接: %s:%d", addr[0], addr[1])
            conn.settimeout(None)
            self._set_socket(conn)
            self._recv_loop(conn)

        server_sock.close()

    def _client_loop(self):
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((self.peer_ip, self.port))
                sock.settimeout(None)
                rospy.loginfo("已连接对端 %s:%d", self.peer_ip, self.port)
                self._set_socket(sock)
                self._recv_loop(sock)
            except socket.error as exc:
                rospy.logwarn_throttle(5.0, "连接失败，%ss 后重试: %s",
                                       self.reconnect_interval, exc)
                self._close_socket()
                rospy.sleep(self.reconnect_interval)

    def shutdown(self):
        self._stop.set()
        self._close_socket()


if __name__ == "__main__":
    node = CarTcpBridge()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
