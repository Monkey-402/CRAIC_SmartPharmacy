#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""双车 TCP 桥接：ROS Int32 <-> 对端整数，不依赖跨车 ROS 通信。

实现两车协同场景下的跨车整数通信：
  本车 ROS 话题 /car_link/send   →  TCP 发往对端
  对端 TCP 数据                 →  本车 ROS 话题 /car_link/recv

角色: server（监听, 1号车） / client（主动连接, 2号车）
"""

import socket
import threading

import rospy
from std_msgs.msg import Int32


class CarTcpBridge(object):
    """双向桥接节点：将本车 Int32 消息通过 TCP 发给对端车，同时将
    对端发来的 TCP 数据解析为 Int32 发布到 ROS 话题。"""

    def __init__(self):
        rospy.init_node("car_tcp_bridge", anonymous=False)

        # ---- 从参数服务器读取配置 ----
        self.role = rospy.get_param("~role", "client").lower()          # server / client
        self.peer_ip = rospy.get_param("~peer_ip", "192.168.1.102")     # 对端车 IP（client 角色使用）
        self.port = int(rospy.get_param("~port", 9000))                 # TCP 端口
        self.send_topic = rospy.get_param("~send_topic", "/car_link/send")  # 本车要发出的 Int32 话题
        self.recv_topic = rospy.get_param("~recv_topic", "/car_link/recv")  # 本车接收到的 Int32 话题
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))  # 断线重连间隔(秒)

        # ---- 内部状态 ----
        self._sock = None                     # 当前 TCP socket（None = 未连接）
        self._sock_lock = threading.Lock()    # 保护 _sock 的线程锁
        self._recv_buffer = ""                # TCP 接收缓冲区（处理粘包）
        self._stop = threading.Event()        # 通知后台线程退出的标志

        # ---- ROS 话题 ----
        # 将对端发来的整数发布到 recv_topic
        self._pub = rospy.Publisher(self.recv_topic, Int32, queue_size=10)
        # 订阅本车要发出的整数（send_topic）
        self._sub = rospy.Subscriber(
            self.send_topic, Int32, self._send_cb, queue_size=10
        )

        # ---- 根据角色启动后台连接线程 ----
        if self.role == "server":
            self._conn_thread = threading.Thread(target=self._server_loop)
            rospy.loginfo(
                "car_tcp_bridge [server]: listen :%d, send=%s, recv=%s",
                self.port, self.send_topic, self.recv_topic,
            )
        elif self.role == "client":
            self._conn_thread = threading.Thread(target=self._client_loop)
            rospy.loginfo(
                "car_tcp_bridge [client]: connect %s:%d, send=%s, recv=%s",
                self.peer_ip, self.port, self.send_topic, self.recv_topic,
            )
        else:
            raise rospy.ROSInitException("role 必须是 server 或 client")

        self._conn_thread.daemon = True   # 设为守护线程, 主线程退出时自动杀掉
        self._conn_thread.start()

    # ========== Socket 管理（线程安全） ==========

    def _close_socket_unlocked(self):
        """关闭当前 socket（无锁版本, 调用者需已持有 _sock_lock）。

        同时使用 shutdown 和 close 确保双方都感知到连接断开,
        并清空接收缓冲区, 避免旧数据干扰重连后的状态。
        """
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)   # 通知对端不再收发
            except socket.error:
                pass
            try:
                self._sock.close()
            except socket.error:
                pass
            self._sock = None
        self._recv_buffer = ""    # 清空残留在缓冲区的半截数据

    def _close_socket(self):
        """关闭 socket 的线程安全封装。"""
        with self._sock_lock:
            self._close_socket_unlocked()

    def _set_socket(self, sock):
        """原子操作：关闭旧 socket 并替换为新 socket。

        在 server_loop 或 client_loop 中成功建立连接后调用,
        保证同一时刻只有一个 socket 处于活跃状态。
        """
        with self._sock_lock:
            self._close_socket_unlocked()
            self._sock = sock
            self._recv_buffer = ""

    # ========== 发送（ROS 回调线程） ==========

    def _send_cb(self, msg):
        """订阅回调：本车有人往 send_topic 发 Int32 时被调用。

        将整数转成 "123\\n" 格式的 UTF-8 字节流, 通过 TCP 发往对端。
        如果当前未连接, 丢弃消息并打印警告。
        """
        # 格式化: 整数 + 换行符作为协议分隔
        line = "{0}\n".format(int(msg.data)).encode("utf-8")

        # 线程安全地拿到当前 socket 引用（只读, 不加锁整个发送过程）
        with self._sock_lock:
            sock = self._sock

        if sock is None:
            # 还没连上对端, 消息丢弃（不阻塞重试逻辑）
            rospy.logwarn_throttle(5.0, "未连接对端，丢弃发送: %d", msg.data)
            return

        try:
            sock.sendall(line)
            rospy.logdebug("已发送: %d", msg.data)
        except socket.error as exc:
            rospy.logwarn("发送失败: %s", exc)
            self._close_socket()   # socket 已损坏, 触发重连

    # ========== 接收（后台线程） ==========

    def _handle_received_line(self, line):
        """处理一行从对端收到的完整数据。

        去掉首尾空白后转为整数, 发布到 ROS 的 recv_topic。
        """
        line = line.strip()
        if not line:
            return   # 空行跳过
        try:
            value = int(line)
        except ValueError:
            rospy.logwarn("收到非法数据: %r", line)
            return
        self._pub.publish(Int32(data=value))
        rospy.loginfo("收到对端: %d -> %s", value, self.recv_topic)

    def _recv_loop(self, sock):
        """接收循环：持续从 TCP socket 读取数据, 逐行解析整数。

        由于 TCP 是流式协议, 可能存在"粘包"（一次 recv 收到多条数据）,
        用缓冲区 + split("\\n") 按行拆分:
            1. recv 原始数据追加到 _recv_buffer
            2. 循环切出每一个完整的 "\\n" 之前的行
            3. 剩下的半截留在缓冲区等下一次 recv
        """
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                chunk = sock.recv(4096)          # 最多读 4096 字节
                if not chunk:                    # recv 返回空字节 = 对端已关闭连接
                    rospy.logwarn("对端断开连接")
                    break
                # 追加到接收缓冲区（replace 容错: 非法 UTF-8 用 ? 代替）
                self._recv_buffer += chunk.decode("utf-8", errors="replace")
                # 按换行符分割, 处理完整的行
                while "\n" in self._recv_buffer:
                    line, self._recv_buffer = self._recv_buffer.split("\n", 1)
                    self._handle_received_line(line)
            except socket.error as exc:
                rospy.logwarn("接收失败: %s", exc)
                break

        # 退出循环 = 连接断开或节点关闭
        self._close_socket()

    # ========== 连接管理（后台线程） ==========

    def _server_loop(self):
        """Server 角色的连接循环（在独立后台线程中运行）。

        流程:
            1. 监听端口, accept 等待对端连接
            2. 一旦有车连上来, 存入 _set_socket
            3. 进入 _recv_loop 持续接收
            4. 接收循环退出（连接断开）→ 回去 accept 等待下一次连接
        """
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许快速重用端口
        server_sock.bind(("", self.port))   # 监听所有网卡
        server_sock.listen(1)               # 只接受一个对端连接
        server_sock.settimeout(1.0)         # accept 超时 1 秒, 方便检查 shutdown

        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                conn, addr = server_sock.accept()   # 阻塞等待对端连接
            except socket.timeout:
                continue   # 超时是正常现象, 继续等
            except socket.error as exc:
                rospy.logwarn_throttle(5.0, "监听失败: %s", exc)
                rospy.sleep(self.reconnect_interval)
                continue

            rospy.loginfo("对端已连接: %s:%d", addr[0], addr[1])
            conn.settimeout(None)           # 连接后设为阻塞模式
            self._set_socket(conn)          # 替换为新的连接
            self._recv_loop(conn)           # 进入接收循环, 阻塞到断开

        server_sock.close()

    def _client_loop(self):
        """Client 角色的连接循环（在独立后台线程中运行）。

        流程:
            1. 主动连接对端 IP:端口
            2. 成功则存入 _set_socket 并进入 _recv_loop
            3. 失败或断开则等待 reconnect_interval 秒后重试
            4. 无限循环直到节点关闭
        """
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)                              # 连接超时 3 秒
                sock.connect((self.peer_ip, self.port))
                sock.settimeout(None)                             # 连接后阻塞模式
                rospy.loginfo("已连接对端 %s:%d", self.peer_ip, self.port)
                self._set_socket(sock)
                self._recv_loop(sock)                             # 阻塞到断开
            except socket.error as exc:
                rospy.logwarn_throttle(5.0, "连接失败，%ss 后重试: %s",
                                       self.reconnect_interval, exc)
                self._close_socket()
                rospy.sleep(self.reconnect_interval)              # 等待后重试

    # ========== 节点关闭 ==========

    def shutdown(self):
        """ROS 节点退出时的清理。

        设置 _stop 标志 → 后台线程的 loop 检测到后退出 →
        socket 被关闭 → 线程自然结束。
        """
        self._stop.set()       # 通知后台线程退出
        self._close_socket()   # 关闭 socket（让 recv/accept 立即返回）


if __name__ == "__main__":
    node = CarTcpBridge()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
