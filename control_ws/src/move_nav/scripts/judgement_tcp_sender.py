#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""订阅 JudgementReport，按 CRAIC 规则经 TCP 发送 JSON 至裁判软件。

核心流程:
    /judgement/report (ROS话题)  →  _report_cb 缓存最新一条消息
           ↓
    定时器(默认1.5Hz)  →  _send_timer_cb 触发
           →  _to_payload(msg)  转成 JSON 字节流
           →  _connect()        确保 TCP 已连上裁判软件
           →  socket.sendall()  发送数据
"""

import json
import socket
import threading

import rospy
from move_nav.msg import JudgementReport


class JudgementTcpSender(object):
    """ROS 节点: 将 JudgementReport 消息通过 TCP 转发给裁判评分软件。"""

    def __init__(self):
        # ---- ROS 节点初始化 ----
        rospy.init_node("judgement_tcp_sender", anonymous=False)

        # ---- 从 launch 文件 / 参数服务器读取配置 ----
        self.server_ip = rospy.get_param("~server_ip", "192.168.1.100")   # 裁判软件 IP
        self.server_port = int(rospy.get_param("~server_port", 8888))      # 裁判软件 TCP 端口
        self.send_rate = float(rospy.get_param("~send_rate", 1.5))         # 发送频率(Hz), 规则要求 1-2Hz
        self.topic = rospy.get_param("~input_topic", "/judgement/report")  # 订阅的 ROS 话题名
        self.append_newline = rospy.get_param("~append_newline", True)     # JSON 末尾是否加换行符

        # ---- 线程安全与状态 ----
        # _report_cb 运行在 ROS 回调线程, _send_timer_cb 运行在定时器线程,
        # 这两个线程同时读写 _latest, 所以需要加锁保护。
        self._lock = threading.Lock()
        self._latest = None   # 缓存最新的 JudgementReport 消息
        self._sock = None     # TCP socket, None 表示未连接

        # ---- 订阅 /judgement/report 话题 ----
        self._sub = rospy.Subscriber(
            self.topic,
            JudgementReport,     # 消息类型
            self._report_cb,     # 收到消息时的回调函数
            queue_size=10,       # 消息队列长度, 防止处理不过来时有缓冲
        )

        # ---- 定时发送 ----
        # 按 send_rate 频率(如 1.5Hz) 周期性调用 _send_timer_cb,
        # Duration(1.0/1.5) = 0.667 秒触发一次
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

    # ========== 消息接收 ==========

    def _report_cb(self, msg):
        """ROS 订阅回调: 每当收到 JudgementReport 消息时被调用。

        只做一件事: 线程安全地缓存最新消息。
        发送由定时器驱动, 不与接收同步。
        """
        with self._lock:  
            self._latest = msg

    # ========== 数据编码 ==========

    def _to_payload(self, msg):
        """将 JudgementReport(ROS消息) 编码为裁判软件期望的 JSON 字节流。

        裁判软件 TCP 协议字段:
            id     - 小车编号
            speed  - 当前速度
            odom   - [x, y] 位置坐标(只取前两项)
            task   - 当前任务
            CV1    - 自定义字段 1(视觉/状态)
            CV2    - 自定义字段 2(视觉/状态)

        返回:
            bytes: 编码后的 JSON 字节流, 如 b'{"id":1,"speed":0.5,...}\n'
            None:  数据不完整(如 odom 数组长度不足)时返回
        """
        # 安全检查: 位置信息至少要有 x, y 两个值
        if len(msg.odom) < 2:
            rospy.logwarn_throttle(5.0, "odom 至少需要 2 个元素 [x, y]")
            return None

        # 组装裁判软件需要的字段
        data = {
            "id": msg.id,
            "speed": msg.speed,
            "odom": [msg.odom[0], msg.odom[1]],
            "task": msg.task,
            "CV1": msg.CV1,
            "CV2": msg.CV2,
        }

        # 紧凑序列化: separators=(",",":") 去掉冒号和逗号后的空格,
        # ensure_ascii=False 保留非 ASCII 字符不转义
        text = json.dumps(data, separators=(",", ":"), ensure_ascii=False)

        # 裁判软件 TCP 协议可能需要行尾标志来分隔每条消息
        if self.append_newline:
            text += "\n"

        return text.encode("utf-8")

    # ========== TCP 连接管理 ==========

    def _connect(self):
        """建立到裁判软件的 TCP 连接(支持自动重连)。

        如果当前已连接(self._sock 非 None), 直接返回 True。
        否则创建新 socket, 连接超时设为 3 秒。
        成功连接后设为阻塞模式(sendall 不需要超时, 一直等到发完为止)。

        返回:
            True:  已连接或连接成功
            False: 连接失败
        """
        if self._sock is not None:
            return True   # 已连接, 无需反复建连

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket  地址和tcp协议
            sock.settimeout(3.0)                   # 连接阶段超时 3 秒, 防止卡死
            sock.connect((self.server_ip, self.server_port))
            sock.settimeout(None)                  # 恢复阻塞模式, sendall 不限时
            self._sock = sock
            rospy.loginfo("已连接裁判软件 %s:%d", self.server_ip, self.server_port)
            return True
        except socket.error as exc:
            rospy.logwarn_throttle(5.0, "连接裁判软件失败: %s", exc)
            self._close_socket()
            return False

    def _close_socket(self):
        """关闭 TCP socket 并置为 None, 下次 _connect 会触发自动重连。"""
        if self._sock is not None:
            try:
                self._sock.close()
            except socket.error:
                pass    # 已经在关闭过程中, 忽略重复关闭的错误
            self._sock = None

    # ========== 定时发送 ==========

    def _send_timer_cb(self, _event):
        """定时器回调: 每隔 1/send_rate 秒被调用一次。

        流程:
            1. 线程安全地取出缓存的 _latest 消息
            2. 转换成 JSON 字节流 (_to_payload)
            3. 确保 TCP 连接可用 (_connect, 断开会自动重连)
            4. 调用 socket.sendall() 发送全部数据

        任何步骤失败则静默跳过本次发送, 等待下一轮定时器触发。
        """
        # 1. 加锁取出最新消息(和 _report_cb 共用 _latest, 必须同步)
        with self._lock:
            if self._latest is None:
                return   # 还没收到过任何消息, 什么也不发
            msg = self._latest

        # 2. 编码为 JSON 字节流
        payload = self._to_payload(msg)
        if payload is None:
            return      # 数据不完整(如 odom 缺少字段)

        # 3. 确保 TCP 连接可用(自动重连)
        if not self._connect():
            return      # 连接失败(裁判软件可能没启动), 等待下一轮

        # 4. 发送全部数据
        try:
            self._sock.sendall(payload)
        except socket.error as exc:
            rospy.logwarn("发送失败，将重连: %s", exc)
            self._close_socket()   # 断开连接, 下一轮定时器会触发自动重连

    # ========== 节点关闭 ==========

    def shutdown(self):
        """ROS 节点退出时的清理函数。
        停止定时器 + 关闭 TCP socket, 避免资源泄漏。
        """
        self._timer.shutdown()
        self._close_socket()


if __name__ == "__main__":
    # 实例化节点
    node = JudgementTcpSender()
    # 注册退出钩子: Ctrl+C 或 rosnode kill 时会调用 node.shutdown()
    rospy.on_shutdown(node.shutdown)
    # 进入 ROS 事件循环, 等待回调触发
    rospy.spin()
