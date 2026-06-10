"""ROS2 integration (PLAN Part 3).

Everything here is optional: if ``rclpy`` cannot be imported (e.g. a Windows
dev box) the bridge reports ``available = False`` and the HTTP/WS routes return
a clear 503 instead of crashing the server.
"""
