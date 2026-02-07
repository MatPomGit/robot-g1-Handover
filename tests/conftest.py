import sys
import types


def _install_geometry_msgs_stub():
    """Install minimal geometry_msgs stubs so planners can be imported without ROS."""
    if "geometry_msgs.msg" in sys.modules:
        return

    geometry_msgs = types.ModuleType("geometry_msgs")
    msg = types.ModuleType("geometry_msgs.msg")

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self, position=None, orientation=None):
            self.position = position or Point()
            self.orientation = orientation or Quaternion()

    class PoseStamped:
        def __init__(self, header=None, pose=None):
            self.header = header
            self.pose = pose or Pose()

    msg.Point = Point
    msg.Quaternion = Quaternion
    msg.Pose = Pose
    msg.PoseStamped = PoseStamped
    geometry_msgs.msg = msg

    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = msg


_install_geometry_msgs_stub()
