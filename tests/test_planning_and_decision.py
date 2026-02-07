from config.constants import HANDOVER_OFFSET_Z, LIFT_HEIGHT, PREGRASP_OFFSET_Z
from decision.wma_task_manager import WMATaskManager
from manipulation.grasp_planner import compute_lift_pose, compute_pregrasp, validate_grasp_pose
from manipulation.handover_planner import compute_handover_pose


def make_pose(x=0.5, y=0.0, z=0.3):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose


def test_pregrasp_does_not_mutate_input():
    object_pose = make_pose(z=0.3)
    original_z = object_pose.pose.position.z

    pregrasp = compute_pregrasp(object_pose)

    assert pregrasp is not object_pose
    assert object_pose.pose.position.z == original_z
    assert pregrasp.pose.position.z == original_z + PREGRASP_OFFSET_Z
    assert pregrasp.pose is not object_pose.pose


def test_lift_pose_does_not_mutate_input():
    grasp_pose = make_pose(z=0.25)
    original_z = grasp_pose.pose.position.z

    lifted = compute_lift_pose(grasp_pose)

    assert lifted.pose.position.z == original_z + LIFT_HEIGHT
    assert grasp_pose.pose.position.z == original_z
    assert lifted.pose is not grasp_pose.pose


def test_handover_pose_uses_offset_and_preserves_source():
    human_pose = make_pose(z=1.0)
    result = compute_handover_pose(human_pose)

    assert result.pose.position.z == human_pose.pose.position.z + HANDOVER_OFFSET_Z
    assert human_pose.pose.position.z == 1.0
    assert result.pose is not human_pose.pose


def test_validate_grasp_pose_bounds():
    assert validate_grasp_pose(make_pose(x=0.5, y=0.0, z=0.5)) is True
    assert validate_grasp_pose(make_pose(x=1.2, y=0.0, z=0.5)) is False
    assert validate_grasp_pose(make_pose(x=0.5, y=0.6, z=0.5)) is False
    assert validate_grasp_pose(make_pose(x=0.5, y=0.0, z=1.6)) is False


def test_wma_task_manager_transitions_in_sequence():
    manager = WMATaskManager()

    assert manager.state == "idle"
    assert manager.update(object_pose=None) == "approach"
    assert manager.update(object_pose=None) == "grasp"
    assert manager.update(object_pose=None) == "lift"
    assert manager.update(object_pose=None) == "lift"
