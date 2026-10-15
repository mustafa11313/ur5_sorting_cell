"""Add the static cell (pedestal, conveyor, bins, camera mast) to MoveIt's planning scene."""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .cell import load_cell


def box(name, center, size, frame='world'):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.w = 1.0
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


def cell_objects(cell):
    objs = []
    ped = cell['pedestal']
    # 1 cm lower than the real pedestal so the robot base does not "collide" with it
    objs.append(box('pedestal', [ped['center'][0], ped['center'][1], ped['center'][2] - 0.005],
                    [ped['size'][0], ped['size'][1], ped['size'][2] - 0.01]))
    belt = cell['belt']
    length = belt['max_x'] - belt['min_x']
    objs.append(box('conveyor', [(belt['max_x'] + belt['min_x']) / 2, belt['center_y'],
                                 (belt['surface_z'] - 0.05) / 2],
                    [length, belt['width'], belt['surface_z'] - 0.05]))
    objs.append(box('conveyor_belt', [(belt['max_x'] + belt['min_x']) / 2, belt['center_y'],
                                      belt['surface_z'] - 0.025],
                    [length, belt['width'], 0.05]))
    for side in (-1, 1):  # side rails, 2 cm above the belt surface
        objs.append(box(f'rail_{side + 1}', [(belt['max_x'] + belt['min_x']) / 2,
                                             belt['center_y'] + side * (belt['width'] / 2 + 0.015),
                                             belt['surface_z'] - 0.015],
                        [length + 0.1, 0.03, 0.07]))
    rack = cell['bin_rack']
    objs.append(box('bin_rack', rack['center'], rack['size']))
    inner, wall, height = 0.18, 0.01, 0.12
    for name, (x, y, z) in cell['bins'].items():
        half = inner / 2 + wall / 2
        objs.append(box(f'{name}_n', [x, y + half, z + height / 2], [inner + 2 * wall, wall, height]))
        objs.append(box(f'{name}_s', [x, y - half, z + height / 2], [inner + 2 * wall, wall, height]))
        objs.append(box(f'{name}_e', [x + half, y, z + height / 2], [wall, inner, height]))
        objs.append(box(f'{name}_w', [x - half, y, z + height / 2], [wall, inner, height]))
    for key, part in cell['camera_mast'].items():
        objs.append(box(f'camera_{key}', part['center'], part['size']))
    for m in cell.get('markers', []):
        objs.append(box(f'aruco_{m["id"]}', [m['x'], m['y'], m['top_z'] - 0.03], [0.1, 0.1, 0.06]))
    objs.append(box('floor', [0, 0, -0.01], [4.0, 4.0, 0.02]))
    return objs


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__('planning_scene_loader')
        self.declare_parameter('cell_config', '')
        path = self.get_parameter('cell_config').value or None
        self.objects = cell_objects(load_cell(path))
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def apply(self):
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('waiting for move_group /apply_planning_scene ...')
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = self.objects
        future = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        ok = future.result() is not None and future.result().success
        self.get_logger().info(
            f'added {len(self.objects)} collision objects' if ok else 'failed to apply scene')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneLoader()
    try:
        node.apply()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
