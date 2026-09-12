#!/usr/bin/env python3

import time

import rclpy

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import GetModelList
from gazebo_msgs.srv import SpawnEntity

from geometry_msgs.msg import Pose
from rclpy.node import Node


class SceneSetupNode(Node):

    def __init__(self):

        super().__init__('erc_scene_setup_node')

        # ==========================================================
        # ENTITY NAMES
        # ==========================================================

        self.support_name = 'erc_phase8_book_support'
        self.book_name = 'erc_phase8_blue_book'

        # ==========================================================
        # PLACEMENT
        # ==========================================================

        self.reference_frame = None
        self.robot_model_name = None

        # Spawn close enough to the robot after scene reset.
        self.book_x = 0.52
        self.book_y = 0.20
        self.book_z = 0.485

        self.support_x = 0.57
        self.support_y = 0.20
        self.support_z = 0.235

        # ==========================================================
        # SERVICES
        # ==========================================================

        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity',
        )

        self.delete_client = self.create_client(
            DeleteEntity,
            '/delete_entity',
        )

        self.model_list_client = self.create_client(
            GetModelList,
            '/get_model_list',
        )

        self.get_logger().info(
            '================================================'
        )
        self.get_logger().info(
            'ERC PHASE 8 COMPETITION SCENE'
        )
        self.get_logger().info(
            'Gripper-friendly BLUE book: ENABLED'
        )
        self.get_logger().info(
            'High-friction book contact: ENABLED'
        )
        self.get_logger().info(
            'Automatic placement: ENABLED'
        )
        self.get_logger().info(
            '================================================'
        )

    # ==============================================================
    # WAIT
    # ==============================================================

    def wait_for_services(self):

        services = [
            (self.spawn_client, '/spawn_entity'),
            (self.delete_client, '/delete_entity'),
            (self.model_list_client, '/get_model_list'),
        ]

        for client, name in services:

            self.get_logger().info(
                f'Waiting for {name}...'
            )

            while rclpy.ok():

                if client.wait_for_service(
                    timeout_sec=1.0
                ):
                    break

            self.get_logger().info(
                f'{name} available.'
            )

    # ==============================================================
    # MODEL LIST
    # ==============================================================

    def get_model_names(self):

        request = GetModelList.Request()

        future = self.model_list_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0,
        )

        if not future.done():

            self.get_logger().error(
                '/get_model_list timed out.'
            )

            return []

        response = future.result()

        if response is None:
            return []

        return list(
            response.model_names
        )

    # ==============================================================
    # DISCOVER TIAGO
    # ==============================================================

    def discover_robot(self):

        names = self.get_model_names()

        self.get_logger().info(
            'Gazebo models: ' + ', '.join(names)
        )

        tiago_models = [
            name
            for name in names
            if 'tiago' in name.lower()
        ]

        if not tiago_models:

            self.get_logger().error(
                'TIAGo model not found.'
            )

            return False

        self.robot_model_name = tiago_models[0]

        # PAL Gazebo accepted base_footprint previously.
        self.reference_frame = (
            f'{self.robot_model_name}::base_footprint'
        )

        self.get_logger().info(
            (
                'Robot model: '
                f'{self.robot_model_name}'
            )
        )

        self.get_logger().info(
            (
                'Reference frame: '
                f'{self.reference_frame}'
            )
        )

        return True

    # ==============================================================
    # DELETE
    # ==============================================================

    def delete_entity(self, name):

        request = DeleteEntity.Request()

        request.name = name

        future = self.delete_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0,
        )

        if future.done():

            response = future.result()

            if (
                response is not None
                and
                response.success
            ):

                self.get_logger().info(
                    f'Deleted: {name}'
                )

    # ==============================================================
    # SUPPORT
    # ==============================================================

    def support_sdf(self):

        return """
<sdf version='1.6'>

  <model name='erc_phase8_book_support'>

    <static>true</static>

    <link name='support_link'>

      <collision name='support_collision'>

        <geometry>
          <box>
            <size>0.28 0.30 0.30</size>
          </box>
        </geometry>

        <surface>

          <friction>
            <ode>
              <mu>3.0</mu>
              <mu2>3.0</mu2>
            </ode>
          </friction>

        </surface>

      </collision>

      <visual name='support_visual'>

        <geometry>
          <box>
            <size>0.28 0.30 0.30</size>
          </box>
        </geometry>

        <material>
          <ambient>0.35 0.35 0.35 1</ambient>
          <diffuse>0.45 0.45 0.45 1</diffuse>
        </material>

      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # BOOK
    # ==============================================================

    def book_sdf(self):

        # ----------------------------------------------------------
        # COMPETITION-GRASPABLE BOOK
        #
        # OLD:
        #   0.045 x 0.160 x 0.250
        #
        # NEW:
        #   0.040 x 0.065 x 0.200
        #
        # The Y pinch dimension is now only 6.5 cm.
        # ----------------------------------------------------------

        return """
<sdf version='1.6'>

  <model name='erc_phase8_blue_book'>

    <static>false</static>

    <link name='book_link'>

      <gravity>true</gravity>

      <inertial>

        <mass>0.08</mass>

        <inertia>
          <ixx>0.00030</ixx>
          <iyy>0.00030</iyy>
          <izz>0.00010</izz>

          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>

      </inertial>

      <collision name='book_collision'>

        <geometry>
          <box>
            <size>0.040 0.065 0.200</size>
          </box>
        </geometry>

        <surface>

          <friction>

            <ode>
              <mu>5.0</mu>
              <mu2>5.0</mu2>
              <fdir1>0 1 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>

          </friction>

          <contact>

            <ode>
              <kp>1000000.0</kp>
              <kd>10.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>

          </contact>

          <bounce>
            <restitution_coefficient>
              0.0
            </restitution_coefficient>
            <threshold>1000000.0</threshold>
          </bounce>

        </surface>

      </collision>

      <visual name='book_visual'>

        <geometry>
          <box>
            <size>0.040 0.065 0.200</size>
          </box>
        </geometry>

        <material>
          <ambient>0.0 0.0 1.0 1.0</ambient>
          <diffuse>0.0 0.0 1.0 1.0</diffuse>
          <specular>0.05 0.05 0.05 1.0</specular>
        </material>

      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # SPAWN
    # ==============================================================

    def spawn(
        self,
        name,
        xml,
        x,
        y,
        z,
    ):

        request = SpawnEntity.Request()

        request.name = name
        request.xml = xml

        request.robot_namespace = ''

        request.reference_frame = (
            self.reference_frame
        )

        request.initial_pose = Pose()

        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)

        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0

        self.get_logger().info(
            (
                f'Spawning {name}: '
                f'X={x:.3f}, '
                f'Y={y:.3f}, '
                f'Z={z:.3f}'
            )
        )

        future = self.spawn_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0,
        )

        if not future.done():

            self.get_logger().error(
                f'Spawn timed out: {name}'
            )

            return False

        response = future.result()

        if response is None:

            self.get_logger().error(
                f'No spawn response: {name}'
            )

            return False

        if not response.success:

            self.get_logger().error(
                (
                    f'Spawn failed: {name}: '
                    f'{response.status_message}'
                )
            )

            return False

        self.get_logger().info(
            f'SPAWN SUCCESS: {name}'
        )

        return True

    # ==============================================================
    # BUILD
    # ==============================================================

    def build_scene(self):

        self.wait_for_services()

        if not self.discover_robot():
            return False

        self.delete_entity(
            self.book_name
        )

        self.delete_entity(
            self.support_name
        )

        time.sleep(0.5)

        if not self.spawn(
            self.support_name,
            self.support_sdf(),
            self.support_x,
            self.support_y,
            self.support_z,
        ):

            return False

        time.sleep(0.5)

        if not self.spawn(
            self.book_name,
            self.book_sdf(),
            self.book_x,
            self.book_y,
            self.book_z,
        ):

            return False

        time.sleep(1.5)

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'PHASE 8 COMPETITION SCENE READY'
        )

        self.get_logger().info(
            'BLUE book pinch width = 0.065 m'
        )

        self.get_logger().info(
            'BLUE book mass = 0.08 kg'
        )

        self.get_logger().info(
            'High-friction contact enabled.'
        )

        self.get_logger().info(
            '================================================'
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = SceneSetupNode()

    try:

        node.build_scene()

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
