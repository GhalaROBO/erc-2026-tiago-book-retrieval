#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity

from geometry_msgs.msg import Pose


class ERCCompetitionArena(Node):

    def __init__(self):

        super().__init__('erc_competition_arena')

        # ==========================================================
        # FIXED WORLD POSITIONS
        # ==========================================================
        #
        # Robot HOME is approximately around the Gazebo origin.
        #
        # Shelf is placed forward from HOME.
        # Bin is placed to the robot's right-hand side of the arena.
        #
        # These are WORLD coordinates. They do NOT follow TIAGo.
        # ==========================================================

        self.shelf_x = 1.45
        self.shelf_y = 0.00
        self.shelf_z = 0.00

        # Books stand at front edge of shelf.
        self.book_x = 1.24

        self.book_positions = {
            'BLUE': {
                'x': self.book_x,
                'y': 0.24,
                'z': 0.505,
            },

            'RED': {
                'x': self.book_x,
                'y': 0.08,
                'z': 0.505,
            },

            'GREEN': {
                'x': self.book_x,
                'y': -0.08,
                'z': 0.505,
            },

            'YELLOW': {
                'x': self.book_x,
                'y': -0.24,
                'z': 0.505,
            },
        }

        self.bin_x = 0.35
        self.bin_y = -1.05
        self.bin_z = 0.00

        self.home_x = 0.20
        self.home_y = 0.10
        self.home_z = 0.005

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

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC 2026 COMPETITION ARENA'
        )

        self.get_logger().info(
            'Fixed shelf: ENABLED'
        )

        self.get_logger().info(
            'Physical multi-book shelf: ENABLED'
        )

        self.get_logger().info(
            'Physical collection bin: ENABLED'
        )

        self.get_logger().info(
            'World-frame placement: ENABLED'
        )

        self.get_logger().info(
            'Hidden grasp attachment: DISABLED'
        )

        self.get_logger().info(
            '================================================'
        )

    # ==============================================================
    # WAIT FOR GAZEBO
    # ==============================================================

    def wait_for_gazebo(self):

        self.get_logger().info(
            'Waiting for Gazebo services...'
        )

        while rclpy.ok():

            spawn_ready = (
                self.spawn_client.wait_for_service(
                    timeout_sec=1.0
                )
            )

            delete_ready = (
                self.delete_client.wait_for_service(
                    timeout_sec=1.0
                )
            )

            if spawn_ready and delete_ready:
                break

            self.get_logger().info(
                'Gazebo not ready yet...'
            )

        self.get_logger().info(
            'Gazebo spawn/delete services ready.'
        )

    # ==============================================================
    # DELETE
    # ==============================================================

    def delete_entity(
        self,
        name,
    ):

        request = DeleteEntity.Request()

        request.name = name

        future = self.delete_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=4.0,
        )

        if not future.done():
            return

        try:

            response = future.result()

        except Exception:
            return

        if (
            response is not None
            and
            response.success
        ):

            self.get_logger().info(
                f'Deleted old entity: {name}'
            )

    # ==============================================================
    # SPAWN
    # ==============================================================

    def spawn_entity(
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

        request.reference_frame = 'world'

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

        try:

            response = future.result()

        except Exception as exc:

            self.get_logger().error(
                (
                    f'Spawn exception for '
                    f'{name}: {exc}'
                )
            )

            return False

        if (
            response is None
            or
            not response.success
        ):

            message = (
                response.status_message
                if response is not None
                else 'No response'
            )

            self.get_logger().error(
                (
                    f'Failed to spawn {name}: '
                    f'{message}'
                )
            )

            return False

        self.get_logger().info(
            f'SPAWN SUCCESS: {name}'
        )

        return True

    # ==============================================================
    # SHELF MODEL
    # ==============================================================

    def shelf_sdf(self):

        return """
<sdf version='1.6'>

  <model name='erc_competition_shelf'>

    <static>true</static>

    <link name='shelf_link'>

      <!-- BASE -->
      <collision name='base_collision'>
        <pose>0 0 0.025 0 0 0</pose>
        <geometry>
          <box>
            <size>0.40 1.10 0.05</size>
          </box>
        </geometry>
      </collision>

      <visual name='base_visual'>
        <pose>0 0 0.025 0 0 0</pose>
        <geometry>
          <box>
            <size>0.40 1.10 0.05</size>
          </box>
        </geometry>

        <material>
          <ambient>0.30 0.30 0.30 1</ambient>
          <diffuse>0.42 0.42 0.42 1</diffuse>
        </material>
      </visual>

      <!-- MAIN SHELF SURFACE -->
      <collision name='surface_collision'>
        <pose>0 0 0.375 0 0 0</pose>
        <geometry>
          <box>
            <size>0.40 1.10 0.05</size>
          </box>
        </geometry>

        <surface>
          <friction>
            <ode>
              <mu>2.5</mu>
              <mu2>2.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name='surface_visual'>
        <pose>0 0 0.375 0 0 0</pose>
        <geometry>
          <box>
            <size>0.40 1.10 0.05</size>
          </box>
        </geometry>

        <material>
          <ambient>0.45 0.28 0.12 1</ambient>
          <diffuse>0.55 0.35 0.15 1</diffuse>
        </material>
      </visual>

      <!-- LEFT SUPPORT -->
      <collision name='left_support_collision'>
        <pose>0 0.50 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.10 0.10 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='left_support_visual'>
        <pose>0 0.50 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.10 0.10 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.30 0.30 0.30 1</ambient>
          <diffuse>0.40 0.40 0.40 1</diffuse>
        </material>
      </visual>

      <!-- RIGHT SUPPORT -->
      <collision name='right_support_collision'>
        <pose>0 -0.50 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.10 0.10 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='right_support_visual'>
        <pose>0 -0.50 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.10 0.10 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.30 0.30 0.30 1</ambient>
          <diffuse>0.40 0.40 0.40 1</diffuse>
        </material>
      </visual>

      <!-- BACKBOARD -->
      <collision name='back_collision'>
        <pose>0.17 0 0.70 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 1.10 0.65</size>
          </box>
        </geometry>
      </collision>

      <visual name='back_visual'>
        <pose>0.17 0 0.70 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 1.10 0.65</size>
          </box>
        </geometry>

        <material>
          <ambient>0.34 0.34 0.34 1</ambient>
          <diffuse>0.45 0.45 0.45 1</diffuse>
        </material>
      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # BOOK MODEL
    # ==============================================================

    def book_sdf(
        self,
        model_name,
        r,
        g,
        b,
    ):

        return f"""
<sdf version='1.6'>

  <model name='{model_name}'>

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
              <mu>3.5</mu>
              <mu2>3.5</mu2>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>

          <bounce>
            <restitution_coefficient>
              0.0
            </restitution_coefficient>
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
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>

      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # BIN
    # ==============================================================

    def bin_sdf(self):

        return """
<sdf version='1.6'>

  <model name='erc_competition_bin'>

    <static>true</static>

    <link name='bin_link'>

      <!-- FLOOR -->

      <collision name='floor_collision'>
        <pose>0 0 0.03 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.55 0.06</size>
          </box>
        </geometry>
      </collision>

      <visual name='floor_visual'>
        <pose>0 0 0.03 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.55 0.06</size>
          </box>
        </geometry>

        <material>
          <ambient>0.12 0.12 0.12 1</ambient>
          <diffuse>0.18 0.18 0.18 1</diffuse>
        </material>
      </visual>

      <!-- FRONT WALL -->

      <collision name='front_collision'>
        <pose>-0.25 0 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 0.55 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='front_visual'>
        <pose>-0.25 0 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 0.55 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.12 0.12 0.12 1</ambient>
          <diffuse>0.18 0.18 0.18 1</diffuse>
        </material>
      </visual>

      <!-- BACK WALL -->

      <collision name='back_collision'>
        <pose>0.25 0 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 0.55 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='back_visual'>
        <pose>0.25 0 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.05 0.55 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.12 0.12 0.12 1</ambient>
          <diffuse>0.18 0.18 0.18 1</diffuse>
        </material>
      </visual>

      <!-- LEFT WALL -->

      <collision name='left_collision'>
        <pose>0 0.25 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.05 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='left_visual'>
        <pose>0 0.25 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.05 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.12 0.12 0.12 1</ambient>
          <diffuse>0.18 0.18 0.18 1</diffuse>
        </material>
      </visual>

      <!-- RIGHT WALL -->

      <collision name='right_collision'>
        <pose>0 -0.25 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.05 0.40</size>
          </box>
        </geometry>
      </collision>

      <visual name='right_visual'>
        <pose>0 -0.25 0.20 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.05 0.40</size>
          </box>
        </geometry>

        <material>
          <ambient>0.12 0.12 0.12 1</ambient>
          <diffuse>0.18 0.18 0.18 1</diffuse>
        </material>
      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # HOME MARKER
    # ==============================================================

    def home_marker_sdf(self):

        return """
<sdf version='1.6'>

  <model name='erc_home_marker'>

    <static>true</static>

    <link name='marker_link'>

      <visual name='marker_visual'>

        <geometry>
          <cylinder>
            <radius>0.32</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>

        <material>
          <ambient>0.10 0.70 0.10 0.55</ambient>
          <diffuse>0.10 0.70 0.10 0.55</diffuse>
        </material>

      </visual>

    </link>

  </model>

</sdf>
"""

    # ==============================================================
    # BUILD ARENA
    # ==============================================================

    def build_arena(self):

        self.wait_for_gazebo()

        entities = [
            'erc_competition_shelf',
            'erc_book_blue',
            'erc_book_red',
            'erc_book_green',
            'erc_book_yellow',
            'erc_competition_bin',
            'erc_home_marker',
        ]

        self.get_logger().info(
            'Removing previous competition arena...'
        )

        for entity in entities:

            self.delete_entity(
                entity
            )

        time.sleep(0.5)

        # ----------------------------------------------------------
        # SHELF
        # ----------------------------------------------------------

        if not self.spawn_entity(
            'erc_competition_shelf',
            self.shelf_sdf(),
            self.shelf_x,
            self.shelf_y,
            self.shelf_z,
        ):

            return False

        time.sleep(0.5)

        # ----------------------------------------------------------
        # BOOKS
        # ----------------------------------------------------------

        colors = {
            'BLUE': (
                'erc_book_blue',
                0.0,
                0.0,
                1.0,
            ),

            'RED': (
                'erc_book_red',
                1.0,
                0.0,
                0.0,
            ),

            'GREEN': (
                'erc_book_green',
                0.0,
                0.75,
                0.0,
            ),

            'YELLOW': (
                'erc_book_yellow',
                1.0,
                0.85,
                0.0,
            ),
        }

        for color, values in colors.items():

            model_name, r, g, b = values

            position = (
                self.book_positions[color]
            )

            success = self.spawn_entity(
                model_name,
                self.book_sdf(
                    model_name,
                    r,
                    g,
                    b,
                ),
                position['x'],
                position['y'],
                position['z'],
            )

            if not success:
                return False

            time.sleep(0.25)

        # ----------------------------------------------------------
        # BIN
        # ----------------------------------------------------------

        if not self.spawn_entity(
            'erc_competition_bin',
            self.bin_sdf(),
            self.bin_x,
            self.bin_y,
            self.bin_z,
        ):

            return False

        # ----------------------------------------------------------
        # HOME
        # ----------------------------------------------------------

        if not self.spawn_entity(
            'erc_home_marker',
            self.home_marker_sdf(),
            self.home_x,
            self.home_y,
            self.home_z,
        ):

            return False

        # Let physics settle.

        time.sleep(2.0)

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'ERC COMPETITION ARENA READY'
        )

        self.get_logger().info(
            (
                'SHELF world position: '
                f'X={self.shelf_x:.2f}, '
                f'Y={self.shelf_y:.2f}'
            )
        )

        self.get_logger().info(
            (
                'BIN world position: '
                f'X={self.bin_x:.2f}, '
                f'Y={self.bin_y:.2f}'
            )
        )

        self.get_logger().info(
            (
                'HOME world position: '
                f'X={self.home_x:.2f}, '
                f'Y={self.home_y:.2f}'
            )
        )

        self.get_logger().info(
            'Books: BLUE RED GREEN YELLOW'
        )

        self.get_logger().info(
            'Books use real Gazebo physics.'
        )

        self.get_logger().info(
            'No perfect-grasp attachment is active.'
        )

        self.get_logger().info(
            '================================================'
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = ERCCompetitionArena()

    success = False

    try:

        success = node.build_arena()

    except KeyboardInterrupt:

        pass

    except Exception as exc:

        node.get_logger().error(
            f'Arena setup exception: {exc}'
        )

    finally:

        if success:

            node.get_logger().info(
                'Competition arena setup completed.'
            )

        else:

            node.get_logger().error(
                'Competition arena setup failed.'
            )

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()
