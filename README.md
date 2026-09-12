# ERC 2026 TIAGo Pro Autonomous Book Retrieval

ROS 2 Humble / Gazebo project for autonomous book detection, approach, manipulation, and delivery using TIAGo Pro.

## Current Status

Working: competition arena, BLUE-book perception, 3D localization, Nav2, TF tracking, final base approach, MoveIt arm control, and gripper control.

In progress: reliable physical grasp of the simulated book.

Remaining: verify grasp success, carry the book to the collection bin, release it, and validate the complete end-to-end mission.

## Main ROS 2 Packages

- erc_bringup
- erc_competition_world
- erc_navigation
- erc_perception
- erc_manipulation
- erc_mission
- erc_scene_setup

## Important Tools

- depth_tests/book_3d_locator.py
- tools/final_book_approach.py

## Status

Work in progress. Physical grasp reliability is the current development milestone.
