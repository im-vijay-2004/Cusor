# Copyright 2025 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

import math
import time
import numpy as np
import cv2
from typing import Optional, Tuple
import asyncio
import threading
import random

from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from synapse_msgs.msg import Status
from synapse_msgs.msg import WarehouseShelf

from scipy.ndimage import label, center_of_mass
from scipy.spatial.distance import euclidean
from sklearn.decomposition import PCA

import tkinter as tk
from tkinter import ttk

# Removed pyzbar import to avoid external dependency on libzbar.

QOS_PROFILE_DEFAULT = 10
SERVER_WAIT_TIMEOUT_SEC = 5.0

PROGRESS_TABLE_GUI = True


class WindowProgressTable:
	def __init__(self, root, shelf_count):
		self.root = root
		self.root.title("Warehouse Shelf Objects & QR Link")
		self.root.attributes("-topmost", True)

		# Dynamic layout to accommodate up to 8 objects per shelf
		self.max_objects_per_shelf = 8
		self.row_count = self.max_objects_per_shelf + 1  # Objects + 1 row for QR
		self.col_count = shelf_count

		self.boxes = []
		
		# Create header labels
		for col in range(self.col_count):
			header = tk.Label(root, text=f"Shelf {col+1}", font=("Helvetica", 12, "bold"), 
							 bg="lightgray", relief="solid", borderwidth=1)
			header.grid(row=0, column=col, padx=2, pady=2, sticky="nsew")
		
		# Create object boxes (rows 1-8 for objects)
		for row in range(1, self.max_objects_per_shelf + 1):
			row_boxes = []
			for col in range(self.col_count):
				box = tk.Text(root, width=12, height=2, wrap=tk.WORD, borderwidth=1,
					      relief="solid", font=("Helvetica", 10))
				box.insert(tk.END, f"Object {row}")
				box.config(state=tk.DISABLED)  # Read-only initially
				box.grid(row=row, column=col, padx=2, pady=1, sticky="nsew")
				row_boxes.append(box)
			self.boxes.append(row_boxes)
		
		# Create QR row (last row)
		self.qr_boxes = []
		for col in range(self.col_count):
			qr_box = tk.Text(root, width=12, height=2, wrap=tk.WORD, borderwidth=1,
						    relief="solid", font=("Helvetica", 10, "bold"))
			qr_box.insert(tk.END, "QR: Waiting...")
			qr_box.config(state=tk.DISABLED, bg="lightyellow")
			qr_box.grid(row=self.max_objects_per_shelf + 1, column=col, padx=2, pady=2, sticky="nsew")
			self.qr_boxes.append(qr_box)
		
		# Track which shelves have been completed
		self.shelf_status = ["empty"] * self.col_count  # "empty", "objects", "complete"
		self.current_shelf = 0

		# Make the grid layout responsive
		for row in range(self.max_objects_per_shelf + 2):  # +2 for header and QR
			self.root.grid_rowconfigure(row, weight=1)
		for col in range(self.col_count):
			self.root.grid_columnconfigure(col, weight=1)

	def change_box_color(self, row, col, color):
		if row < len(self.boxes):
			self.boxes[row][col].config(bg=color)

	def change_box_text(self, row, col, text):
		if row < len(self.boxes):
			self.boxes[row][col].config(state=tk.NORMAL)
			self.boxes[row][col].delete(1.0, tk.END)
			self.boxes[row][col].insert(tk.END, text)
			self.boxes[row][col].config(state=tk.DISABLED)
	
	def update_shelf_objects(self, shelf_col, object_list):
		"""Update all objects for a specific shelf column."""
		# Ensure we don't exceed available columns
		if shelf_col >= self.col_count:
			print(f"Warning: Trying to update shelf {shelf_col}, but only {self.col_count} columns available")
			return
			
		# Clear all object boxes for this shelf
		for row in range(len(self.boxes)):
			self.change_box_text(row, shelf_col, "---")
			self.change_box_color(row, shelf_col, "white")
		
		# Fill with detected objects
		for idx, (name, count) in enumerate(object_list):
			if idx < len(self.boxes):
				self.change_box_text(idx, shelf_col, f"{name}: {count}")
				self.change_box_color(idx, shelf_col, "lightgreen")
		
		# Update shelf status
		self.shelf_status[shelf_col] = "objects"
		print(f"Updated shelf {shelf_col} with {len(object_list)} objects")
	
	def update_shelf_qr(self, shelf_col, qr_text):
		"""Update QR code for a specific shelf."""
		if shelf_col >= len(self.qr_boxes):
			print(f"Warning: Trying to update QR for shelf {shelf_col}, but only {len(self.qr_boxes)} QR boxes available")
			return
			
		self.qr_boxes[shelf_col].config(state=tk.NORMAL)
		self.qr_boxes[shelf_col].delete(1.0, tk.END)
		self.qr_boxes[shelf_col].insert(tk.END, f"QR: {qr_text}")
		self.qr_boxes[shelf_col].config(state=tk.DISABLED, bg="lightblue")
		
		# Update shelf status and move to next shelf
		self.shelf_status[shelf_col] = "complete"
		print(f"Completed shelf {shelf_col} with QR: {qr_text}")
		
		# Find next available shelf
		self.current_shelf = self._find_next_available_shelf()
		print(f"Next available shelf: {self.current_shelf}")
	
	def _find_next_available_shelf(self):
		"""Find the next available shelf column."""
		for i in range(self.col_count):
			if self.shelf_status[i] == "empty":
				return i
		return 0  # Wrap around if all shelves used
	
	def get_current_shelf_column(self):
		"""Get the current shelf column for new detections."""
		return self.current_shelf

box_app = None
def run_gui(shelf_count):
	global box_app
	root = tk.Tk()
	box_app = WindowProgressTable(root, shelf_count)
	root.mainloop()


class WarehouseExplore(Node):
	""" Initializes warehouse explorer node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('warehouse_explore')

		self.action_client = ActionClient(
			self,
			NavigateToPose,
			'/navigate_to_pose')

		self.subscription_pose = self.create_subscription(
			PoseWithCovarianceStamped,
			'/pose',
			self.pose_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_global_map = self.create_subscription(
			OccupancyGrid,
			'/global_costmap/costmap',
			self.global_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_simple_map = self.create_subscription(
			OccupancyGrid,
			'/map',
			self.simple_map_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_status = self.create_subscription(
			Status,
			'/cerebri/out/status',
			self.cerebri_status_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_behavior = self.create_subscription(
			BehaviorTreeLog,
			'/behavior_tree_log',
			self.behavior_tree_log_callback,
			QOS_PROFILE_DEFAULT)

		self.subscription_shelf_objects = self.create_subscription(
			WarehouseShelf,
			'/shelf_objects',
			self.shelf_objects_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Publisher for output image (for debug purposes).
		self.publisher_qr_decode = self.create_publisher(
			CompressedImage,
			"/debug_images/qr_code",
			QOS_PROFILE_DEFAULT)

		self.publisher_shelf_data = self.create_publisher(
			WarehouseShelf,
			"/shelf_data",
			QOS_PROFILE_DEFAULT)

		self.declare_parameter('shelf_count', 1)
		self.declare_parameter('initial_angle', 0.0)

		self.shelf_count = \
			self.get_parameter('shelf_count').get_parameter_value().integer_value
		self.initial_angle = \
			self.get_parameter('initial_angle').get_parameter_value().double_value

		# --- Robot State ---
		self.armed = False
		self.logger = self.get_logger()

		# --- Robot Pose ---
		self.pose_curr = PoseWithCovarianceStamped()
		self.buggy_pose_x = 0.0
		self.buggy_pose_y = 0.0
		self.buggy_center = (0.0, 0.0)
		self.world_center = (0.0, 0.0)

		# --- Map Data ---
		self.simple_map_curr = None
		self.global_map_curr = None

		# --- Goal Management ---
		self.xy_goal_tolerance = 0.5
		self.goal_completed = True  # No goal is currently in-progress.
		self.goal_handle_curr = None
		self.cancelling_goal = False
		self.recovery_threshold = 10

		# --- Goal Creation ---
		self._frame_id = "map"

		# --- Exploration Parameters ---
		self.max_step_dist_world_meters = 7.0
		self.min_step_dist_world_meters = 4.0
		self.full_map_explored_count = 0

		# --- QR Code Data ---
		self.qr_code_str = "Empty"
		# Initialize OpenCV QR code detector (detects & decodes without extra dependencies).
		self.qr_detector = cv2.QRCodeDetector()

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()
		
		# --- Additional State Management ---
		self.exploration_mode = "frontier"  # "frontier", "random", "complete", "shelf_focus"
		self.last_qr_detection_time = self.get_clock().now()
		self.last_object_detection_time = self.get_clock().now()
		self.visited_positions = []  # Track visited positions
		self.max_exploration_attempts = 50
		self.exploration_attempts = 0
		
		# --- Movement Control ---
		self.movement_paused = False
		self.shelf_focus_mode = False
		self.focus_start_time = None
		self.max_focus_time = 30.0  # Maximum time to focus on one shelf
		self.movement_speed_reduction = 0.3  # Slower movement when near shelf
		
		# --- Orbital Movement for Shelf Detection ---
		self.orbital_mode = False
		self.orbital_center = None  # Center point around which to orbit
		self.orbital_radius = 1.5  # Meters from shelf center
		self.orbital_angle = 0.0  # Current orbital angle
		self.orbital_speed = 0.15  # Linear speed during orbital movement
		self.orbital_angular_speed = 0.3  # Angular speed during orbital movement
		self.orbital_start_time = None
		self.max_orbital_time = 45.0  # Maximum time for orbital movement
		self.expected_objects_count = 6  # Expected objects on a shelf
		self.orbital_detection_timer = None
		
		# --- NEW Shelf Detection State (Objects FIRST, then QR) ---
		self.objects_detected = False
		self.current_objects = None
		self.objects_detection_time = None
		self.qr_scanning_enabled = False  # Only scan QR after objects detected
		self.current_shelf_qr = None
		self.objects_timeout = 15.0  # Seconds to wait for stable object detection
		self.qr_timeout = 10.0  # Seconds to wait for QR after objects detected
		self.min_objects_for_shelf = 1  # Minimum objects to consider it a valid shelf
		self.objects_stable_count = 0  # Count of consecutive stable object detections
		self.required_stable_detections = 3  # Required stable detections before enabling QR
		
		# Create a timer for periodic status updates
		self.status_timer = self.create_timer(5.0, self.periodic_status_update)
		
		# Create a timer to check for object-qr association
		self.detection_monitor_timer = self.create_timer(1.0, self.check_detection_workflow)



	def pose_callback(self, message):
		"""Callback function to handle pose updates.

		Args:
			message: ROS2 message containing the current pose of the rover.

		Returns:
			None
		"""
		self.pose_curr = message
		self.buggy_pose_x = message.pose.pose.position.x
		self.buggy_pose_y = message.pose.pose.position.y
		self.buggy_center = (self.buggy_pose_x, self.buggy_pose_y)

	def simple_map_callback(self, message):
		"""Callback function to handle simple map updates.

		Args:
			message: ROS2 message containing the simple map data.

		Returns:
			None
		"""
		self.simple_map_curr = message
		map_info = self.simple_map_curr.info
		self.world_center = self.get_world_coord_from_map_coord(
			map_info.width / 2, map_info.height / 2, map_info
		)

	def global_map_callback(self, message):
		"""Callback function to handle global map updates.

		Args:
			message: ROS2 message containing the global map data.

		Returns:
			None
		"""
		self.global_map_curr = message

		# STOP exploration if we're focusing on a shelf
		if self.shelf_focus_mode or self.movement_paused:
			self.get_logger().debug("🛑 Exploration paused - focusing on shelf")
			return

		if not self.goal_completed:
			return

		height, width = self.global_map_curr.info.height, self.global_map_curr.info.width
		map_array = np.array(self.global_map_curr.data).reshape((height, width))

		frontiers = self.get_frontiers_for_space_exploration(map_array)

		map_info = self.global_map_curr.info
		if frontiers:
			closest_frontier = None
			min_distance_curr = float('inf')

			for fy, fx in frontiers:
				fx_world, fy_world = self.get_world_coord_from_map_coord(fx, fy,
											 map_info)
				distance = euclidean((fx_world, fy_world), self.buggy_center)
				if (distance < min_distance_curr and
				    distance <= self.max_step_dist_world_meters and
				    distance >= self.min_step_dist_world_meters):
					min_distance_curr = distance
					closest_frontier = (fy, fx)

			if closest_frontier:
				fy, fx = closest_frontier
				goal = self.create_goal_from_map_coord(fx, fy, map_info)
				self.send_goal_from_world_pose(goal)
				print("Sending goal for space exploration.")
				return
			else:
				self.max_step_dist_world_meters += 2.0
				new_min_step_dist = self.min_step_dist_world_meters - 1.0
				self.min_step_dist_world_meters = max(0.25, new_min_step_dist)

			self.full_map_explored_count = 0
		else:
			self.full_map_explored_count += 1
			self.exploration_attempts += 1
			print(f"Nothing found in frontiers; count = {self.full_map_explored_count}")
			
			# Switch to random exploration if frontiers are exhausted
			if self.full_map_explored_count > 5 and self.exploration_mode == "frontier":
				self.exploration_mode = "random"
				self.get_logger().info("Switching to random exploration mode")
				self._attempt_random_exploration()
			elif self.exploration_attempts > self.max_exploration_attempts:
				self.exploration_mode = "complete"
				self.get_logger().info("Exploration complete - maximum attempts reached")

	def get_frontiers_for_space_exploration(self, map_array):
		"""Identifies frontiers for space exploration.

		Args:
			map_array: 2D numpy array representing the map.

		Returns:
			frontiers: List of tuples representing frontier coordinates.
		"""
		frontiers = []
		for y in range(1, map_array.shape[0] - 1):
			for x in range(1, map_array.shape[1] - 1):
				if map_array[y, x] == -1:  # Unknown space and not visited.
					neighbors_complete = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
						(y - 1, x - 1),
						(y + 1, x - 1),
						(y - 1, x + 1),
						(y + 1, x + 1)
					]

					near_obstacle = False
					for ny, nx in neighbors_complete:
						if map_array[ny, nx] > 0:  # Obstacles.
							near_obstacle = True
							break
					if near_obstacle:
						continue

					neighbors_cardinal = [
						(y, x - 1),
						(y, x + 1),
						(y - 1, x),
						(y + 1, x),
					]

					for ny, nx in neighbors_cardinal:
						if map_array[ny, nx] == 0:  # Free space.
							frontiers.append((ny, nx))
							break

		return frontiers



	def publish_debug_image(self, publisher, image):
		"""Publishes images for debugging purposes.

		Args:
			publisher: ROS2 publisher of the type sensor_msgs.msg.CompressedImage.
			image: Image given by an n-dimensional numpy array.

		Returns:
			None
		"""
		if image.size:
			message = CompressedImage()
			_, encoded_data = cv2.imencode('.jpg', image)
			message.format = "jpeg"
			message.data = encoded_data.tobytes()
			publisher.publish(message)

	def camera_image_callback(self, message):
		"""Callback function to handle incoming camera images.

		Args:
			message: ROS2 message of the type sensor_msgs.msg.CompressedImage.

		Returns:
			None
		"""
		# Convert the compressed image ROS message to an OpenCV image.
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		if image is None:
			self.get_logger().warning("Failed to decode image from /camera/image_raw/compressed.")
			return

		# Detect and decode QR code(s) with OpenCV QRCodeDetector.
		qr_found = False
		try:
			# detectAndDecodeMulti returns (retval, decoded_info, points, straight_qrcode)
			retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(image)
			if retval:
				for qr_str, pts in zip(decoded_info, points):
					if not qr_str:
						continue
					qr_found = True
					
					# Only process QR codes if we have detected objects first
					if self.qr_scanning_enabled and self.objects_detected:
						if qr_str != self.qr_code_str:
							self.qr_code_str = qr_str
							self.current_shelf_qr = qr_str
							self.last_qr_detection_time = self.get_clock().now()
							self.get_logger().info(f"🔗 QR code detected AFTER objects: {qr_str}")
							
							# Process complete shelf now that we have both objects and QR
							self._process_complete_shelf()

						# Draw bounding box around the QR code for visual debugging.
						pts = pts.astype(int).reshape(-1, 2)
						for i in range(len(pts)):
							pt1 = tuple(pts[i])
							pt2 = tuple(pts[(i + 1) % len(pts)])
							cv2.line(image, pt1, pt2, (0, 255, 0), 2)
						# Put text near first corner
						pt_text = tuple(pts[0])
						cv2.putText(image, qr_str, pt_text, cv2.FONT_HERSHEY_SIMPLEX,
							0.5, (0, 255, 0), 2, cv2.LINE_AA)
					else:
						# Draw QR but indicate it's not being processed yet
						pts = pts.astype(int).reshape(-1, 2)
						for i in range(len(pts)):
							pt1 = tuple(pts[i])
							pt2 = tuple(pts[(i + 1) % len(pts)])
							cv2.line(image, pt1, pt2, (0, 0, 255), 2)  # Red = not ready
						pt_text = tuple(pts[0])
						status = "WAITING FOR OBJECTS" if not self.objects_detected else "QR READY"
						cv2.putText(image, f"{qr_str} ({status})", pt_text, cv2.FONT_HERSHEY_SIMPLEX,
							0.4, (0, 0, 255), 2, cv2.LINE_AA)
		except Exception as e:
			self.get_logger().error(f"Error in QR detection: {e}")

		# Log when QR codes are not found for extended periods.
		if not qr_found:
			self.get_logger().debug("No QR code detected in current frame.")

		# Publish the annotated image so that it can be viewed in Foxglove (optional).
		self.publish_debug_image(self.publisher_qr_decode, image)

	def cerebri_status_callback(self, message):
		"""Callback function to handle cerebri status updates.

		Args:
			message: ROS2 message containing cerebri status.

		Returns:
			None
		"""
		if message.mode == 3 and message.arming == 2:
			self.armed = True
		else:
			# Initialize and arm the CMD_VEL mode.
			msg = Joy()
			msg.buttons = [0, 1, 0, 0, 0, 0, 0, 1]
			msg.axes = [0.0, 0.0, 0.0, 0.0]
			self.publisher_joy.publish(msg)

	def behavior_tree_log_callback(self, message):
		"""Alternative method for checking goal status.

		Args:
			message: ROS2 message containing behavior tree log.

		Returns:
			None
		"""
		for event in message.event_log:
			if (event.node_name == "FollowPath" and
				event.previous_status == "SUCCESS" and
				event.current_status == "IDLE"):
				# self.goal_completed = True
				# self.goal_handle_curr = None
				pass

	def shelf_objects_callback(self, message):
		"""Callback function to handle shelf objects updates.
		NEW WORKFLOW: Objects detected FIRST, then QR scanning enabled.

		Args:
			message: ROS2 message containing shelf objects data.

		Returns:
			None
		"""
		self.shelf_objects_curr = message
		
		# Process object detection (step 1 of new workflow)
		if len(message.object_name) > 0:
			self.last_object_detection_time = self.get_clock().now()
			
			# Check if objects are sufficient for a shelf
			if len(message.object_name) >= self.min_objects_for_shelf:
				
				# Check if objects are stable (same as previous detection)
				if self._are_objects_stable(message):
					self.objects_stable_count += 1
					self.get_logger().info(f"📦 Stable objects detected ({self.objects_stable_count}/{self.required_stable_detections}): {len(message.object_name)} items")
					
					# If objects are stable for required count, enable QR scanning
					if self.objects_stable_count >= self.required_stable_detections and not self.objects_detected:
						self._enable_qr_scanning_after_objects(message)
					
				else:
					# Objects changed, reset stability counter
					self.objects_stable_count = 1
					self.current_objects = message
					obj_list = [f"{name}({count})" for name, count in zip(message.object_name, message.object_count)]
					self.get_logger().info(f"📦 New objects detected (resetting stability): {', '.join(obj_list)}")
				
				# SLOW DOWN movement when objects are being detected (but not full focus yet)
				if not self.shelf_focus_mode:
					self._slow_down_for_potential_shelf()
				
			else:
				self.get_logger().debug(f"📦 Too few objects ({len(message.object_name)}) - need at least {self.min_objects_for_shelf}")
		else:
			# No objects detected, reset state
			if self.objects_detected or self.qr_scanning_enabled:
				self.get_logger().info("📦 No objects detected - resetting detection state")
				self._reset_detection_state()

	def _are_objects_stable(self, new_message):
		"""Check if the detected objects are the same as previous detection."""
		if self.current_objects is None:
			return False
		
		# Compare object names and counts
		if len(new_message.object_name) != len(self.current_objects.object_name):
			return False
		
		# Create sorted lists for comparison
		new_objects = sorted(zip(new_message.object_name, new_message.object_count))
		current_objects = sorted(zip(self.current_objects.object_name, self.current_objects.object_count))
		
		return new_objects == current_objects

	def _enable_qr_scanning_after_objects(self, objects_message):
		"""Enable QR scanning after stable object detection."""
		self.objects_detected = True
		self.current_objects = objects_message
		self.objects_detection_time = self.get_clock().now()
		self.qr_scanning_enabled = True
		
		# CRITICAL: Enable shelf focus mode to stop exploration
		self._enter_shelf_focus_mode()
		
		obj_list = [f"{name}({count})" for name, count in zip(objects_message.object_name, objects_message.object_count)]
		self.get_logger().info(f"✅ OBJECTS CONFIRMED: {', '.join(obj_list)} - QR scanning now ENABLED")
		self.get_logger().info(f"🎯 SHELF FOCUS MODE: Robot will focus entirely on this shelf")
		
		# Update GUI with detected objects
		if PROGRESS_TABLE_GUI and box_app is not None:
			try:
				current_shelf_col = box_app.get_current_shelf_column()
				object_pairs = list(zip(objects_message.object_name, objects_message.object_count))
				box_app.update_shelf_objects(current_shelf_col, object_pairs)
				self.get_logger().info(f"📊 Updated GUI shelf {current_shelf_col} with objects")
			except Exception as e:
				self.get_logger().warning(f"GUI update failed: {e}")

	def _enter_shelf_focus_mode(self):
		"""Enter shelf focus mode - start orbital movement around shelf."""
		self.shelf_focus_mode = True
		self.movement_paused = False  # Don't pause movement, use orbital instead
		self.focus_start_time = self.get_clock().now()
		self.exploration_mode = "shelf_focus"
		
		# Cancel any current navigation goal
		if not self.goal_completed and self.goal_handle_curr is not None:
			self.get_logger().info("🛑 Cancelling navigation goal to focus on shelf")
			self.cancel_current_goal()
		
		# Start orbital movement around the current position
		self._start_orbital_movement()
		self.get_logger().info("🎯 ENTERED SHELF FOCUS MODE - starting orbital movement")

	def _exit_shelf_focus_mode(self):
		"""Exit shelf focus mode and resume exploration."""
		self.shelf_focus_mode = False
		self.movement_paused = False
		self.focus_start_time = None
		self.exploration_mode = "frontier"  # Resume normal exploration
		
		# Stop orbital movement
		self._stop_orbital_movement()
		self.get_logger().info("🚀 EXITED SHELF FOCUS MODE - resuming exploration")

	def _slow_down_for_potential_shelf(self):
		"""Slow down robot movement when objects are detected but shelf not confirmed yet."""
		# If robot has an active goal, we can't directly control speed
		# But we can log that we should be more careful
		self.get_logger().debug("🐌 Potential shelf detected - robot should move carefully")
		# Note: In a real implementation, you might want to:
		# - Reduce navigation goal distances
		# - Send slower velocity commands
		# - Increase recovery behaviors patience

	def rover_move_careful(self, speed_multiplier=0.3):
		"""Move robot more carefully when near potential shelves."""
		if self.shelf_focus_mode:
			# Don't move at all in shelf focus mode - orbital movement handles this
			return
		
		# Reduce speed for careful movement
		careful_speed = 0.2 * speed_multiplier  # Very slow forward movement
		careful_turn = 0.1 * speed_multiplier   # Very slow turning
		
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, careful_speed, 0.0, careful_turn]
		self.publisher_joy.publish(msg)
		self.get_logger().debug(f"🐌 Careful movement: speed={careful_speed:.2f}, turn={careful_turn:.2f}")

	def _start_orbital_movement(self):
		"""Start orbital movement around the current position."""
		if self.pose_curr is None:
			self.get_logger().warning("⚠️  Cannot start orbital movement - no pose available")
			return
			
		# Use current position as the center of orbit
		self.orbital_center = (self.buggy_pose_x, self.buggy_pose_y)
		self.orbital_mode = True
		self.orbital_angle = 0.0
		self.orbital_start_time = self.get_clock().now()
		
		# Create a timer for orbital movement execution
		if self.orbital_detection_timer is not None:
			self.orbital_detection_timer.cancel()
		self.orbital_detection_timer = self.create_timer(0.1, self._orbital_movement_step)
		
		self.get_logger().info(f"🌀 Starting orbital movement around point ({self.orbital_center[0]:.2f}, {self.orbital_center[1]:.2f})")

	def _stop_orbital_movement(self):
		"""Stop orbital movement."""
		self.orbital_mode = False
		self.orbital_center = None
		self.orbital_start_time = None
		
		if self.orbital_detection_timer is not None:
			self.orbital_detection_timer.cancel()
			self.orbital_detection_timer = None
			
		# Stop the robot
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, 0.0, 0.0, 0.0]
		self.publisher_joy.publish(msg)
		
		self.get_logger().info("🛑 Stopped orbital movement")

	def _orbital_movement_step(self):
		"""Execute one step of orbital movement."""
		if not self.orbital_mode or self.orbital_center is None:
			return
			
		# Check timeout
		current_time = self.get_clock().now()
		if (self.orbital_start_time and 
			(current_time - self.orbital_start_time).nanoseconds / 1e9 > self.max_orbital_time):
			self.get_logger().warning(f"⏰ Orbital movement timeout after {self.max_orbital_time}s")
			self._complete_orbital_detection()
			return
			
		# Check if we have enough objects detected
		if (self.current_objects and 
			len(self.current_objects.object_name) >= self.expected_objects_count):
			self.get_logger().info(f"✅ All {self.expected_objects_count} objects detected! Stopping orbital movement")
			self._complete_orbital_detection()
			return
			
		# Continue orbital movement
		self._execute_orbital_movement()

	def _execute_orbital_movement(self):
		"""Execute orbital movement around the shelf."""
		if not self.orbital_mode or self.orbital_center is None or self.pose_curr is None:
			return
			
		# Calculate desired position on the orbit
		self.orbital_angle += self.orbital_angular_speed * 0.1  # 0.1s timer interval
		if self.orbital_angle > 2 * math.pi:
			self.orbital_angle -= 2 * math.pi
			
		desired_x = self.orbital_center[0] + self.orbital_radius * math.cos(self.orbital_angle)
		desired_y = self.orbital_center[1] + self.orbital_radius * math.sin(self.orbital_angle)
		
		# Calculate movement towards desired position
		current_x = self.buggy_pose_x
		current_y = self.buggy_pose_y
		
		dx = desired_x - current_x
		dy = desired_y - current_y
		distance_to_target = math.sqrt(dx*dx + dy*dy)
		
		# Calculate desired orientation (facing the shelf center)
		angle_to_center = math.atan2(self.orbital_center[1] - current_y, 
									self.orbital_center[0] - current_x)
		
		# Get current orientation
		current_orientation = self._get_current_yaw()
		
		# Calculate turn command
		angle_diff = angle_to_center - current_orientation
		# Normalize angle difference to [-pi, pi]
		while angle_diff > math.pi:
			angle_diff -= 2 * math.pi
		while angle_diff < -math.pi:
			angle_diff += 2 * math.pi
			
		# Calculate movement commands
		speed = min(self.orbital_speed, distance_to_target * 2.0)  # Proportional speed
		turn = max(-1.0, min(1.0, angle_diff * 2.0))  # Proportional turn
		
		# Apply movement
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, speed, 0.0, turn]
		self.publisher_joy.publish(msg)
		
		self.get_logger().debug(f"🌀 Orbital: angle={self.orbital_angle:.2f}, "
							   f"target=({desired_x:.2f},{desired_y:.2f}), "
							   f"speed={speed:.2f}, turn={turn:.2f}")

	def _get_current_yaw(self):
		"""Get current yaw angle from pose."""
		if self.pose_curr is None:
			return 0.0
			
		# Convert quaternion to yaw
		orientation = self.pose_curr.pose.pose.orientation
		siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
		cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
		yaw = math.atan2(siny_cosp, cosy_cosp)
		return yaw

	def _complete_orbital_detection(self):
		"""Complete the orbital detection phase and proceed with QR scanning."""
		self._stop_orbital_movement()
		
		if self.current_objects and len(self.current_objects.object_name) > 0:
			# Mark objects as detected and enable QR scanning
			self.objects_detected = True
			self.objects_detection_time = self.get_clock().now()
			self.qr_scanning_enabled = True
			
			obj_summary = [f"{name}({count})" for name, count in zip(self.current_objects.object_name, self.current_objects.object_count)]
			self.get_logger().info(f"🎯 Orbital detection complete! Objects: [{', '.join(obj_summary)}] - Enabling QR scanning")
		else:
			self.get_logger().warning("⚠️  Orbital detection complete but no objects found")
			self._reset_detection_state()

	def _process_complete_shelf(self):
		"""Process complete shelf when both objects and QR are available."""
		if not self.objects_detected or not self.current_shelf_qr or not self.current_objects:
			self.get_logger().warning("⚠️  Cannot process shelf - missing objects or QR")
			return
		
		# Create shelf data message for evaluation
		shelf_data_message = WarehouseShelf()
		shelf_data_message.object_name = self.current_objects.object_name
		shelf_data_message.object_count = self.current_objects.object_count
		shelf_data_message.qr_decoded = self.current_shelf_qr
		
		# Publish the shelf data for evaluation
		self.publisher_shelf_data.publish(shelf_data_message)
		
		obj_summary = [f"{name}({count})" for name, count in zip(self.current_objects.object_name, self.current_objects.object_count)]
		self.get_logger().info(f"🎉 SHELF COMPLETE: QR='{self.current_shelf_qr}', Objects=[{', '.join(obj_summary)}]")

		# Update GUI with QR code
		if PROGRESS_TABLE_GUI and box_app is not None:
			try:
				current_shelf_col = box_app.get_current_shelf_column()
				box_app.update_shelf_qr(current_shelf_col, self.current_shelf_qr)
				self.get_logger().info(f"📊 Updated GUI shelf {current_shelf_col} with QR: {self.current_shelf_qr}")
			except Exception as e:
				self.get_logger().warning(f"GUI QR update failed: {e}")
		
		# Exit shelf focus mode and reset detection state for next shelf
		self._exit_shelf_focus_mode()
		self._reset_detection_state()
	
	def _reset_detection_state(self):
		"""Reset all detection state for next shelf."""
		self.objects_detected = False
		self.current_objects = None
		self.objects_detection_time = None
		self.qr_scanning_enabled = False
		self.current_shelf_qr = None
		self.objects_stable_count = 0
		
		# Stop orbital movement if active
		if self.orbital_mode:
			self._stop_orbital_movement()
		
		# Also exit shelf focus mode if still active
		if self.shelf_focus_mode:
			self._exit_shelf_focus_mode()
			
		self.get_logger().info("🔄 Detection state reset. Ready for next shelf.")
	
	def check_detection_workflow(self):
		"""Periodic check to handle timeouts in the new detection workflow."""
		current_time = self.get_clock().now()
		
		# Check shelf focus mode timeout (but not during orbital movement which has its own timeout)
		if (self.shelf_focus_mode and self.focus_start_time and not self.orbital_mode and
			(current_time - self.focus_start_time).nanoseconds / 1e9 > self.max_focus_time):
			self.get_logger().warning(f"⏰ Shelf Focus Timeout: Focused for {self.max_focus_time}s, resuming exploration")
			self._reset_detection_state()  # This will also exit focus mode
		
		# Check timeout for objects detection
		if (self.objects_detected and self.objects_detection_time and not self.current_shelf_qr and
			(current_time - self.objects_detection_time).nanoseconds / 1e9 > self.qr_timeout):
			obj_list = [f"{name}({count})" for name, count in zip(self.current_objects.object_name, self.current_objects.object_count)]
			self.get_logger().warning(f"⏰ QR Timeout: Objects detected [{', '.join(obj_list)}] but no QR code found within {self.qr_timeout}s. Resetting.")
			self._reset_detection_state()
		
		# Check timeout for object stability
		if (self.objects_stable_count > 0 and not self.objects_detected and self.last_object_detection_time and
			(current_time - self.last_object_detection_time).nanoseconds / 1e9 > self.objects_timeout):
			self.get_logger().warning(f"⏰ Objects Timeout: No stable objects detected within {self.objects_timeout}s. Resetting.")
			self._reset_detection_state()

	def rover_move_manual_mode(self, speed, turn):
		"""Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: The speed of the car in float. Range = [-1.0, +1.0];
				   Direction: forward for positive, reverse for negative.
			turn: Steer value of the car in float. Range = [-1.0, +1.0];
				  Direction: left turn for positive, right turn for negative.

		Returns:
			None
		"""
		# Handle orbital movement when in shelf focus mode
		if self.shelf_focus_mode and self.orbital_mode:
			self._execute_orbital_movement()
			return
		elif self.movement_paused:
			speed = 0.0
			turn = 0.0
			self.get_logger().debug("🛑 Movement blocked - movement paused")
		elif self.objects_stable_count > 0 and not self.objects_detected:
			# Slow down when objects are being detected but not confirmed
			speed = speed * self.movement_speed_reduction
			turn = turn * self.movement_speed_reduction
			self.get_logger().debug(f"🐌 Movement reduced - objects stabilizing")

		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, speed, 0.0, turn]
		self.publisher_joy.publish(msg)



	def cancel_goal_callback(self, future):
		"""
		Callback function executed after a cancellation request is processed.

		Args:
			future (rclpy.Future): The future is the result of the cancellation request.
		"""
		cancel_result = future.result()
		if cancel_result:
			self.logger.info("Goal cancellation successful.")
			self.cancelling_goal = False  # Mark cancellation as completed (success).
			return True
		else:
			self.logger.error("Goal cancellation failed.")
			self.cancelling_goal = False  # Mark cancellation as completed (failed).
			return False

	def cancel_current_goal(self):
		"""Requests cancellation of the currently active navigation goal."""
		if self.goal_handle_curr is not None and not self.cancelling_goal:
			self.cancelling_goal = True  # Mark cancellation in-progress.
			self.logger.info("Requesting cancellation of current goal...")
			cancel_future = self.action_client._cancel_goal_async(self.goal_handle_curr)
			cancel_future.add_done_callback(self.cancel_goal_callback)

	def goal_result_callback(self, future):
		"""
		Callback function executed when the navigation goal reaches a final result.

		Args:
			future (rclpy.Future): The future that is result of the navigation action.
		"""
		status = future.result().status
		# NOTE: Refer https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html.

		if status == GoalStatus.STATUS_SUCCEEDED:
			self.logger.info("Goal completed successfully!")
		else:
			self.logger.warn(f"Goal failed with status: {status}")

		self.goal_completed = True  # Mark goal as completed.
		self.goal_handle_curr = None  # Clear goal handle.

	def goal_response_callback(self, future):
		"""
		Callback function executed after the goal is sent to the action server.

		Args:
			future (rclpy.Future): The future that is server's response to goal request.
		"""
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.logger.warn('Goal rejected :(')
			self.goal_completed = True  # Mark goal as completed (rejected).
			self.goal_handle_curr = None  # Clear goal handle.
		else:
			self.logger.info('Goal accepted :)')
			self.goal_completed = False  # Mark goal as in progress.
			self.goal_handle_curr = goal_handle  # Store goal handle.

			get_result_future = goal_handle.get_result_async()
			get_result_future.add_done_callback(self.goal_result_callback)

	def goal_feedback_callback(self, msg):
		"""
		Callback function to receive feedback from the navigation action.

		Args:
			msg (nav2_msgs.action.NavigateToPose.Feedback): The feedback message.
		"""
		distance_remaining = msg.feedback.distance_remaining
		number_of_recoveries = msg.feedback.number_of_recoveries
		navigation_time = msg.feedback.navigation_time.sec
		estimated_time_remaining = msg.feedback.estimated_time_remaining.sec

		self.logger.debug(f"Recoveries: {number_of_recoveries}, "
				  f"Navigation time: {navigation_time}s, "
				  f"Distance remaining: {distance_remaining:.2f}, "
				  f"Estimated time remaining: {estimated_time_remaining}s")

		if number_of_recoveries > self.recovery_threshold and not self.cancelling_goal:
			self.logger.warn(f"Cancelling. Recoveries = {number_of_recoveries}.")
			self.cancel_current_goal()  # Unblock by discarding the current goal.

	def send_goal_from_world_pose(self, goal_pose):
		"""
		Sends a navigation goal to the Nav2 action server.

		Args:
			goal_pose (geometry_msgs.msg.PoseStamped): The goal pose in the world frame.

		Returns:
			bool: True if the goal was successfully sent, False otherwise.
		"""
		if not self.goal_completed or self.goal_handle_curr is not None:
			return False

		self.goal_completed = False  # Starting a new goal.

		goal = NavigateToPose.Goal()
		goal.pose = goal_pose

		if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
			self.logger.error('NavigateToPose action server not available!')
			return False

		# Send goal asynchronously (non-blocking).
		goal_future = self.action_client.send_goal_async(goal, self.goal_feedback_callback)
		goal_future.add_done_callback(self.goal_response_callback)

		return True



	def _get_map_conversion_info(self, map_info) -> Optional[Tuple[float, float]]:
		"""Helper function to get map origin and resolution."""
		if map_info:
			origin = map_info.origin
			resolution = map_info.resolution
			return resolution, origin.position.x, origin.position.y
		else:
			return None

	def get_world_coord_from_map_coord(self, map_x: int, map_y: int, map_info) \
					   -> Tuple[float, float]:
		"""Converts map coordinates to world coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			world_x = (map_x + 0.5) * resolution + origin_x
			world_y = (map_y + 0.5) * resolution + origin_y
			return (world_x, world_y)
		else:
			return (0.0, 0.0)

	def get_map_coord_from_world_coord(self, world_x: float, world_y: float, map_info) \
					   -> Tuple[int, int]:
		"""Converts world coordinates to map coordinates."""
		if map_info:
			resolution, origin_x, origin_y = self._get_map_conversion_info(map_info)
			map_x = int((world_x - origin_x) / resolution)
			map_y = int((world_y - origin_y) / resolution)
			return (map_x, map_y)
		else:
			return (0, 0)

	def _create_quaternion_from_yaw(self, yaw: float) -> Quaternion:
		"""Helper function to create a Quaternion from a yaw angle."""
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		q = Quaternion()
		q.x = 0.0
		q.y = 0.0
		q.z = sy
		q.w = cy
		return q

	def create_yaw_from_vector(self, dest_x: float, dest_y: float,
				   source_x: float, source_y: float) -> float:
		"""Calculates the yaw angle from a source to a destination point.
			NOTE: This function is independent of the type of map used.

			Input: World coordinates for destination and source.
			Output: Angle (in radians) with respect to x-axis.
		"""
		delta_x = dest_x - source_x
		delta_y = dest_y - source_y
		yaw = math.atan2(delta_y, delta_x)

		return yaw

	def create_goal_from_world_coord(self, world_x: float, world_y: float,
					 yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from world coordinates.
			NOTE: This function is independent of the type of map used.
		"""
		goal_pose = PoseStamped()
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		goal_pose.header.frame_id = self._frame_id

		goal_pose.pose.position.x = world_x
		goal_pose.pose.position.y = world_y

		if yaw is None and self.pose_curr is not None:
			# Calculate yaw from current position to goal position.
			source_x = self.pose_curr.pose.pose.position.x
			source_y = self.pose_curr.pose.pose.position.y
			yaw = self.create_yaw_from_vector(world_x, world_y, source_x, source_y)
		elif yaw is None:
			yaw = 0.0
		else:  # No processing needed; yaw is supplied by the user.
			pass

		goal_pose.pose.orientation = self._create_quaternion_from_yaw(yaw)

		pose = goal_pose.pose.position
		print(f"Goal created: ({pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f})")
		return goal_pose

	def create_goal_from_map_coord(self, map_x: int, map_y: int, map_info,
				       yaw: Optional[float] = None) -> PoseStamped:
		"""Creates a goal PoseStamped from map coordinates."""
		world_x, world_y = self.get_world_coord_from_map_coord(map_x, map_y, map_info)

		return self.create_goal_from_world_coord(world_x, world_y, yaw)

	def _attempt_random_exploration(self):
		"""Attempts random exploration when frontiers are exhausted."""
		if not self.global_map_curr:
			return
			
		map_info = self.global_map_curr.info
		height, width = map_info.height, map_info.width
		map_array = np.array(self.global_map_curr.data).reshape((height, width))
		
		# Find free space cells for random exploration
		free_cells = []
		for y in range(height):
			for x in range(width):
				if map_array[y, x] == 0:  # Free space
					world_x, world_y = self.get_world_coord_from_map_coord(x, y, map_info)
					distance_from_robot = euclidean((world_x, world_y), self.buggy_center)
					
					# Check if we haven't been too close to this position before
					too_close_to_visited = False
					for visited_pos in self.visited_positions:
						if euclidean((world_x, world_y), visited_pos) < 2.0:
							too_close_to_visited = True
							break
					
					if (not too_close_to_visited and 
						distance_from_robot > 3.0 and 
						distance_from_robot < 15.0):
						free_cells.append((x, y))
		
		if free_cells:
			# Select a random free cell
			selected_cell = random.choice(free_cells)
			x, y = selected_cell
			goal = self.create_goal_from_map_coord(x, y, map_info)
			
			if self.send_goal_from_world_pose(goal):
				world_x, world_y = self.get_world_coord_from_map_coord(x, y, map_info)
				self.visited_positions.append((world_x, world_y))
				self.get_logger().info(f"Attempting random exploration to ({world_x:.2f}, {world_y:.2f})")
				
				# Keep only recent visited positions to avoid memory growth
				if len(self.visited_positions) > 20:
					self.visited_positions = self.visited_positions[-10:]

	def periodic_status_update(self):
		"""Periodic status update for monitoring exploration progress."""
		current_time = self.get_clock().now()
		
		# Calculate time since last detections
		qr_time_diff = (current_time - self.last_qr_detection_time).nanoseconds / 1e9
		obj_time_diff = (current_time - self.last_object_detection_time).nanoseconds / 1e9
		
		# Detection status information
		if self.objects_detected and self.current_shelf_qr:
			detection_status = f"Complete: {self.current_shelf_qr}"
		elif self.objects_detected:
			detection_status = f"Objects OK, QR waiting"
		elif self.objects_stable_count > 0:
			detection_status = f"Objects stabilizing ({self.objects_stable_count}/{self.required_stable_detections})"
		else:
			detection_status = "Waiting for objects"
		
		qr_status = "Enabled" if self.qr_scanning_enabled else "Disabled"
		if self.shelf_focus_mode and self.orbital_mode:
			focus_status = "ORBITAL DETECTION"
		elif self.shelf_focus_mode:
			focus_status = "SHELF FOCUS"
		else:
			focus_status = "Exploring"
		movement_status = "PAUSED" if self.movement_paused else "Active"
		
		orbital_info = ""
		if self.orbital_mode and self.current_objects:
			current_obj_count = len(self.current_objects.object_name)
			orbital_info = f", Orbital Objects: {current_obj_count}/{self.expected_objects_count}"
		
		status_msg = (
			f"🤖 Status - Mode: {focus_status}, "
			f"Movement: {movement_status}, "
			f"Armed: {self.armed}, "
			f"Goal Active: {not self.goal_completed}, "
			f"Detection: {detection_status}, "
			f"QR Scan: {qr_status}, "
			f"Last Objects: {obj_time_diff:.1f}s ago{orbital_info}"
		)
		
		self.get_logger().info(status_msg)
		
		# Check if we should reset detection state if no object activity for a while
		if obj_time_diff > 30.0 and (self.objects_detected or self.qr_scanning_enabled):
			self.qr_code_str = "Empty"
			self._reset_detection_state()
			self.get_logger().info("Reset detection state due to no recent object activity")

	def get_exploration_statistics(self):
		"""Returns current exploration statistics."""
		return {
			'mode': self.exploration_mode,
			'attempts': self.exploration_attempts,
			'max_attempts': self.max_exploration_attempts,
			'visited_positions': len(self.visited_positions),
			'current_qr': self.qr_code_str,
			'objects_detected': self.objects_detected,
			'qr_scanning_enabled': self.qr_scanning_enabled,
			'current_shelf_qr': self.current_shelf_qr,
			'objects_stable_count': self.objects_stable_count,
			'current_objects_count': len(self.current_objects.object_name) if self.current_objects else 0,
			'armed': self.armed,
			'goal_active': not self.goal_completed
		}


def main(args=None):
	rclpy.init(args=args)

	warehouse_explore = WarehouseExplore()

	if PROGRESS_TABLE_GUI:
		gui_thread = threading.Thread(target=run_gui, args=(warehouse_explore.shelf_count,))
		gui_thread.start()

	rclpy.spin(warehouse_explore)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	warehouse_explore.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()