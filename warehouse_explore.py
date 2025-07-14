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
		# Clear all object boxes for this shelf
		for row in range(len(self.boxes)):
			self.change_box_text(row, shelf_col, "---")
			self.change_box_color(row, shelf_col, "white")
		
		# Fill with detected objects
		for idx, (name, count) in enumerate(object_list):
			if idx < len(self.boxes):
				self.change_box_text(idx, shelf_col, f"{name}: {count}")
				self.change_box_color(idx, shelf_col, "lightgreen")
	
	def update_shelf_qr(self, shelf_col, qr_text):
		"""Update QR code for a specific shelf."""
		if shelf_col < len(self.qr_boxes):
			self.qr_boxes[shelf_col].config(state=tk.NORMAL)
			self.qr_boxes[shelf_col].delete(1.0, tk.END)
			self.qr_boxes[shelf_col].insert(tk.END, f"QR: {qr_text}")
			self.qr_boxes[shelf_col].config(state=tk.DISABLED, bg="lightblue")

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
		if PROGRESS_TABLE_GUI:
			self.table_row_count = 0
			self.table_col_count = 0

		# --- Shelf Data ---
		self.shelf_objects_curr = WarehouseShelf()
		
		# --- Additional State Management ---
		self.exploration_mode = "frontier"  # "frontier", "random", "complete"
		self.last_qr_detection_time = self.get_clock().now()
		self.last_object_detection_time = self.get_clock().now()
		self.visited_positions = []  # Track visited positions
		self.max_exploration_attempts = 50
		self.exploration_attempts = 0
		
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
		
		obj_list = [f"{name}({count})" for name, count in zip(objects_message.object_name, objects_message.object_count)]
		self.get_logger().info(f"✅ OBJECTS CONFIRMED: {', '.join(obj_list)} - QR scanning now ENABLED")
		
		# Update GUI with detected objects
		if PROGRESS_TABLE_GUI and box_app is not None:
			try:
				object_pairs = list(zip(objects_message.object_name, objects_message.object_count))
				box_app.update_shelf_objects(self.table_col_count, object_pairs)
			except Exception as e:
				self.get_logger().warning(f"GUI update failed: {e}")

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
				box_app.update_shelf_qr(self.table_col_count, self.current_shelf_qr)
				self.table_col_count += 1  # Move to next shelf column
			except Exception as e:
				self.get_logger().warning(f"GUI QR update failed: {e}")
		
		# Reset detection state for next shelf
		self._reset_detection_state()
	
	def _reset_detection_state(self):
		"""Reset all detection state for next shelf."""
		self.objects_detected = False
		self.current_objects = None
		self.objects_detection_time = None
		self.qr_scanning_enabled = False
		self.current_shelf_qr = None
		self.objects_stable_count = 0
		self.get_logger().info("🔄 Detection state reset. Ready for next shelf.")
	
	def check_detection_workflow(self):
		"""Periodic check to handle timeouts in the new detection workflow."""
		current_time = self.get_clock().now()
		
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
		
		status_msg = (
			f"Exploration Status - Mode: {self.exploration_mode}, "
			f"Armed: {self.armed}, "
			f"Goal Active: {not self.goal_completed}, "
			f"Detection: {detection_status}, "
			f"QR Scan: {qr_status}, "
			f"Last Objects: {obj_time_diff:.1f}s ago, "
			f"Attempts: {self.exploration_attempts}/{self.max_exploration_attempts}"
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