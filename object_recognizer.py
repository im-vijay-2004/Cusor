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
from synapse_msgs.msg import WarehouseShelf

import cv2
import numpy as np
import os
import time
import yaml
from typing import Dict, List, Tuple, Optional

from sensor_msgs.msg import CompressedImage

try:
	import pkg_resources
except ImportError:
	pkg_resources = None

try:
	import torch
	import torchvision
	TORCH_AVAILABLE = True
except ImportError:
	TORCH_AVAILABLE = False
	print("Warning: PyTorch not available. Some functionality may be limited.")

try:
	import tflite_runtime.interpreter as tflite
	TFLITE_AVAILABLE = True
except ImportError:
	try:
		import tensorflow.lite as tflite
		TFLITE_AVAILABLE = True
	except ImportError:
		TFLITE_AVAILABLE = False
		print("Warning: TensorFlow Lite not available. Model inference disabled.")

QOS_PROFILE_DEFAULT = 10

PACKAGE_NAME = 'b3rb_ros_aim_india'

# Color definitions for debug visualization
RED_COLOR = (0, 0, 255)
BLUE_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)
YELLOW_COLOR = (0, 255, 255)
PURPLE_COLOR = (255, 0, 255)

# Shelf-relevant object categories (subset of COCO classes that are typically found on warehouse shelves)
SHELF_RELEVANT_OBJECTS = {
	'bottle', 'cup', 'bowl', 'banana', 'apple', 'orange', 'broccoli', 'carrot',
	'hot dog', 'pizza', 'donut', 'cake', 'book', 'clock', 'vase', 'scissors',
	'teddy bear', 'hair drier', 'toothbrush', 'wine glass', 'fork', 'knife',
	'spoon', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
	'oven', 'toaster', 'sink', 'refrigerator', 'blender', 'backpack', 'umbrella',
	'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
	'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 
	'tennis racket', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck'
}


def xywh2xyxy(x):
	"""Converts bounding box from xywh to xyxy format."""
	if TORCH_AVAILABLE and isinstance(x, torch.Tensor):
		y = x.clone()
	else:
		y = np.copy(x)
	
	y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
	y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
	y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
	y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y

	return y


def non_max_suppression(
	prediction,
	conf_thres=0.25,
	iou_thres=0.45,
	classes=None,
	agnostic=False,
	multi_label=False,
	labels=(),
	max_det=300,
	nm=0,  # number of masks
):
	"""
	Non-Maximum Suppression (NMS) on inference results to reject overlapping detections.

	Returns:
		 list of detections, on (n,6) tensor per image [xyxy, conf, cls]
	"""
	if not TORCH_AVAILABLE:
		return []

	# Checks
	assert 0 <= conf_thres <= 1, f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
	assert 0 <= iou_thres <= 1, f"Invalid IoU threshold value {iou_thres}, valid values are between 0.0 and 1.0"
	if isinstance(prediction, (list, tuple)):  # YOLOv5 model in validation model, output = (inference_out, loss_out)
		prediction = prediction[0]  # select only inference output

	device = prediction.device
	mps = "mps" in device.type  # Apple MPS
	if mps:  # MPS not fully supported yet, convert tensors to CPU before NMS
		prediction = prediction.cpu()
	bs = prediction.shape[0]  # batch size
	nc = prediction.shape[2] - nm - 5  # number of classes
	xc = prediction[..., 4] > conf_thres  # candidates

	# Settings
	max_wh = 7680  # (pixels) maximum box width and height
	max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
	time_limit = 0.5 + 0.05 * bs  # seconds to quit after
	redundant = True  # require redundant detections
	multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
	merge = False  # use merge-NMS

	t = time.time()
	mi = 5 + nc  # mask start index
	output = [torch.zeros((0, 6 + nm), device=prediction.device)] * bs
	for xi, x in enumerate(prediction):  # image index, image inference
		# Apply constraints
		x = x[xc[xi]]  # confidence

		# Cat apriori labels if autolabelling
		if labels and len(labels[xi]):
			lb = labels[xi]
			v = torch.zeros((len(lb), nc + nm + 5), device=x.device)
			v[:, :4] = lb[:, 1:5]  # box
			v[:, 4] = 1.0  # conf
			v[range(len(lb)), lb[:, 0].long() + 5] = 1.0  # cls
			x = torch.cat((x, v), 0)

		# If none remain process next image
		if not x.shape[0]:
			continue

		# Compute conf
		x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

		# Box/Mask
		box = xywh2xyxy(x[:, :4])  # center_x, center_y, width, height) to (x1, y1, x2, y2)
		mask = x[:, mi:]  # zero columns if no masks

		# Detections matrix nx6 (xyxy, conf, cls)
		if multi_label:
			i, j = (x[:, 5:mi] > conf_thres).nonzero(as_tuple=False).T
			x = torch.cat((box[i], x[i, 5 + j, None], j[:, None].float(), mask[i]), 1)
		else:  # best class only
			conf, j = x[:, 5:mi].max(1, keepdim=True)
			x = torch.cat((box, conf, j.float(), mask), 1)[conf.view(-1) > conf_thres]

		# Filter by class
		if classes is not None:
			x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

		# Check shape
		n = x.shape[0]  # number of boxes
		if not n:  # no boxes
			continue
		x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence and remove excess boxes

		# Batched NMS
		c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
		boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
		i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
		i = i[:max_det]  # limit detections
		if merge and (1 < n < 3e3):  # Merge NMS (boxes merged using weighted mean)
			# update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
			iou = box.iou(boxes[i], boxes) > iou_thres  # iou matrix
			weights = iou * scores[None]  # box weights
			x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
			if redundant:
				i = i[iou.sum(1) > 1]  # require redundancy

		output[xi] = x[i]
		if mps:
			output[xi] = output[xi].to(device)
		if (time.time() - t) > time_limit:
			break  # time limit exceeded

	return output


class ObjectRecognizer(Node):
	"""Advanced object recognizer node with shelf-aware detection and robust error handling.
	
	This node processes camera images to detect objects that are typically found on warehouse
	shelves, integrating seamlessly with the warehouse exploration workflow.
	"""
	
	def __init__(self):
		super().__init__('object_recognizer')

		# Declare parameters
		self.declare_parameter('confidence_threshold', 0.3)
		self.declare_parameter('iou_threshold', 0.45)
		self.declare_parameter('max_detections', 50)
		self.declare_parameter('filter_shelf_objects', True)
		self.declare_parameter('min_object_size', 0.01)  # Minimum object size as fraction of image
		self.declare_parameter('debug_enabled', True)
		self.declare_parameter('performance_monitoring', True)

		# Get parameters
		self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
		self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
		self.max_detections = self.get_parameter('max_detections').get_parameter_value().integer_value
		self.filter_shelf_objects = self.get_parameter('filter_shelf_objects').get_parameter_value().bool_value
		self.min_object_size = self.get_parameter('min_object_size').get_parameter_value().double_value
		self.debug_enabled = self.get_parameter('debug_enabled').get_parameter_value().bool_value
		self.performance_monitoring = self.get_parameter('performance_monitoring').get_parameter_value().bool_value

		# Subscription for camera images
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for Shelf Objects
		self.publisher_shelf_objects = self.create_publisher(
			WarehouseShelf,
			'/shelf_objects',
			QOS_PROFILE_DEFAULT)

		# Publisher for debug image (only if debug enabled)
		if self.debug_enabled:
			self.publisher_object_recog = self.create_publisher(
				CompressedImage,
				"/debug_images/object_recog",
				QOS_PROFILE_DEFAULT)

		# Initialize model and labels
		self.model_initialized = False
		self.label_names = []
		self.interpreter = None
		self.input_details = None
		self.output_details = None
		
		# Performance tracking
		self.frame_count = 0
		self.total_inference_time = 0.0
		self.last_stats_time = time.time()
		
		# Error tracking
		self.consecutive_errors = 0
		self.max_consecutive_errors = 10

		# Initialize model
		self._initialize_model()
		
		# Create performance monitoring timer
		if self.performance_monitoring:
			self.create_timer(10.0, self._report_performance_stats)

		self.get_logger().info("🔍 Object Recognizer initialized")
		self.get_logger().info(f"📊 Config: conf={self.confidence_threshold:.2f}, iou={self.iou_threshold:.2f}, max_det={self.max_detections}")
		self.get_logger().info(f"🎯 Shelf filtering: {'Enabled' if self.filter_shelf_objects else 'Disabled'}")

	def _initialize_model(self):
		"""Initialize the YOLO model and class labels."""
		try:
			# Load class names
			self._load_class_names()
			
			# Load model
			if TFLITE_AVAILABLE:
				self._load_tflite_model()
			else:
				self.get_logger().error("❌ TensorFlow Lite not available. Object detection disabled.")
				return

			self.model_initialized = True
			self.get_logger().info("✅ Model initialization successful")
			
		except Exception as e:
			self.get_logger().error(f"❌ Model initialization failed: {e}")
			self.model_initialized = False

	def _load_class_names(self):
		"""Load COCO class names from YAML file."""
		try:
			# Try multiple possible locations for the YAML file
			possible_paths = [
				"../../../../share/ament_index/resource_index/coco.yaml",
				"coco.yaml",
				"/opt/ros/humble/share/ament_index/resource_index/coco.yaml",
				os.path.expanduser("~/coco.yaml")
			]
			
			yaml_path = None
			for path in possible_paths:
				try:
					if pkg_resources and PACKAGE_NAME:
						yaml_path = pkg_resources.resource_filename(PACKAGE_NAME, path)
					else:
						yaml_path = path
					
					if os.path.exists(yaml_path):
						break
				except:
					continue
			
			if yaml_path and os.path.exists(yaml_path):
				with open(yaml_path) as f:
					data = yaml.load(f, Loader=yaml.FullLoader)
					self.label_names = data['names']
				self.get_logger().info(f"📝 Loaded {len(self.label_names)} class names from {yaml_path}")
			else:
				# Fallback to hardcoded COCO class names
				self._load_default_coco_names()
				self.get_logger().warning("⚠️  Using default COCO class names (YAML file not found)")
				
		except Exception as e:
			self.get_logger().error(f"❌ Failed to load class names: {e}")
			self._load_default_coco_names()

	def _load_default_coco_names(self):
		"""Load default COCO class names if YAML file is not available."""
		self.label_names = [
			'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
			'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
			'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
			'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
			'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
			'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
			'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
			'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
			'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
			'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
			'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
			'toothbrush'
		]

	def _load_tflite_model(self):
		"""Load TensorFlow Lite model."""
		try:
			# Try multiple possible locations for the model file
			possible_paths = [
				"../../../../share/ament_index/resource_index/yolov5n-int8.tflite",
				"yolov5n-int8.tflite",
				"/opt/ros/humble/share/ament_index/resource_index/yolov5n-int8.tflite",
				os.path.expanduser("~/yolov5n-int8.tflite")
			]
			
			model_path = None
			for path in possible_paths:
				try:
					if pkg_resources and PACKAGE_NAME:
						model_path = pkg_resources.resource_filename(PACKAGE_NAME, path)
					else:
						model_path = path
					
					if os.path.exists(model_path):
						break
				except:
					continue
			
			if not model_path or not os.path.exists(model_path):
				raise FileNotFoundError("YOLO model file not found in any expected location")

			# Initialize TensorFlow Lite interpreter
			ext_delegate_ops = {}
			# Uncomment for NavQPlus NPU support:
			# ext_delegate = [tflite.load_delegate("/usr/lib/libvx_delegate.so", ext_delegate_ops)]
			# self.interpreter = tflite.Interpreter(model_path=model_path, experimental_delegates=ext_delegate)
			
			self.interpreter = tflite.Interpreter(model_path=model_path)
			self.interpreter.allocate_tensors()

			self.input_details = self.interpreter.get_input_details()
			self.output_details = self.interpreter.get_output_details()
			
			self.get_logger().info(f"🤖 Loaded YOLO model from {model_path}")
			
		except Exception as e:
			self.get_logger().error(f"❌ Failed to load TensorFlow Lite model: {e}")
			raise

	def _is_shelf_relevant_object(self, object_name: str) -> bool:
		"""Check if the detected object is relevant for shelf detection."""
		if not self.filter_shelf_objects:
			return True
		return object_name.lower() in SHELF_RELEVANT_OBJECTS

	def _filter_objects_by_size(self, detections: List[Tuple], image_width: int, image_height: int) -> List[Tuple]:
		"""Filter out objects that are too small to be reliably detected."""
		filtered = []
		min_area = self.min_object_size * image_width * image_height
		
		for detection in detections:
			x1, y1, x2, y2 = detection[0:4]
			area = (x2 - x1) * (y2 - y1)
			if area >= min_area:
				filtered.append(detection)
		
		return filtered

	def publish_debug_image(self, publisher, image):
		"""Publishes images for debugging purposes."""
		if not self.debug_enabled or image is None or image.size == 0:
			return
			
		try:
			message = CompressedImage()
			_, encoded_data = cv2.imencode('.jpg', image)
			message.format = "jpeg"
			message.data = encoded_data.tobytes()
			publisher.publish(message)
		except Exception as e:
			self.get_logger().warning(f"⚠️  Failed to publish debug image: {e}")

	def camera_image_callback(self, message):
		"""Analyzes camera images to detect shelf objects.
		
		Processes images through YOLO model and publishes detected objects
		that are relevant for warehouse shelf detection.
		"""
		if not self.model_initialized:
			self.get_logger().warning("⚠️  Model not initialized, skipping frame")
			return

		try:
			start_time = time.time()
			
			# Convert ROS message to OpenCV image
			np_arr = np.frombuffer(message.data, np.uint8)
			original_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
			if original_image is None:
				self.get_logger().warning("⚠️  Failed to decode image")
				return

			height, width, _ = original_image.shape
			self.get_logger().debug(f"📷 Processing image: {width}x{height}")

			# Prepare image for inference
			input_size = self.input_details[0]['shape'][1]
			processed_image = cv2.resize(original_image, (input_size, input_size))
			processed_image = processed_image.astype(np.float32)
			processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2RGB)
			processed_image /= 255.0
			img_input = np.expand_dims(processed_image, axis=0)

			# Prepare for quantized model if needed
			input_details = self.input_details[0]
			int8 = input_details["dtype"] == np.uint8
			if int8:
				scale, zero_point = input_details["quantization"]
				img_input = (img_input / scale + zero_point).astype(np.uint8)

			# Run inference
			self.interpreter.set_tensor(input_details["index"], img_input)
			
			inference_start = time.time()
			self.interpreter.invoke()
			inference_time = time.time() - inference_start
			
			# Get output
			outputs = []
			for output in self.output_details:
				x = self.interpreter.get_tensor(output["index"])
				if int8:
					scale, zero_point = output["quantization"]
					x = (x.astype(np.float32) - zero_point) * scale
				outputs.append(x)

			# Process detections
			object_count_dict = {}
			debug_image = original_image.copy() if self.debug_enabled else None

			# Process each output
			for pred in outputs:
				if not TORCH_AVAILABLE:
					self.get_logger().warning("⚠️  PyTorch not available, skipping NMS")
					break
					
				# Scale predictions to input size
				w, h = self.input_details[0]["shape"][1:3]
				pred[0][..., :4] *= [w, h, w, h]
				pred_tensor = torch.tensor(pred)

				# Apply NMS
				detections = non_max_suppression(
					pred_tensor, 
					conf_thres=self.confidence_threshold,
					iou_thres=self.iou_threshold,
					max_det=self.max_detections
				)

				# Process each detection
				for i, det in enumerate(detections):
					if len(det):
						# Scale detections back to original image size
						det[:, :4] = det[:, :4] * torch.tensor([width/input_size, height/input_size, width/input_size, height/input_size])
						
						# Filter by size
						det_list = det.cpu().numpy().tolist()
						det_list = self._filter_objects_by_size(det_list, width, height)

						for detection in det_list:
							x1, y1, x2, y2, conf, cls = detection
							
							if int(cls) >= len(self.label_names):
								continue
								
							object_name = self.label_names[int(cls)]
							
							# Filter for shelf-relevant objects
							if not self._is_shelf_relevant_object(object_name):
								continue

							# Update object count
							if object_name in object_count_dict:
								object_count_dict[object_name] += 1
							else:
								object_count_dict[object_name] = 1

							# Draw debug visualization
							if self.debug_enabled and debug_image is not None:
								start_point = (int(x1), int(y1))
								end_point = (int(x2), int(y2))
								
								# Color based on confidence
								color = GREEN_COLOR if conf > 0.7 else YELLOW_COLOR if conf > 0.5 else RED_COLOR
								
								cv2.rectangle(debug_image, start_point, end_point, color, 2)
								
								label = f"{object_name} {conf:.2f}"
								cv2.putText(debug_image, label, 
										   (start_point[0], start_point[1] - 10),
										   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

			# Publish debug image
			if self.debug_enabled and debug_image is not None:
				# Add performance info to debug image
				perf_text = f"Inference: {inference_time*1000:.1f}ms | Objects: {sum(object_count_dict.values())}"
				cv2.putText(debug_image, perf_text, (10, 30), 
						   cv2.FONT_HERSHEY_SIMPLEX, 0.7, PURPLE_COLOR, 2, cv2.LINE_AA)
				
				self.publish_debug_image(self.publisher_object_recog, debug_image)

			# Publish shelf objects message
			shelf_objects_message = WarehouseShelf()
			for object_name, count in object_count_dict.items():
				shelf_objects_message.object_name.append(object_name)
				shelf_objects_message.object_count.append(count)

			self.publisher_shelf_objects.publish(shelf_objects_message)

			# Log detection results
			total_objects = sum(object_count_dict.values())
			if total_objects > 0:
				obj_summary = [f"{name}({count})" for name, count in object_count_dict.items()]
				self.get_logger().info(f"🔍 Detected {total_objects} objects: {', '.join(obj_summary)}")
			else:
				self.get_logger().debug("🔍 No objects detected")

			# Update performance stats
			total_time = time.time() - start_time
			self.frame_count += 1
			self.total_inference_time += inference_time
			self.consecutive_errors = 0  # Reset error count on success

			self.get_logger().debug(f"⏱️  Processing time: {total_time*1000:.1f}ms (inference: {inference_time*1000:.1f}ms)")

		except Exception as e:
			self.consecutive_errors += 1
			self.get_logger().error(f"❌ Error processing image: {e}")
			
			if self.consecutive_errors >= self.max_consecutive_errors:
				self.get_logger().error(f"❌ Too many consecutive errors ({self.consecutive_errors}). Reinitializing model...")
				self._initialize_model()
				self.consecutive_errors = 0

	def _report_performance_stats(self):
		"""Report performance statistics periodically."""
		if self.frame_count == 0:
			return
			
		current_time = time.time()
		time_elapsed = current_time - self.last_stats_time
		
		if time_elapsed > 0:
			fps = self.frame_count / time_elapsed
			avg_inference_time = (self.total_inference_time / self.frame_count) * 1000
			
			self.get_logger().info(
				f"📊 Performance: {fps:.1f} FPS | "
				f"Avg inference: {avg_inference_time:.1f}ms | "
				f"Frames processed: {self.frame_count}"
			)
		
		# Reset counters
		self.frame_count = 0
		self.total_inference_time = 0.0
		self.last_stats_time = current_time


def main(args=None):
	rclpy.init(args=args)

	try:
		object_recognizer = ObjectRecognizer()
		rclpy.spin(object_recognizer)
	except KeyboardInterrupt:
		print("\n🛑 Object recognizer stopped by user")
	except Exception as e:
		print(f"❌ Object recognizer failed: {e}")
	finally:
		try:
			object_recognizer.destroy_node()
		except:
			pass
		rclpy.shutdown()


if __name__ == '__main__':
	main()