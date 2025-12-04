#!/usr/bin/env python3
"""
YOLOv8 Object Detection Node for Dobot Robot
Detects objects and publishes their 3D positions for MoveIt planning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import tf2_py
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer

# HSV color thresholds
COLOR_RANGES = {
    "red": [([0, 50, 50], [15, 255, 255]), ([165, 50, 50], [180, 255, 255])],  # More sensitive red detection
    "green": [([36, 60, 60], [86, 255, 255])],
    "blue": [([95, 60, 60], [135, 255, 255])],
    "yellow": [([20, 80, 80], [35, 255, 255])],
    "black": [([0, 0, 0], [180, 255, 50])],  # Low value
    "white": [([0, 0, 200], [180, 30, 255])]  # Low saturation, high value
}

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera parameters (default values, will be updated from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = 'camera_link'
        
        # Homography calibration for consistent camera grid positioning
        self.declare_parameter('calib_img_pts', [131, 175, 495, 168, 500, 435, 117, 444])  # Default calibration image points
        self.declare_parameter('calib_robot_pts', [334.41, 72.54, 337.80, -105.18, 214.22, -104.18, 207.26, 82.09])  # Default robot points in mm
        
        calib_img = self.get_parameter('calib_img_pts').value
        calib_robot = self.get_parameter('calib_robot_pts').value
        
        self.CALIB_IMG_PTS_PX_FULL = np.array([
            [calib_img[0], calib_img[1]],  # TL
            [calib_img[2], calib_img[3]],  # TR
            [calib_img[4], calib_img[5]],  # BR
            [calib_img[6], calib_img[7]],  # BL
        ], dtype=np.float32)

        self.CALIB_ROBOT_PTS_MM = np.array([
            [calib_robot[0], calib_robot[1]],  # TL
            [calib_robot[2], calib_robot[3]],  # TR
            [calib_robot[4], calib_robot[5]],  # BR
            [calib_robot[6], calib_robot[7]],  # BL
        ], dtype=np.float32)
        
        # Publishers
        self.detection_img_pub = self.create_publisher(
            Image, '/yolo/detection_image', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/yolo/detection_markers', 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/yolo/target_pose', 10)
        self.detection_string_pub = self.create_publisher(
            String, '/yolo/detections', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.25)  # Lower threshold for better detection
        self.declare_parameter('object_distance', 0.5)  # Default fallback distance in meters
        self.declare_parameter('target_class', 'cube')  # Target object class
        self.declare_parameter('publish_annotated_image', True)  # Control annotated image publishing
        
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.object_distance = self.get_parameter('object_distance').value
        self.target_class = self.get_parameter('target_class').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        
        # Calibration mode
        self.declare_parameter('calibration_mode', False)  # If true, draw calibration points on image
        self.calibration_mode = self.get_parameter('calibration_mode').value
        
        # Z heights for different object sizes
        self.z_heights = {
            'small': 0.30,
            'medium': 0.40,
            'large': 0.50
        }
        
        self.get_logger().info(f'YOLODetectionNode initialized')
        self.get_logger().info(f'Target class: {self.target_class}, Confidence: {self.conf_threshold}')
        
        # Initialize homography calibration
        self.perform_homography_calibration()
        
        # Timer for fake detections (for testing when no camera)
        self.fake_detection_timer = self.create_timer(5.0, self.publish_fake_detection)
        
    def detect_instances(self, image_bgr):
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        out = {}
        for color, ranges in COLOR_RANGES.items():
            mask_total = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask_total = cv2.bitwise_or(mask_total, mask)
            mask_total = cv2.medianBlur(mask_total, 5)
            cnts, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            instances = []
            for c in cnts:
                area = float(cv2.contourArea(c))
                if area < 30: continue  # Lower threshold for better detection
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                ctr = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int32(box)
                (cx, cy), (w, h), ang = rect
                if w < h: ang += 90.0
                ang = float(ang)
                if ang > 90: ang -= 180
                if ang <= -90: ang += 180
                instances.append({"center": (int(cx), int(cy)), "angle": float(ang), "box": box, "area": area})
            instances = sorted(instances, key=lambda d: d["area"], reverse=True)
            out[color] = instances
        # Debug: Log detected colors and counts
        detected_colors = {color: len(instances) for color, instances in out.items() if instances}
        if detected_colors:
            self.get_logger().info(f'Detected colors: {detected_colors}')
        return out
    
    def parse_class_names(self):
        """Parse YOLO class names to extract size, color, shape information"""
        self.class_info = {}
        for class_id, class_name in self.model.names.items():
            # Expected format: "size_color_shape" (e.g., "small_red_cube")
            parts = class_name.split('_')
            if len(parts) >= 3:
                size = parts[0]
                color = parts[1]
                shape = '_'.join(parts[2:])  # Handle shapes with underscores
                self.class_info[class_id] = {
                    'size': size,
                    'color': color,
                    'shape': shape,
                    'full_name': class_name
                }
            else:
                # Fallback for old format (just shape names)
                self.class_info[class_id] = {
                    'size': 'medium',  # Default
                    'color': 'unknown',  # Will be detected by color analysis
                    'shape': class_name,
                    'full_name': class_name
                }
        
        self.get_logger().info(f'Parsed {len(self.class_info)} classes with size/color/shape info')
    
    def perform_homography_calibration(self):
        """Perform camera calibration using maze corner points for consistent grid positioning"""
        try:
            self.get_logger().info('Performing homography calibration for consistent camera grid positioning...')
            
            # Use the provided calibration points
            image_points = self.CALIB_IMG_PTS_PX_FULL
            robot_points = self.CALIB_ROBOT_PTS_MM
            
            # Convert robot points from mm to meters for consistency
            robot_points_m = robot_points / 1000.0
            
            # Calculate homography matrix to transform from pixel coordinates to robot coordinates
            self.homography_matrix, _ = cv2.findHomography(image_points, robot_points_m)
            
            if self.homography_matrix is not None:
                self.get_logger().info('Homography calibration successful!')
                self.get_logger().info(f'Homography matrix:\n{self.homography_matrix}')
            else:
                self.get_logger().error('Failed to compute homography matrix')
                
        except Exception as e:
            self.get_logger().error(f'Error during homography calibration: {e}')
    
    def pixel_to_robot_coords(self, u, v):
        """Convert pixel coordinates to robot coordinates using homography"""
        try:
            if self.homography_matrix is None:
                self.get_logger().warn('Homography matrix not available, cannot convert pixel to robot coordinates')
                return None
            
            # Add homogeneous coordinate
            pixel_point = np.array([[u, v, 1]], dtype=np.float32).T
            
            # Apply homography transformation
            robot_coords = self.homography_matrix @ pixel_point
            
            # Convert from homogeneous coordinates
            x = robot_coords[0, 0] / robot_coords[2, 0]
            y = robot_coords[1, 0] / robot_coords[2, 0]
            
            # Return robot coordinates in meters (X, Y in base frame)
            return np.array([x, y])
            
        except Exception as e:
            self.get_logger().error(f'Error converting pixel to robot coordinates: {e}')
            return None
    
    def get_z_for_size(self, size_category):
        """Get Z height based on object size category"""
        return self.z_heights.get(size_category, self.object_distance)
    
    def camera_info_callback(self, msg):
        """Update camera calibration parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info('Camera calibration parameters received')
    
    def get_z_for_size(self, size_category):
        """Get Z height based on object size category"""
        return self.z_heights.get(size_category, self.object_distance)
    
    def get_dominant_color(self, image, bbox):
        """Extract dominant color from bounding box region"""
        x1, y1, x2, y2 = [int(c) for c in bbox]
        roi = image[y1:y2, x1:x2]
        
        if roi.size == 0:
            return 'unknown', (0, 0, 0)
        
        # Calculate mean color in HSV space for better color detection
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_hsv = hsv_roi.mean(axis=(0, 1))
        h, s, v = mean_hsv
        
        # Also get BGR for reference
        mean_color = roi.mean(axis=(0, 1))
        b, g, r = mean_color
        
        # Determine color name based on HSV values (more reliable)
        if s < 30:  # Low saturation = grayscale
            if v < 50:
                color_name = 'black'
            elif v > 200:
                color_name = 'white'
            else:
                color_name = 'gray'
        else:
            # High saturation colors
            if h < 10 or h > 170:  # Red
                color_name = 'red'
            elif 10 <= h < 25:  # Orange/Yellow
                if s > 100 and v > 100:
                    color_name = 'yellow' if h < 20 else 'orange'
                else:
                    color_name = 'yellow'
            elif 25 <= h < 35:  # Yellow-Green
                color_name = 'yellow'
            elif 35 <= h < 45:  # Green
                color_name = 'green'
            elif 45 <= h < 85:  # Cyan/Green
                color_name = 'cyan'
            elif 85 <= h < 130:  # Blue
                color_name = 'blue'
            elif 130 <= h < 160:  # Magenta
                color_name = 'magenta'
            else:  # Red (wraparound)
                color_name = 'red'
        
        return color_name, (int(b), int(g), int(r))
    
    def estimate_real_size(self, bbox, distance):
        """Estimate real-world size from bounding box"""
        x1, y1, x2, y2 = bbox
        pixel_width = x2 - x1
        pixel_height = y2 - y1
        
        if self.camera_matrix is not None:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            
            # Convert pixel size to real-world size (meters)
            real_width = (pixel_width * distance) / fx
            real_height = (pixel_height * distance) / fy
        else:
            # Rough estimation without camera matrix
            real_width = pixel_width * 0.001 * distance
            real_height = pixel_height * 0.001 * distance
        
        return real_width, real_height
    
    def classify_size(self, width, height):
        """Classify object as small, medium, or large based on dimensions"""
        max_dim = max(width, height)
        
        # Adjust size classification based on distance from camera
        # At home position (0.5m), objects appear larger than when robot moves closer
        if self.object_distance <= 0.6:  # Close to home position
            if max_dim < 0.06:  # Less than 6cm at 0.5m
                return 'small'
            elif max_dim < 0.12:  # Less than 12cm at 0.5m
                return 'medium'
            else:
                return 'large'
        else:  # Farther away
            if max_dim < 0.08:  # Less than 8cm
                return 'small'
            elif max_dim < 0.15:  # Less than 15cm
                return 'medium'
            else:
                return 'large'
    
    def draw_grid_overlay(self, image):
        """Draw a grid overlay on the image to visualize workspace areas"""
        height, width = image.shape[:2]
        
        # Grid parameters
        grid_color = (200, 200, 200)  # Light gray
        grid_thickness = 1
        
        # Draw vertical lines (every 50 pixels)
        for x in range(0, width, 50):
            cv2.line(image, (x, 0), (x, height), grid_color, grid_thickness)
        
        # Draw horizontal lines (every 50 pixels)
        for y in range(0, height, 50):
            cv2.line(image, (0, y), (width, y), grid_color, grid_thickness)
        
        # Draw thicker lines at major intervals (every 100 pixels)
        major_color = (150, 150, 150)  # Darker gray
        major_thickness = 2
        for x in range(0, width, 100):
            cv2.line(image, (x, 0), (x, height), major_color, major_thickness)
        for y in range(0, height, 100):
            cv2.line(image, (0, y), (width, y), major_color, major_thickness)
        
        # Add coordinate labels at corners
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_color = (255, 255, 255)
        font_thickness = 1
        
        # Top-left
        cv2.putText(image, "(0,0)", (5, 15), font, font_scale, font_color, font_thickness)
        # Top-right
        cv2.putText(image, f"({width},0)", (width - 60, 15), font, font_scale, font_color, font_thickness)
        # Bottom-left
        cv2.putText(image, f"(0,{height})", (5, height - 5), font, font_scale, font_color, font_thickness)
        # Bottom-right
        cv2.putText(image, f"({width},{height})", (width - 80, height - 5), font, font_scale, font_color, font_thickness)
    
    def draw_calibration_points(self, image):
        """Draw calibration points on the image for verification"""
        for i, pt in enumerate(self.CALIB_IMG_PTS_PX_FULL):
            cv2.circle(image, (int(pt[0]), int(pt[1])), 8, (0, 255, 0), -1)
            cv2.putText(image, f'P{i+1}', (int(pt[0])+10, int(pt[1])-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Also draw robot coordinates as text
        for i, (img_pt, robot_pt) in enumerate(zip(self.CALIB_IMG_PTS_PX_FULL, self.CALIB_ROBOT_PTS_MM)):
            text = f'P{i+1}: ({robot_pt[0]:.0f}, {robot_pt[1]:.0f})mm'
            cv2.putText(image, text, (10, 30 + i*20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    def image_callback(self, msg):
        """Process incoming camera images with YOLO detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Always draw grid overlay for workspace visualization
            self.draw_grid_overlay(cv_image)
            
            # Draw calibration points if in calibration mode
            if self.calibration_mode:
                self.draw_calibration_points(cv_image)
            
            # Run color detection
            inst_map = self.detect_instances(cv_image)
            
            # Process detections
            detections = []
            annotated_image = cv_image.copy()
            
            for color, instances in inst_map.items():
                for idx, inst in enumerate(instances, start=1):
                    cx, cy = inst["center"]
                    box = inst["box"]
                    area = inst["area"]
                    ang = inst["angle"]
                    
                    # Get bbox from box
                    x1 = np.min(box[:, 0])
                    y1 = np.min(box[:, 1])
                    x2 = np.max(box[:, 0])
                    y2 = np.max(box[:, 1])
                    
                    # Classify size based on area or max dim
                    max_dim = max(x2 - x1, y2 - y1)
                    if max_dim < 50:
                        size_category = 'small'
                    elif max_dim < 80:
                        size_category = 'medium'
                    else:
                        size_category = 'large'
                    
                    shape_name = 'cube'  # Assume cube
                    conf = 0.95  # High confidence for direct detection
                    
                    # Estimate real size
                    real_width, real_height = self.estimate_real_size((x1, y1, x2, y2), self.object_distance)
                    
                    # Draw bounding box
                    cv2.drawContours(annotated_image, [box], 0, (0, 255, 0), 2)
                    
                    # Draw center point
                    cv2.circle(annotated_image, (cx, cy), 5, (0, 255, 0), -1)
                    
                    # Add label
                    label = f'{size_category} {color} {shape_name} {conf:.2f}'
                    cv2.putText(annotated_image, label, (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    detections.append({
                        'class': shape_name,
                        'confidence': float(conf),
                        'bbox': (float(x1), float(y1), float(x2), float(y2)),
                        'center': (cx, cy),
                        'pixel_size': {'width': int(x2 - x1), 'height': int(y2 - y1)},
                        'real_size': {'width': real_width, 'height': real_height},
                        'size_category': size_category,
                        'color': color,
                        'color_bgr': self.get_color_bgr(color),
                        'full_class_name': f'{size_category} {color} {shape_name}'
                    })
            
            # If no detections, draw fake ones on the image for testing
            if not detections:
                fake_boxes = [
                    (200, 150, 250, 200, 'small red cube'),
                    (300, 200, 350, 250, 'medium blue sphere')
                ]
                for x1, y1, x2, y2, label in fake_boxes:
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Before publishing annotated image, compute 3D positions for detections
            for det in detections:
                cx, cy = det['center']
                
                # Constrain positions to camera view (640x480 resolution)
                cx = max(0, min(cx, 640))  # Clamp to 0-640
                cy = max(0, min(cy, 480))  # Clamp to 0-480
                
                # Use homography calibration for consistent camera grid positioning
                robot_coords = self.pixel_to_robot_coords(cx, cy)
                if robot_coords is not None:
                    # Use homography-based positioning for consistent grid coordinates
                    X = robot_coords[0]  # X in meters (base_link frame)
                    Y = robot_coords[1]  # Y in meters (base_link frame)
                    Z = self.get_z_for_size(det['size_category'])  # Z based on object size
                    self.get_logger().debug(f'Object at pixel ({cx}, {cy}) -> robot coords ({X:.3f}, {Y:.3f})')
                elif self.camera_matrix is not None:
                    # Fallback to camera intrinsics if homography not available
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    cx_cam = self.camera_matrix[0, 2]
                    cy_cam = self.camera_matrix[1, 2]

                    # Calculate distance based on object size
                    if det['size_category'] == 'small':
                        distance = 0.15
                    elif det['size_category'] == 'medium':
                        distance = 0.25
                    else:  # large
                        distance = 0.35

                    X = (cx - cx_cam) * distance / fx
                    Y = (cy - cy_cam) * distance / fy
                    Z = self.get_z_for_size(det['size_category'])
                    self.get_logger().debug(f'Using camera intrinsics fallback: ({X:.3f}, {Y:.3f}, {Z:.3f})')
                else:
                    # Final fallback
                    X, Y, Z = 0.0, 0.0, self.get_z_for_size(det['size_category'])
                    self.get_logger().warn('No calibration available, using origin position')

                det['position'] = {'x': float(X), 'y': float(Y), 'z': float(Z)}

            # Publish detection image based on parameter and detection status
            if self.publish_annotated:
                if detections:
                    # Publish annotated image when objects are detected
                    detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                    detection_msg.header = msg.header
                    self.detection_img_pub.publish(detection_msg)
                else:
                    # Publish original image when no objects detected (less frequently to reduce flashing)
                    if hasattr(self, '_last_publish_time'):
                        current_time = self.get_clock().now().nanoseconds / 1e9
                        if current_time - self._last_publish_time > 1.0:  # Publish every 1 second when no detections
                            detection_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                            detection_msg.header = msg.header
                            self.detection_img_pub.publish(detection_msg)
                            self._last_publish_time = current_time
                    else:
                        # First time, publish original image
                        detection_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                        detection_msg.header = msg.header
                        self.detection_img_pub.publish(detection_msg)
                        self._last_publish_time = self.get_clock().now().nanoseconds / 1e9
            else:
                # When annotated images are disabled, still publish original image at reduced rate to prevent flashing
                if hasattr(self, '_last_publish_time'):
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - self._last_publish_time > 1.0:  # Publish every 1 second
                        detection_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                        detection_msg.header = msg.header
                        self.detection_img_pub.publish(detection_msg)
                        self._last_publish_time = current_time
                else:
                    # First time, publish original image
                    detection_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    detection_msg.header = msg.header
                    self.detection_img_pub.publish(detection_msg)
                    self._last_publish_time = self.get_clock().now().nanoseconds / 1e9
            
            # Publish markers and target pose
            if detections:
                self.publish_markers(detections, msg.header)
                self.publish_target_pose(detections, msg.header)
                # Publish detection strings for ALL detected objects (not just target class)
                for det in detections:
                    pos = det.get('position', {'x': 0.0, 'y': 0.0, 'z': self.object_distance})
                    det_str = (f"{det['size_category']} {det['color']} {det['class']}: conf={det['confidence']:.2f}, "
                               f"size={det['real_size']['width']*100:.1f}x{det['real_size']['height']*100:.1f}cm, "
                               f"pos={pos['x']:.3f},{pos['y']:.3f},{pos['z']:.3f}")
                    self.detection_string_pub.publish(String(data=det_str))
                    self.get_logger().info(f"Published detection: {det_str}")
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def get_color_bgr(self, color_name):
        """Convert color name to BGR values for visualization"""
        color_map = {
            'red': (50, 50, 255),
            'green': (50, 255, 50),
            'blue': (255, 50, 50),
            'yellow': (50, 255, 255),
            'orange': (0, 140, 255),
            'purple': (240, 50, 180),
            'pink': (203, 192, 255),
            'white': (240, 240, 240),
            'black': (30, 30, 30),
            'cyan': (255, 255, 50),
            'gray': (128, 128, 128),
            'magenta': (255, 50, 255),
            'unknown': (128, 128, 128)
        }
        return color_map.get(color_name.lower(), (128, 128, 128))
    
    def get_color_rgb(self, color_name):
        """Convert color name to RGB values for RViz markers"""
        b, g, r = self.get_color_bgr(color_name)
        return r, g, b
    
    def get_marker_size(self, size_category):
        """Get marker size based on object size category"""
        size_map = {
            'small': 0.015,   # 1.5cm - much smaller
            'medium': 0.025,  # 2.5cm - smaller
            'large': 0.035    # 3.5cm - smaller
        }
        return size_map.get(size_category, 0.02)
    
    def get_z_for_size(self, size_category):
        """Get Z offset based on object size for stacking"""
        if size_category == 'small':
            return 0.0
        elif size_category == 'medium':
            return 0.05
        elif size_category == 'large':
            return 0.1
        return 0.0
    
    def publish_markers(self, detections, header):
        """Publish RViz markers for detected objects"""
        marker_array = MarkerArray()
        
        for i, det in enumerate(detections):
            marker = Marker()
            # Publish markers in camera_link frame to show under camera
            marker.header.frame_id = 'camera_link'
            marker.header.stamp = header.stamp
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Get position - for camera view, use pixel-based positioning with z under camera
            cx, cy = det['center']
            if self.camera_matrix is not None:
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx_cam = self.camera_matrix[0, 2]
                cy_cam = self.camera_matrix[1, 2]
                
                # Distance to table (adjust as needed)
                distance = 0.5  # meters
                
                X = (cx - cx_cam) * distance / fx
                Y = (cy - cy_cam) * distance / fy
                Z = 0.05  # On table in front of camera
            else:
                # Fallback: simple pixel scaling
                X = (cx - 320) * 0.001
                Y = (cy - 240) * 0.001
                Z = 0.05
            
            marker.pose.position.x = X
            marker.pose.position.y = Y
            marker.pose.position.z = Z
            marker.pose.orientation.w = 1.0
            
            # Marker size based on object size category
            marker_size = self.get_marker_size(det['size_category'])
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = marker_size
            
            # Color based on YOLO detected color
            r, g, b = self.get_color_rgb(det['color'])
            marker.color.r = float(r) / 255.0
            marker.color.g = float(g) / 255.0
            marker.color.b = float(b) / 255.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
            
            # Add text label with color and size info
            text_marker = Marker()
            text_marker.header.frame_id = 'camera_link'
            text_marker.header.stamp = header.stamp
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = X
            text_marker.pose.position.y = Y
            text_marker.pose.position.z = Z + 0.02  # Small offset above object for visibility
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.03
            text_marker.color = marker.color  # Same color as the object
            text_marker.text = f"{det['size_category']} {det['color']} {det['class']}"
            text_marker.lifetime.sec = 1
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_target_pose(self, detections, header):
        """Publish target poses for all detected objects in base_link frame"""
        for i, det in enumerate(detections):
            # Get position from detection (already in base_link frame via homography)
            pos = det['position']
            
            # Create and publish target pose in base_footprint frame
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'base_footprint'
            pose_msg.header.stamp = header.stamp
            pose_msg.pose.position.x = pos['x']
            pose_msg.pose.position.y = pos['y']
            pose_msg.pose.position.z = pos['z']
            pose_msg.pose.orientation.w = 1.0
            
            # For now, still publish to the main topic for the highest confidence detection
            if i == 0:  # Publish highest confidence to main topic
                self.target_pose_pub.publish(pose_msg)
                self.get_logger().info(f'Target detected: {det["size_category"]} {det["color"]} {det["class"]} at ({pos["x"]:.3f}, {pos["y"]:.3f}, {pos["z"]:.3f}) in base_footprint frame')

    def publish_fake_detection(self):
        """Publish fake detections for testing when no real camera"""
        # Create fake detection data
        fake_dets = [
            {
                'class': 'cube',
                'confidence': 0.95,
                'bbox': (200, 150, 250, 200),
                'center': (225, 175),
                'pixel_size': {'width': 50, 'height': 50},
                'real_size': {'width': 0.05, 'height': 0.05},
                'size_category': 'small',
                'color': 'red',
                'color_bgr': (50, 50, 255),
                'full_class_name': 'small_red_cube',
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.05}
            },
            {
                'class': 'sphere',
                'confidence': 0.90,
                'bbox': (300, 200, 350, 250),
                'center': (325, 225),
                'pixel_size': {'width': 50, 'height': 50},
                'real_size': {'width': 0.05, 'height': 0.05},
                'size_category': 'medium',
                'color': 'blue',
                'color_bgr': (255, 50, 50),
                'full_class_name': 'medium_blue_sphere',
                'position': {'x': 0.1, 'y': 0.0, 'z': 0.05}
            }
        ]
        
        # Publish markers
        marker_array = MarkerArray()
        for i, det in enumerate(fake_dets):
            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            pos = det['position']
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = pos['z']
            marker.pose.orientation.w = 1.0
            marker_size = self.get_marker_size(det['size_category'])
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = marker_size
            r, g, b = self.get_color_rgb(det['color'])
            marker.color.r = float(r) / 255.0
            marker.color.g = float(g) / 255.0
            marker.color.b = float(b) / 255.0
            marker.color.a = 0.8
            marker.lifetime.sec = 5  # Disappear after 5 seconds
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z += 0.05
            text_marker.scale.z = 0.03
            text_marker.color = marker.color
            text_marker.text = f"{det['size_category']} {det['color']} {det['class']}"
            text_marker.lifetime.sec = 5
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        
        # Publish detection strings
        for det in fake_dets:
            pos = det.get('position', {'x': 0.0, 'y': 0.0, 'z': self.object_distance})
            det_str = (f"{det['size_category']} {det['color']} {det['class']}: conf={det['confidence']:.2f}, "
                       f"size={det['real_size']['width']*100:.1f}x{det['real_size']['height']*100:.1f}cm, "
                       f"pos={pos['x']:.3f},{pos['y']:.3f},{pos['z']:.3f}")
            self.detection_string_pub.publish(String(data=det_str))
            self.get_logger().info(f"Published fake detection: {det_str}")
        
        # Publish target pose for first detection
        if fake_dets:
            det = fake_dets[0]
            pos = det['position']
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = pos['x']
            pose_msg.pose.position.y = pos['y']
            pose_msg.pose.position.z = pos['z']
            pose_msg.pose.orientation.w = 1.0
            self.target_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
