#!/usr/bin/env python3
import cv2
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper
from solution.integration_activity import (
    NUMBER_FRAMES_SKIPPED, 
    filter_by_classes, 
    filter_by_scores, 
    filter_by_bboxes,
    get_detection_priority
)

class ObjectDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self.initialized = False
        self.frame_id = 0
        
        self.veh = os.environ['VEHICLE_NAME']
        self.pub_vel = rospy.Publisher(f"/{self.veh}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.pub_detections_image = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)
        
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.bridge = CvBridge()
        
        # Get parameters
        self.AIDO_eval = rospy.get_param("~AIDO_eval", False)
        self.use_enhanced_filtering = rospy.get_param("~use_enhanced_filtering", True)
        self.stop_distance_threshold = rospy.get_param("~stop_distance_threshold", 1.5)  # meters
        
        # Enhanced detection parameters
        self.center_region_left = rospy.get_param("~center_region_left", 0.33)
        self.center_region_right = rospy.get_param("~center_region_right", 0.75)
        self.duckie_area_threshold = rospy.get_param("~duckie_area_threshold", 2000)
        self.duckiebot_area_threshold = rospy.get_param("~duckiebot_area_threshold", 10000)
        self.min_confidence = rospy.get_param("~min_confidence", 0.7)
        
        # Movement parameters
        self.default_forward_speed = rospy.get_param("~default_forward_speed", 0.2)
        self.default_angular_speed = rospy.get_param("~default_angular_speed", 0.0)
        
        try:
            self.model_wrapper = Wrapper(self.AIDO_eval)
            rospy.loginfo("Object detection model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load object detection model: {e}")
            self.model_wrapper = None
        
        self.initialized = True
        self.log("Object Detection Node initialized successfully!")

    def image_cb(self, image_msg):
        if not self.initialized or self.model_wrapper is None:
            return

        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())
        if self.frame_id != 0:
            return

        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr(f"Could not decode image: {e}")
            return

        rgb = bgr[..., ::-1]
        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))

        # Get predictions from model
        bboxes, classes, scores = self.model_wrapper.predict(rgb)
        
        # Apply filtering if enabled
        if self.use_enhanced_filtering:
            filtered_detections = self.filter_detections(bboxes, classes, scores)
        else:
            # Use legacy filtering
            filtered_detections = list(zip(bboxes, classes, scores))
        
        # Process detections for navigation decisions
        stop_signal, detection_info = self.process_detections_for_navigation(filtered_detections)
        
        # Create and publish velocity command
        vel_cmd = self.create_velocity_command(stop_signal, detection_info)
        self.pub_vel.publish(vel_cmd)

        # Visualize detections
        self.visualize_detections(rgb, filtered_detections)

    def filter_detections(self, bboxes, classes, scores):
        """Apply enhanced filtering to detections"""
        filtered_detections = []
        
        for bbox, cls, score in zip(bboxes, classes, scores):
            # Convert to integer tuple for filtering functions
            bbox_int = tuple(map(int, bbox))
            
            # Apply all filters
            if (filter_by_classes(int(cls)) and 
                filter_by_scores(float(score)) and 
                filter_by_bboxes(bbox_int)):
                
                filtered_detections.append((bbox, cls, score))
        
        # Sort by priority if we have detections
        if filtered_detections:
            detection_priorities = []
            for bbox, cls, score in filtered_detections:
                bbox_int = tuple(map(int, bbox))
                priority = get_detection_priority(int(cls), float(score), bbox_int)
                detection_priorities.append((bbox, cls, score, priority))
            
            # Sort by priority (highest first)
            detection_priorities.sort(key=lambda x: x[3], reverse=True)
            filtered_detections = [(bbox, cls, score) for bbox, cls, score, _ in detection_priorities]
        
        return filtered_detections

    def process_detections_for_navigation(self, detections):
        """Process filtered detections to make navigation decisions"""
        stop_signal = False
        detection_info = {
            'duckies': [],
            'duckiebots': [],
            'other_vehicles': [],
            'highest_priority': None
        }
        
        # Define the center region boundaries
        left_boundary = int(IMAGE_SIZE * self.center_region_left)
        right_boundary = int(IMAGE_SIZE * self.center_region_right)
        
        for bbox, cls, score in detections:
            # Calculate center of bounding box
            center_x = (bbox[0] + bbox[2]) / 2
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            
            # Check if detection is in center region (danger zone)
            in_center = left_boundary < center_x < right_boundary
            
            detection_data = {
                'bbox': bbox,
                'class': cls,
                'score': score,
                'center_x': center_x,
                'area': area,
                'in_center': in_center
            }
            
            if cls == 0 and score > self.min_confidence:  # Duckie
                detection_info['duckies'].append(detection_data)
                if area > self.duckie_area_threshold and in_center:
                    stop_signal = True
                    self.log(f"Duckie detected in center - Area: {area:.0f}, Score: {score:.2f}")

            elif cls == 1 and score > self.min_confidence:  # Duckiebot
                detection_info['duckiebots'].append(detection_data)
                if area > self.duckiebot_area_threshold and in_center:
                    stop_signal = True
                    self.log(f"Duckiebot detected in center - Area: {area:.0f}, Score: {score:.2f}")
            
            elif cls in [2, 3] and score > self.min_confidence:  # Truck or Bus
                detection_info['other_vehicles'].append(detection_data)
                if area > self.duckie_area_threshold and in_center:  # Lower threshold for vehicles
                    stop_signal = True
                    self.log(f"Vehicle detected in center - Class: {cls}, Area: {area:.0f}, Score: {score:.2f}")
        
        # Find highest priority detection
        if detections:
            detection_info['highest_priority'] = detections[0]  # Already sorted by priority
        
        return stop_signal, detection_info

    def create_velocity_command(self, stop_signal, detection_info):
        """Create velocity command based on detection results"""
        vel_cmd = Twist2DStamped()
        vel_cmd.header.stamp = rospy.Time.now()
        
        if stop_signal:
            vel_cmd.v = 0.0
            vel_cmd.omega = 0.0
            
            # Log detailed stop reason
            reasons = []
            if detection_info['duckies']:
                reasons.append(f"{len(detection_info['duckies'])} duckie(s)")
            if detection_info['duckiebots']:
                reasons.append(f"{len(detection_info['duckiebots'])} duckiebot(s)")
            if detection_info['other_vehicles']:
                reasons.append(f"{len(detection_info['other_vehicles'])} vehicle(s)")
            
            reason_str = ", ".join(reasons)
            self.log(f"STOPPING for: {reason_str}")
        else:
            vel_cmd.v = self.default_forward_speed
            vel_cmd.omega = self.default_angular_speed
            
            # Optional: Implement avoidance steering for non-critical detections
            if detection_info['highest_priority'] is not None:
                self.log("Clear path - proceeding with caution")
            else:
                self.log("No obstacles detected - proceeding normally")
        
        return vel_cmd

    def visualize_detections(self, rgb, detections):
        """Enhanced visualization with class names and confidence scores"""
        colors = {
            0: (0, 255, 255),    # Duckie - Yellow
            1: (0, 165, 255),    # Duckiebot - Orange  
            2: (255, 0, 0),      # Truck - Red
            3: (255, 0, 255)     # Bus - Magenta
        }
        names = {
            0: "duckie", 
            1: "duckiebot", 
            2: "truck", 
            3: "bus"
        }
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        
        # Draw center region boundaries
        left_boundary = int(IMAGE_SIZE * self.center_region_left)
        right_boundary = int(IMAGE_SIZE * self.center_region_right)
        cv2.line(rgb, (left_boundary, 0), (left_boundary, IMAGE_SIZE), (255, 255, 255), 1)
        cv2.line(rgb, (right_boundary, 0), (right_boundary, IMAGE_SIZE), (255, 255, 255), 1)
        
        for bbox, cls, score in detections:
            if cls in colors:
                pt1 = tuple(map(int, bbox[:2]))
                pt2 = tuple(map(int, bbox[2:]))
                color = tuple(reversed(colors[cls]))
                name = names[cls]
                
                # Draw bounding box
                rgb = cv2.rectangle(rgb, pt1, pt2, color, thickness)
                
                # Draw label with confidence
                label = f"{name}: {score:.2f}"
                text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                text_location = (pt1[0], max(pt1[1] - 5, text_size[1]))
                
                # Draw background for text
                cv2.rectangle(rgb, 
                            (text_location[0], text_location[1] - text_size[1] - 2),
                            (text_location[0] + text_size[0], text_location[1] + 2),
                            color, -1)
                
                # Draw text
                rgb = cv2.putText(rgb, label, text_location, font, font_scale, (255, 255, 255), thickness)
            
        bgr = rgb[..., ::-1]
        obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
        self.pub_detections_image.publish(obj_det_img)

if __name__ == "__main__":
    try:
        object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object detection node interrupted")
    except Exception as e:
        rospy.logerr(f"Object detection node failed: {e}")