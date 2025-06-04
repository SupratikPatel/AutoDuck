from typing import Tuple


def DT_TOKEN() -> str:
    # TODO: change this to your duckietown token
    dt_token = "dt1-3nT7FDbT7NLPrXykNJmqrVWv9QTz2jy2yYjtxgZavm4rbf2-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfZgs6L7NXoCwx3VDAPtT7sovmrxwezRNWk"
    return dt_token


def MODEL_NAME() -> str:
    # Updated to use YOLOv11 nano model compatible with ultralytics
    # If you have a custom trained model, replace this with your model name
    return "yolov5n"  # Keep this for compatibility with existing trained models


def NUMBER_FRAMES_SKIPPED() -> int:
    # Optimized frame skipping for real-time performance
    # Higher values = lower processing load but less responsive detection
    return 6


def filter_by_classes(pred_class: int) -> bool:
    """
    Filter detections by object class.
    
    Class IDs in the Duckietown dataset:
        | Object    | ID    |
        | ---       | ---   |
        | Duckie    | 0     |
        | Cone      | 1     |
        | Truck     | 2     |
        | Bus       | 3     |

    Args:
        pred_class: the class of a prediction
    """
    # Focus on moving objects and important obstacles
    # Duckies (0) and other vehicles (Truck=2, Bus=3) are most important for collision avoidance
    important_classes = [0, 2, 3]  # Duckie, Truck, Bus
    return pred_class in important_classes


def filter_by_scores(score: float) -> bool:
    """
    Filter detections by confidence score.
    
    Args:
        score: the confidence score of a prediction (0.0 to 1.0)
    """
    # Only consider high-confidence detections to reduce false positives
    # This is crucial for safe autonomous navigation
    confidence_threshold = 0.6
    return score >= confidence_threshold


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """
    Filter detections by bounding box properties.
    
    Args:
        bbox: bounding box in xyxy format (leftmost x, topmost y, rightmost x, bottommost y)
    """
    left_x, top_y, right_x, bottom_y = bbox
    
    # Calculate bounding box properties
    width = right_x - left_x
    height = bottom_y - top_y
    area = width * height
    
    # Filter out very small detections (likely noise/distant objects)
    min_area = 500  # Minimum area in pixels
    if area < min_area:
        return False
    
    # Filter out very large detections (likely false positives or very close objects)
    max_area = 50000  # Maximum area in pixels
    if area > max_area:
        return False
    
    # Filter out unrealistic aspect ratios
    aspect_ratio = width / height if height > 0 else 0
    min_aspect_ratio = 0.2
    max_aspect_ratio = 5.0
    if aspect_ratio < min_aspect_ratio or aspect_ratio > max_aspect_ratio:
        return False
    
    # Filter out detections at the very edges of the image (likely partial objects)
    image_width = 640  # Assuming standard input resolution
    image_height = 480
    edge_margin = 10  # Pixels from edge
    
    if (left_x < edge_margin or top_y < edge_margin or 
        right_x > (image_width - edge_margin) or 
        bottom_y > (image_height - edge_margin)):
        return False
    
    return True


# Additional utility functions for enhanced object detection

def get_detection_priority(pred_class: int, score: float, bbox: Tuple[int, int, int, int]) -> float:
    """
    Calculate priority score for detection based on class, confidence, and position.
    Higher values indicate higher priority for collision avoidance.
    
    Returns:
        Priority score (0.0 to 1.0)
    """
    # Base priority by class
    class_priorities = {
        0: 1.0,  # Duckie - highest priority (moving obstacle)
        1: 0.6,  # Cone - medium priority (static obstacle) 
        2: 0.9,  # Truck - high priority (large moving obstacle)
        3: 0.9   # Bus - high priority (large moving obstacle)
    }
    
    base_priority = class_priorities.get(pred_class, 0.5)
    
    # Adjust by confidence
    confidence_factor = min(score, 1.0)
    
    # Adjust by size (larger objects are more dangerous)
    left_x, top_y, right_x, bottom_y = bbox
    area = (right_x - left_x) * (bottom_y - top_y)
    size_factor = min(area / 10000, 1.0)  # Normalize to 0-1
    
    # Adjust by position (center objects are more dangerous)
    center_x = (left_x + right_x) / 2
    image_center = 320  # Assuming 640px width
    distance_from_center = abs(center_x - image_center) / image_center
    position_factor = 1.0 - (distance_from_center * 0.5)  # Reduce priority for edge objects
    
    # Combine factors
    priority = base_priority * confidence_factor * (0.7 + 0.3 * size_factor) * position_factor
    
    return min(priority, 1.0)
