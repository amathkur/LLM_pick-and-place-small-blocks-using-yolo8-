#!/usr/bin/env python3
"""
Generate Box Training Data for YOLO
Adds black box samples to the existing dataset
"""

import cv2
import numpy as np
from pathlib import Path
import random
import os

def generate_box_images():
    """Generate synthetic black box images for training"""
    # Dataset paths
    data_dir = Path("/home/abdulhamid/dobot_rviz_ws/dobot_llm_project/data/yolo_shapes_training")
    images_dir = data_dir / "images"
    labels_dir = data_dir / "labels"

    # Create directories if they don't exist
    images_dir.mkdir(exist_ok=True)
    labels_dir.mkdir(exist_ok=True)

    # Box classes to add
    box_classes = [
        "small_black_box",
        "large_black_box"
    ]

    # Add to dataset.yaml
    dataset_yaml = data_dir / "dataset.yaml"
    if dataset_yaml.exists():
        with open(dataset_yaml, 'r') as f:
            content = f.read()

        # Add box classes if not already present
        if "small_black_box" not in content:
            # Find the nc: line and increment it
            lines = content.split('\n')
            for i, line in enumerate(lines):
                if line.startswith('nc:'):
                    nc = int(line.split(':')[1].strip())
                    lines[i] = f'nc: {nc + 2}'
                    break

            # Add box names
            names_section = False
            for i, line in enumerate(lines):
                if line.startswith('names:'):
                    names_section = True
                    continue
                if names_section and line.strip() == '':
                    # Insert box names before the empty line
                    lines.insert(i, '- small_black_box')
                    lines.insert(i+1, '- large_black_box')
                    break

            # Write back
            with open(dataset_yaml, 'w') as f:
                f.write('\n'.join(lines))

    # Generate images for each class
    for class_name in box_classes:
        print(f"Generating images for {class_name}")

        # Determine size
        if "small" in class_name:
            box_size = (80, 80)  # Small box
        else:
            box_size = (160, 160)  # Large box

        # Generate 20 samples per class
        for i in range(20):
            # Create blank image
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            img.fill(200)  # Light gray background

            # Random position for box
            max_x = 640 - box_size[0]
            max_y = 480 - box_size[1]
            x = random.randint(50, max_x - 50)
            y = random.randint(50, max_y - 50)

            # Draw black box
            cv2.rectangle(img, (x, y), (x + box_size[0], y + box_size[1]), (0, 0, 0), -1)

            # Add some noise/variation
            # Add slight color variation to make it more realistic
            noise = np.random.normal(0, 5, img.shape).astype(np.uint8)
            img = cv2.add(img, noise)

            # Ensure box area stays black
            cv2.rectangle(img, (x, y), (x + box_size[0], y + box_size[1]), (0, 0, 0), -1)

            # Save image
            img_filename = f"{class_name}_{i:03d}.jpg"
            img_path = images_dir / img_filename
            cv2.imwrite(str(img_path), img)

            # Create YOLO label file
            # YOLO format: class_id center_x center_y width height (normalized)
            img_height, img_width = img.shape[:2]

            center_x = (x + box_size[0]/2) / img_width
            center_y = (y + box_size[1]/2) / img_height
            width = box_size[0] / img_width
            height = box_size[1] / img_height

            # Get class ID (add to existing classes)
            if class_name == "small_black_box":
                class_id = 80  # Next available ID
            else:  # large_black_box
                class_id = 81

            label_content = f"{class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}"

            label_filename = f"{class_name}_{i:03d}.txt"
            label_path = labels_dir / label_filename
            with open(label_path, 'w') as f:
                f.write(label_content)

    print("Box training data generation complete!")

if __name__ == "__main__":
    generate_box_images()