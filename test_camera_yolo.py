#!/usr/bin/env python3
"""
Test script to verify USB camera and YOLO detection
"""

import cv2
from ultralytics import YOLO
import os

print("="*60)
print("🔍 CAMERA + YOLO DETECTION TEST")
print("="*60)

# Test camera
print("\n1. Testing USB Camera...")
cap = cv2.VideoCapture(0)  # USB camera at /dev/video0

if not cap.isOpened():
    print("❌ ERROR: Cannot open camera /dev/video0")
    print("   Try: ls /dev/video*")
    exit(1)

ret, frame = cap.read()
if ret:
    h, w = frame.shape[:2]
    print(f"✅ Camera opened successfully: {w}x{h}")
else:
    print("❌ ERROR: Cannot read frame from camera")
    cap.release()
    exit(1)

# Test YOLO model
print("\n2. Testing YOLO Model...")
model_path = os.path.expanduser('~/dobot_rviz_ws/dobot_llm_project/models/yolo_shapes_best.pt')

if not os.path.exists(model_path):
    print(f"❌ ERROR: Model not found at {model_path}")
    cap.release()
    exit(1)

print(f"   Loading model: {model_path}")
model = YOLO(model_path)
print(f"✅ Model loaded successfully!")
print(f"   Classes: {len(model.names)} shapes trained")

# Run detection on camera frame
print("\n3. Running YOLO detection on camera frame...")
results = model(frame, conf=0.25, verbose=False)

detection_count = 0
for result in results:
    boxes = result.boxes
    for box in boxes:
        cls = int(box.cls[0].cpu().numpy())
        conf = box.conf[0].cpu().numpy()
        class_name = model.names[cls]
        detection_count += 1
        print(f"   ✅ Detected: {class_name} (confidence: {conf:.2f})")

if detection_count == 0:
    print("   ⚠️  No objects detected in current frame")
    print("   Tips:")
    print("   - Make sure objects are in camera view")
    print("   - Check lighting (good lighting helps)")
    print("   - Move objects closer to camera")
    print("   - Trained shapes: cube, cylinder, sphere, pyramid, cone, prism,")
    print("                     hexagon, star, disk, torus")
    print("   - Trained colors: red, green, blue, yellow, orange, purple, etc.")
else:
    print(f"\n✅ SUCCESS: {detection_count} object(s) detected!")

# Show live detection for 10 seconds
print("\n4. Showing live detection for 10 seconds...")
print("   Press 'q' to quit early")

import time
start_time = time.time()

while time.time() - start_time < 10:
    ret, frame = cap.read()
    if not ret:
        break
    
    results = model(frame, conf=0.25, verbose=False)
    
    # Draw detections
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            class_name = model.names[cls]
            
            # Draw box and label
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f'{class_name} {conf:.2f}'
            cv2.putText(frame, label, (int(x1), int(y1) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('YOLO Detection Test', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

print("\n" + "="*60)
print("✅ TEST COMPLETE")
print("="*60)
print("\nIf detections work here, the ROS2 system should work too!")
print("Make sure to place colored objects in front of the camera.")
