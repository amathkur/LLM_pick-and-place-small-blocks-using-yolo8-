#!/usr/bin/env python3

"""
Test script for YOLOv9 Dobot integration components
Run this before the full system to verify hardware connections
"""

import cv2
import sys
import time
from glob import glob

def test_camera():
    """Test USB camera connection"""
    print("Testing camera connection...")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Camera not found")
        return False

    ret, frame = cap.read()
    if ret:
        h, w = frame.shape[:2]
        print(f"✅ Camera working: {w}x{h} resolution")
        cap.release()
        return True
    else:
        print("❌ Camera capture failed")
        cap.release()
        return False

def test_dobot_ports():
    """Test Dobot serial port detection"""
    print("Testing Dobot serial ports...")

    candidates = []
    candidates += glob('/dev/ttyACM*')
    candidates += glob('/dev/ttyUSB*')
    candidates += glob('/dev/serial/by-id/*')

    if not candidates:
        print("❌ No serial ports found")
        return False

    print(f"✅ Found {len(candidates)} serial port(s):")
    for port in candidates:
        print(f"  - {port}")
    return True

def test_pydobot_import():
    """Test pydobot library import"""
    print("Testing pydobot import...")

    try:
        import pydobot as pydobot
        print("✅ pydobot imported successfully")
        return True
    except ImportError:
        try:
            import pydobot2 as pydobot
            print("✅ pydobot2 imported successfully")
            return True
        except ImportError:
            print("❌ Neither pydobot nor pydobot2 found")
            print("   Install with: pip install pydobot")
            return False

def test_torch():
    """Test PyTorch installation"""
    print("Testing PyTorch...")

    try:
        import torch
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"✅ PyTorch working on {device}")
        if device == 'cuda':
            print(f"   GPU: {torch.cuda.get_device_name()}")
        return True
    except ImportError:
        print("❌ PyTorch not found")
        print("   Install with: pip install torch torchvision")
        return False

def test_opencv():
    """Test OpenCV installation"""
    print("Testing OpenCV...")

    try:
        import cv2
        version = cv2.__version__
        print(f"✅ OpenCV {version} working")
        return True
    except ImportError:
        print("❌ OpenCV not found")
        print("   Install with: pip install opencv-python")
        return False

def main():
    print("YOLOv9 Dobot Integration Test")
    print("=" * 40)

    tests = [
        test_pydobot_import,
        test_dobot_ports,
        test_camera,
        test_torch,
        test_opencv,
    ]

    results = []
    for test in tests:
        result = test()
        results.append(result)
        print()

    print("Test Summary:")
    print("-" * 20)
    passed = sum(results)
    total = len(results)

    for i, (test, result) in enumerate(zip(tests, results)):
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test.__name__}")

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All systems ready! You can run the YOLOv9 Dobot controller.")
    else:
        print("⚠️  Some tests failed. Please fix issues before running the controller.")
        sys.exit(1)

if __name__ == '__main__':
    main()