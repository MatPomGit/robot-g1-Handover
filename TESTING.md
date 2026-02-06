# Testing Guide dla Robot G1 Handover

## Wprowadzenie

Ten dokument opisuje strategię i implementację testów dla systemu Robot G1 Handover.

## Struktura Testów

```
tests/
├── __init__.py
├── unit/
│   ├── test_object_detector.py
│   ├── test_pose_estimator.py
│   ├── test_grasp_planner.py
│   ├── test_handover_planner.py
│   └── test_moveit_interface.py
├── integration/
│   ├── test_perception_pipeline.py
│   ├── test_manipulation_pipeline.py
│   └── test_full_handover.py
└── fixtures/
    ├── test_images/
    ├── test_poses.yaml
    └── mock_data.py
```

## Przykładowe Testy Jednostkowe

### test_grasp_planner.py

```python
"""Testy jednostkowe dla grasp_planner.py"""

import unittest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from manipulation.grasp_planner import compute_pregrasp, compute_lift_pose, validate_grasp_pose
from config.constants import PREGRASP_OFFSET_Z, LIFT_HEIGHT


class TestGraspPlanner(unittest.TestCase):
    """Test cases dla grasp planner"""
    
    def setUp(self):
        """Przygotowanie danych testowych"""
        self.object_pose = PoseStamped()
        self.object_pose.header = Header(frame_id="base_link")
        self.object_pose.pose = Pose(
            position=Point(x=0.5, y=0.0, z=0.3),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    
    def test_compute_pregrasp_valid(self):
        """Test obliczania pre-grasp dla poprawnej pozy"""
        pregrasp = compute_pregrasp(self.object_pose)
        
        # Sprawdź czy zwrócono wartość
        self.assertIsNotNone(pregrasp)
        
        # Sprawdź czy offset Z został zastosowany
        expected_z = self.object_pose.pose.position.z + PREGRASP_OFFSET_Z
        self.assertAlmostEqual(pregrasp.pose.position.z, expected_z, places=5)
        
        # Sprawdź czy X i Y pozostały bez zmian
        self.assertEqual(pregrasp.pose.position.x, self.object_pose.pose.position.x)
        self.assertEqual(pregrasp.pose.position.y, self.object_pose.pose.position.y)
    
    def test_compute_pregrasp_none_input(self):
        """Test obsługi None jako input"""
        pregrasp = compute_pregrasp(None)
        self.assertIsNone(pregrasp)
    
    def test_compute_lift_pose_valid(self):
        """Test obliczania pozycji podniesienia"""
        lift = compute_lift_pose(self.object_pose)
        
        self.assertIsNotNone(lift)
        expected_z = self.object_pose.pose.position.z + LIFT_HEIGHT
        self.assertAlmostEqual(lift.pose.position.z, expected_z, places=5)
    
    def test_validate_grasp_pose_valid(self):
        """Test walidacji poprawnej pozy"""
        is_valid = validate_grasp_pose(self.object_pose)
        self.assertTrue(is_valid)
    
    def test_validate_grasp_pose_out_of_bounds(self):
        """Test walidacji pozy poza workspace"""
        pose = PoseStamped()
        pose.pose.position.x = 2.0  # Zbyt daleko
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.5
        
        is_valid = validate_grasp_pose(pose)
        self.assertFalse(is_valid)
    
    def test_validate_grasp_pose_none(self):
        """Test walidacji None"""
        is_valid = validate_grasp_pose(None)
        self.assertFalse(is_valid)


if __name__ == '__main__':
    unittest.main()
```

### test_handover_planner.py

```python
"""Testy jednostkowe dla handover_planner.py"""

import unittest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from manipulation.handover_planner import (
    compute_handover_pose, 
    validate_handover_safety,
    compute_approach_trajectory
)
from config.constants import HANDOVER_OFFSET_Z, EMERGENCY_STOP_DISTANCE


class TestHandoverPlanner(unittest.TestCase):
    """Test cases dla handover planner"""
    
    def setUp(self):
        """Przygotowanie danych testowych"""
        self.human_hand_pose = PoseStamped()
        self.human_hand_pose.header = Header(frame_id="base_link")
        self.human_hand_pose.pose = Pose(
            position=Point(x=0.6, y=0.0, z=1.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        self.robot_pose = PoseStamped()
        self.robot_pose.header = Header(frame_id="base_link")
        self.robot_pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.5),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    
    def test_compute_handover_pose_valid(self):
        """Test obliczania pozycji handover"""
        handover = compute_handover_pose(self.human_hand_pose)
        
        self.assertIsNotNone(handover)
        expected_z = self.human_hand_pose.pose.position.z + HANDOVER_OFFSET_Z
        self.assertAlmostEqual(handover.pose.position.z, expected_z, places=5)
    
    def test_validate_handover_safety_safe_distance(self):
        """Test walidacji bezpiecznej odległości"""
        is_safe = validate_handover_safety(self.human_hand_pose, self.robot_pose)
        self.assertTrue(is_safe)
    
    def test_validate_handover_safety_too_close(self):
        """Test walidacji zbyt bliskiej odległości"""
        close_robot_pose = PoseStamped()
        close_robot_pose.pose.position.x = self.human_hand_pose.pose.position.x
        close_robot_pose.pose.position.y = self.human_hand_pose.pose.position.y
        close_robot_pose.pose.position.z = self.human_hand_pose.pose.position.z + 0.01  # 1cm
        
        is_safe = validate_handover_safety(self.human_hand_pose, close_robot_pose)
        self.assertFalse(is_safe)
    
    def test_compute_approach_trajectory(self):
        """Test obliczania trajektorii podejścia"""
        trajectory = compute_approach_trajectory(
            self.robot_pose, 
            self.human_hand_pose, 
            num_waypoints=5
        )
        
        self.assertEqual(len(trajectory), 6)  # 5 waypoints + końcowa pozycja
        
        # Pierwszy punkt to pozycja startowa
        self.assertAlmostEqual(
            trajectory[0].pose.position.x, 
            self.robot_pose.pose.position.x, 
            places=5
        )
        
        # Ostatni punkt to pozycja docelowa
        self.assertAlmostEqual(
            trajectory[-1].pose.position.x, 
            self.human_hand_pose.pose.position.x, 
            places=5
        )


if __name__ == '__main__':
    unittest.main()
```

## Uruchamianie Testów

### Pojedynczy plik testowy
```bash
python3 -m unittest tests/unit/test_grasp_planner.py
```

### Wszystkie testy jednostkowe
```bash
python3 -m unittest discover tests/unit
```

### Wszystkie testy
```bash
python3 -m unittest discover tests
```

### Z pokryciem kodu (coverage)
```bash
pip install coverage
coverage run -m unittest discover tests
coverage report
coverage html  # Generuje raport HTML
```

## Testy Integracyjne

### test_perception_pipeline.py

```python
"""Testy integracyjne dla pipeline percepcji"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import time


class TestPerceptionPipeline(unittest.TestCase):
    """Test cases dla całego pipeline percepcji"""
    
    @classmethod
    def setUpClass(cls):
        """Inicjalizacja ROS 2"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Czyszczenie ROS 2"""
        rclpy.shutdown()
    
    def test_full_perception_pipeline(self):
        """Test kompletnego pipeline: obraz -> detekcje -> poza 3D"""
        # TODO: Implementacja
        pass


if __name__ == '__main__':
    unittest.main()
```

## Mock Data

### mock_data.py

```python
"""Mock data dla testów"""

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np


def create_mock_object_pose(x=0.5, y=0.0, z=0.3):
    """Tworzy mock pozycji obiektu"""
    pose = PoseStamped()
    pose.header = Header(frame_id="base_link")
    pose.pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    return pose


def create_mock_rgb_image(width=640, height=480):
    """Tworzy mock obraz RGB"""
    return np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)


def create_mock_depth_image(width=640, height=480, distance=1.0):
    """Tworzy mock obraz głębokości"""
    # Wartości w milimetrach
    depth = np.full((height, width), distance * 1000, dtype=np.uint16)
    return depth
```

## CI/CD Integration

### GitHub Actions (.github/workflows/test.yml)

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -r requirements.txt
        pip install coverage unittest-xml-reporting
    
    - name: Run unit tests
      run: |
        python -m unittest discover tests/unit -v
    
    - name: Run integration tests
      run: |
        python -m unittest discover tests/integration -v
    
    - name: Generate coverage report
      run: |
        coverage run -m unittest discover tests
        coverage xml
        coverage report
    
    - name: Upload coverage to Codecov
      uses: codecov/codecov-action@v3
      with:
        files: ./coverage.xml
```

## Best Practices

1. **Izolacja**: Każdy test powinien być niezależny
2. **Fixture**: Używaj setUp() i tearDown() do przygotowania/czyszczenia
3. **Asercje**: Używaj odpowiednich asercji (assertEqual, assertAlmostEqual, assertIsNone)
4. **Nazewnictwo**: test_[funkcja]_[scenariusz]
5. **Pokrycie**: Cel minimum 80% code coverage
6. **Mock**: Mockuj zewnętrzne zależności (ROS topics, sieć, hardware)
7. **Dokumentacja**: Każdy test case powinien mieć docstring

## Narzędzia

- **unittest**: Built-in Python testing framework
- **pytest**: Bardziej zaawansowany framework (opcjonalnie)
- **coverage**: Analiza pokrycia kodu
- **mock**: Mockowanie obiektów i funkcji
- **parameterized**: Parametryzowane testy

## Roadmap

- [ ] Utworzyć strukturę katalogów testów
- [ ] Zaimplementować testy jednostkowe dla perception
- [ ] Zaimplementować testy jednostkowe dla manipulation
- [ ] Zaimplementować testy jednostkowe dla decision
- [ ] Zaimplementować testy integracyjne
- [ ] Skonfigurować CI/CD
- [ ] Osiągnąć 80%+ code coverage
- [ ] Dodać testy wydajnościowe (performance tests)
- [ ] Dodać testy symulacyjne (Gazebo/MuJoCo)
