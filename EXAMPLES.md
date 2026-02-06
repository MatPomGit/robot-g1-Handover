# 💻 Przykłady Kodu (Code Examples)

Ten dokument zawiera gotowe do użycia przykłady kodu dla różnych scenariuszy.

---

## 📋 Spis treści

1. [Podstawowe przykłady](#podstawowe-przykłady)
2. [Percepcja](#percepcja)
3. [Manipulacja](#manipulacja)
4. [Integracja](#integracja)
5. [Debugowanie](#debugowanie)

---

## Podstawowe przykłady

### Hello World - Prosty node ROS 2

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world')
        self.get_logger().info('🚀 Hello from Robot G1!')
        
        # Timer - wywołuje callback co 1 sekundę
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'⏰ Tick {self.counter}')

def main():
    rclpy.init()
    node = HelloWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Uruchomienie:**
```bash
python3 hello_world.py
```

---

## Percepcja

### Przykład 1: Subskrypcja obrazu z kamery

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Subskrypcja topiku kamery
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('📷 Camera viewer started. Press Ctrl+C to stop.')
    
    def image_callback(self, msg):
        """Odbiera obraz i wyświetla w oknie"""
        try:
            # Konwersja ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Wyświetl obraz
            cv2.imshow('Camera View', cv_image)
            cv2.waitKey(1)
            
            self.get_logger().info(
                f'📸 Received image: {cv_image.shape[1]}x{cv_image.shape[0]}',
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Przykład 2: Śledzenie wykrytych obiektów

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        self.sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detections_callback,
            10
        )
        
        self.object_counts = {}
        self.get_logger().info('🎯 Object tracker started')
    
    def detections_callback(self, msg):
        """Zlicza wykryte obiekty"""
        if len(msg.detections) == 0:
            self.get_logger().debug('⚪ No objects detected')
            return
        
        # Zlicz obiekty po klasach
        current_objects = {}
        for det in msg.detections:
            class_id = det.results[0].id
            confidence = det.results[0].score
            
            if class_id not in current_objects:
                current_objects[class_id] = 0
            current_objects[class_id] += 1
            
            self.get_logger().debug(
                f'  📦 Class {class_id}: confidence={confidence:.2f}'
            )
        
        # Wyświetl podsumowanie
        self.get_logger().info(
            f'🎯 Detected: {len(msg.detections)} objects - {current_objects}'
        )

def main():
    rclpy.init()
    node = ObjectTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Manipulacja

### Przykład 3: Prosta sekwencja ruchu

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface

class SimpleMotion(Node):
    def __init__(self):
        super().__init__('simple_motion')
        
        # Inicjalizacja MoveIt
        self.get_logger().info('🦾 Initializing MoveIt...')
        self.moveit = MoveItInterface()
        self.get_logger().info('✅ MoveIt ready')
        
        # Wykonaj sekwencję
        self.execute_sequence()
    
    def execute_sequence(self):
        """Wykonuje prostą sekwencję ruchów"""
        
        # 1. Home position
        self.get_logger().info('🏠 Moving to home position...')
        success = self.moveit.arm.go([0, 0, 0, 0, 0, 0], wait=True)
        if success:
            self.get_logger().info('✅ Home position reached')
        else:
            self.get_logger().error('❌ Failed to reach home')
            return
        
        # 2. Open gripper
        self.get_logger().info('✋ Opening gripper...')
        self.moveit.open_gripper()
        
        # 3. Move to target
        self.get_logger().info('🎯 Moving to target position...')
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = 0.5
        target.pose.position.y = 0.0
        target.pose.position.z = 0.5
        target.pose.orientation.w = 1.0
        
        success = self.moveit.move_to_pose(target)
        if success:
            self.get_logger().info('✅ Target reached')
        else:
            self.get_logger().error('❌ Failed to reach target')
            return
        
        # 4. Close gripper
        self.get_logger().info('✊ Closing gripper...')
        self.moveit.close_gripper()
        
        # 5. Return home
        self.get_logger().info('🏠 Returning home...')
        self.moveit.arm.go([0, 0, 0, 0, 0, 0], wait=True)
        
        self.get_logger().info('🎉 Sequence completed!')

def main():
    rclpy.init()
    node = SimpleMotion()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Przykład 4: Bezpieczne chwytanie z walidacją

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface
from manipulation.grasp_planner import compute_pregrasp_pose
import math

class SafeGrasp(Node):
    def __init__(self):
        super().__init__('safe_grasp')
        self.moveit = MoveItInterface()
    
    def validate_target(self, pose: PoseStamped) -> bool:
        """Sprawdza czy pozycja jest bezpieczna"""
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        
        # Sprawdź odległość od bazy
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        if distance < 0.3:
            self.get_logger().error(f'❌ Too close: {distance:.2f}m < 0.3m')
            return False
        
        if distance > 0.8:
            self.get_logger().error(f'❌ Too far: {distance:.2f}m > 0.8m')
            return False
        
        if z < 0.2:
            self.get_logger().error(f'❌ Too low: {z:.2f}m < 0.2m (collision risk)')
            return False
        
        self.get_logger().info(f'✅ Target safe: distance={distance:.2f}m')
        return True
    
    def safe_grasp(self, target_pose: PoseStamped) -> bool:
        """Bezpieczne chwytanie z walidacją"""
        
        # 1. Walidacja
        if not self.validate_target(target_pose):
            return False
        
        # 2. Oblicz pre-grasp
        pregrasp = compute_pregrasp_pose(target_pose)
        
        # 3. Otwórz gripper
        self.get_logger().info('✋ Opening gripper...')
        self.moveit.open_gripper()
        
        # 4. Podjedź do pre-grasp
        self.get_logger().info('🦾 Moving to pre-grasp...')
        if not self.moveit.move_to_pose(pregrasp):
            self.get_logger().error('❌ Failed to reach pre-grasp')
            return False
        
        # 5. Podjedź do celu
        self.get_logger().info('🎯 Moving to grasp...')
        if not self.moveit.move_to_pose(target_pose):
            self.get_logger().error('❌ Failed to reach grasp')
            return False
        
        # 6. Zamknij gripper
        self.get_logger().info('✊ Closing gripper...')
        self.moveit.close_gripper()
        
        # 7. Podnieś
        lift_pose = PoseStamped()
        lift_pose.header = target_pose.header
        lift_pose.pose = target_pose.pose
        lift_pose.pose.position.z += 0.15  # +15cm
        
        self.get_logger().info('⬆️ Lifting...')
        if not self.moveit.move_to_pose(lift_pose):
            self.get_logger().error('❌ Failed to lift')
            return False
        
        self.get_logger().info('✅ Grasp successful!')
        return True

def main():
    rclpy.init()
    node = SafeGrasp()
    
    # Przykład użycia
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x = 0.5
    target.pose.position.y = 0.0
    target.pose.position.z = 0.4
    target.pose.orientation.w = 1.0
    
    node.safe_grasp(target)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Integracja

### Przykład 5: Kompletny pipeline perception → decision → action

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface

class IntegratedSystem(Node):
    def __init__(self):
        super().__init__('integrated_system')
        
        # Subscribers
        self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.pose_callback,
            10
        )
        
        # State
        self.object_detected = False
        self.object_pose = None
        self.moveit = MoveItInterface()
        
        # Timer - sprawdza stan co sekundę
        self.timer = self.create_timer(1.0, self.state_machine)
        
        self.get_logger().info('🤖 Integrated system started')
    
    def detection_callback(self, msg):
        """Odbiera detekcje obiektów"""
        self.object_detected = (len(msg.detections) > 0)
        
        if self.object_detected:
            self.get_logger().info(
                f'👁️ Object detected: {len(msg.detections)} item(s)',
                throttle_duration_sec=2.0
            )
    
    def pose_callback(self, msg):
        """Odbiera pozycję obiektu"""
        self.object_pose = msg
        self.get_logger().debug(
            f'📍 Object at: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )
    
    def state_machine(self):
        """Prosty automat stanów"""
        
        if self.object_detected and self.object_pose is not None:
            self.get_logger().info('🎯 Object ready - initiating grasp')
            self.execute_grasp()
            
            # Reset po wykonaniu
            self.object_detected = False
            self.object_pose = None
    
    def execute_grasp(self):
        """Wykonuje chwytanie"""
        self.get_logger().info('🤝 Starting grasp sequence...')
        
        # Tu wstaw swoją logikę chwytania
        success = self.moveit.move_to_pose(self.object_pose)
        
        if success:
            self.moveit.close_gripper()
            self.get_logger().info('✅ Grasp successful')
        else:
            self.get_logger().error('❌ Grasp failed')

def main():
    rclpy.init()
    node = IntegratedSystem()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Debugowanie

### Przykład 6: Logger z różnymi poziomami

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        
        # Różne poziomy logowania
        self.get_logger().debug('🔍 Debug: Szczegółowe informacje')
        self.get_logger().info('ℹ️ Info: Ogólne informacje')
        self.get_logger().warn('⚠️ Warning: Ostrzeżenie')
        self.get_logger().error('❌ Error: Błąd')
        
        # Throttled logging (ograniczona częstotliwość)
        self.timer = self.create_timer(0.1, self.frequent_callback)
    
    def frequent_callback(self):
        # Loguj max raz na 2 sekundy
        self.get_logger().info(
            '⏰ Frequent message',
            throttle_duration_sec=2.0
        )

def main():
    rclpy.init()
    node = DebugNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Uruchomienie z verbose logging:**
```bash
ros2 run your_package debug_node --ros-args --log-level debug
```

---

## 🎯 Gotowe do użycia skrypty

Wszystkie powyższe przykłady możesz zapisać jako pliki `.py` i uruchomić bezpośrednio:

```bash
chmod +x example.py
python3 example.py
```

Lub przez ROS 2:
```bash
ros2 run g1_pick_and_handover example_node
```

---

## 💡 Wskazówki

1. **Zawsze używaj try-except** w callbackach
2. **Throttle** częste logi (`throttle_duration_sec`)
3. **Waliduj** dane wejściowe przed użyciem
4. **Testuj** każdy moduł osobno przed integracją
5. **Loguj** z kontekstem (emoji + opis akcji)

---

**Potrzebujesz więcej przykładów?** Zobacz [TUTORIALS.md](TUTORIALS.md) lub otwórz Issue!
