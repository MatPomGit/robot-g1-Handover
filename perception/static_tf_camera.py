"""
Moduł statycznej transformacji kamery (Static Camera TF)

Ten node ROS 2 publikuje statyczną transformację (TF) między bazą robota a kamerą.
Transformacja opisuje, gdzie kamera jest zamontowana względem robota.

KROK PO KROKU:
1. Definiuje pozycję i orientację kamery względem bazy robota
2. Publikuje transformację jako StaticTransform
3. Umożliwia systemowi TF2 konwersję współrzędnych między ramkami

SYSTEM TF2 (Transform):
TF2 to system w ROS 2, który zarządza drzewem transformacji między różnymi
układami współrzędnych (frames). Pozwala automatycznie konwertować pozycje
między różnymi ramkami (np. camera_link -> base_link).

Przykład drzewa TF:
    world
      └── base_link (baza robota)
            ├── camera_link (kamera)
            └── arm_link (ramię)

TOPIKI ROS 2:
- Publikuje: /tf_static (tf2_msgs/TFMessage) - statyczne transformacje
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticCameraTF(Node):
    """
    Node ROS 2 publikujący statyczną transformację kamery
    
    Definiuje, gdzie kamera jest zamontowana względem bazy robota.
    Ta informacja jest potrzebna do konwersji wykrytych pozycji obiektów
    z układu kamery do układu robota.
    """

    def __init__(self):
        """
        Inicjalizacja node'a StaticCameraTF
        
        KROK PO KROKU:
        1. Tworzy StaticTransformBroadcaster
        2. Definiuje transformację base_link -> camera_link
        3. Publikuje transformację (jednorazowo, bo statyczna)
        
        KONWENCJE ROS:
        - base_link = główny układ współrzędnych robota (środek podstawy)
        - camera_link = układ współrzędnych kamery
        
        UKŁAD WSPÓŁRZĘDNYCH (REP 103):
        - Oś X wskazuje do przodu
        - Oś Y wskazuje w lewo
        - Oś Z wskazuje do góry
        """
        super().__init__("static_camera_tf")
        
        # StaticTransformBroadcaster służy do publikowania statycznych transformacji
        # Statyczne = nie zmieniają się w czasie (kamera jest na stałe zamontowana)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Tworzymy wiadomość TransformStamped opisującą transformację
        t = TransformStamped()

        # PARENT FRAME: base_link (baza robota)
        # To jest ramka "rodzica" - punkt odniesienia
        t.header.frame_id = "base_link"
        
        # CHILD FRAME: camera_link (kamera)
        # To jest ramka "dziecka" - transformowana względem rodzica
        t.child_frame_id = "camera_link"
        
        # TRANSLACJA (pozycja kamery względem bazy robota w metrach)
        # Kamera jest zamontowana:
        t.transform.translation.x = 0.25  # 25 cm przed bazą (do przodu)
        t.transform.translation.y = 0.0   # Na środku (lewo-prawo)
        t.transform.translation.z = 0.90  # 90 cm nad ziemią (do góry)
        
        # ROTACJA (orientacja kamery jako quaternion)
        # Quaternion [x, y, z, w] = [0, 0, 0, 1] oznacza brak obrotu
        # Kamera patrzy w tym samym kierunku co robot (do przodu)
        t.transform.rotation.w = 1.0
        
        # UWAGA: Jeśli kamera byłaby obrócona (np. skierowana w dół):
        # - Należałoby użyć odpowiedniego quaterniona
        # - Narzędzie: tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # - Przykład: patrzy 30° w dół = euler(0, -30°, 0)

        # Publikujemy transformację
        # Będzie ona dostępna w systemie TF2 dla wszystkich node'ów
        self.broadcaster.sendTransform(t)
        
        # INFO: StaticTransform publikuje się tylko raz przy starcie
        # W przeciwieństwie do dynamicznych transformacji (np. ruchome stawy robota)

def main():
    """
    Funkcja główna uruchamiająca node StaticCameraTF
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a StaticCameraTF
    3. Publikuje transformację (jednorazowo)
    4. Utrzymuje node aktywny (spin)
    5. Po zakończeniu (Ctrl+C) czyści zasoby
    
    Użycie:
        ros2 run g1_pick_and_handover static_tf_camera
    
    Sprawdzenie czy działa:
        # Wyświetl drzewo TF
        ros2 run tf2_tools view_frames
        
        # Sprawdź transformację
        ros2 run tf2_ros tf2_echo base_link camera_link
    
    UWAGA: Ten node musi być uruchomiony przed innymi node'ami,
    które potrzebują transformacji camera_link -> base_link
    (np. pose_estimator_6d)
    """
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Utworzenie i uruchomienie node'a
    # spin() utrzymuje node aktywny, choć już opublikował transformację
    # (StaticTransform trzeba okresowo re-publikować dla nowych node'ów)
    rclpy.spin(StaticCameraTF())
    
    # Czyszczenie po zakończeniu
    rclpy.shutdown()
