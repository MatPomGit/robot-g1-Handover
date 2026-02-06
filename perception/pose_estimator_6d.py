"""
Moduł estymatora pozy 6D obiektów (6D Pose Estimator)

Ten node ROS 2 oblicza pozycję 3D (x, y, z) obiektów wykrytych na obrazie 2D.
Wykorzystuje:
- Detekcje 2D z object_detector (bounding boxes)
- Mapę głębokości z kamery RGB-D
- Parametry kalibracji kamery

KROK PO KROKU:
1. Odbiera detekcje 2D (prostokąty na obrazie)
2. Odbiera mapę głębokości z kamery
3. Odczytuje głębokość w centrum każdego bounding boxa
4. Konwertuje współrzędne 2D (piksele) na 3D (metry) używając pinhole camera model
5. Publikuje pozycję 3D obiektu

TOPIKI ROS 2:
- Subskrybuje: /camera/depth/image_raw (sensor_msgs/Image) - mapa głębokości
- Subskrybuje: /camera/color/camera_info (sensor_msgs/CameraInfo) - kalibracja
- Subskrybuje: /object_detections (vision_msgs/Detection2DArray) - detekcje 2D
- Publikuje: /object_pose (geometry_msgs/PoseStamped) - pozycja 3D obiektu

MATEMATYKA (Pinhole Camera Model):
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
gdzie:
    (u, v) = współrzędne piksela na obrazie
    (cx, cy) = optyczny środek kamery (z kalibracji)
    (fx, fy) = ogniskowe kamery (z kalibracji)
    z = głębokość z depth image
    (x, y, z) = pozycja 3D w układzie kamery
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class PoseEstimator6D(Node):
    """
    Node ROS 2 do estymacji pozy 6D obiektów
    
    Konwertuje detekcje 2D (bounding boxes) na pozycje 3D w przestrzeni
    używając depth image i parametrów kalibracji kamery.
    """

    def __init__(self):
        """
        Inicjalizacja node'a PoseEstimator6D
        
        KROK PO KROKU:
        1. Inicjalizuje bridge OpenCV-ROS
        2. Tworzy zmienne na dane z kamery (depth, parametry)
        3. Subskrybuje topic depth image
        4. Subskrybuje topic camera info (kalibracja)
        5. Subskrybuje topic z detekcjami 2D
        6. Tworzy publisher dla pozy 3D
        """
        super().__init__("pose_estimator_6d")

        # CvBridge do konwersji między ROS Image a numpy array
        self.bridge = CvBridge()
        
        # Zmienne przechowujące dane z kamery
        self.depth = None  # Mapa głębokości (numpy array)
        
        # Parametry kalibracji kamery (Camera Intrinsics)
        # fx, fy = focal length (ogniskowe) w pikselach
        # cx, cy = principal point (optyczny środek kamery) w pikselach
        self.fx = self.fy = self.cx = self.cy = None

        # Subskrypcja depth image z kamery RGB-D
        # Depth image zawiera odległość do każdego piksela w milimetrach
        self.create_subscription(Image,
                                 "/camera/depth/image_raw",
                                 self.depth_cb, 10)
        
        # Subskrypcja informacji o kalibracji kamery
        # CameraInfo zawiera parametry fx, fy, cx, cy w macierzy K
        self.create_subscription(CameraInfo,
                                 "/camera/color/camera_info",
                                 self.info_cb, 10)
        
        # Subskrypcja detekcji 2D z object_detector
        # Detection2DArray zawiera listę wykrytych obiektów z bounding boxes
        self.create_subscription(Detection2DArray,
                                 "/object_detections",
                                 self.det_cb, 10)

        # Publisher dla pozycji 3D obiektu
        # PoseStamped zawiera pozycję (x,y,z) i orientację obiektu
        self.pub = self.create_publisher(PoseStamped,
                                         "/object_pose", 10)

    def info_cb(self, msg):
        """
        Callback odbierający parametry kalibracji kamery
        
        Macierz kalibracji K ma format:
            K = [fx  0  cx]
                [ 0 fy  cy]
                [ 0  0   1]
        
        W ROS macierz K jest spłaszczona do listy 9 elementów:
            msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        
        Args:
            msg: Wiadomość sensor_msgs/CameraInfo
            
        KROK PO KROKU:
        1. Odbiera macierz K z wiadomości CameraInfo
        2. Wyciąga parametry fx, fy, cx, cy
        3. Zapisuje je do zmiennych klasy
        """
        # Wyciąganie parametrów z macierzy K
        # K[0] = fx (focal length w kierunku x)
        # K[4] = fy (focal length w kierunku y)
        # K[2] = cx (x współrzędna principal point)
        # K[5] = cy (y współrzędna principal point)
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_cb(self, msg):
        """
        Callback odbierający mapę głębokości z kamery
        
        Args:
            msg: Wiadomość sensor_msgs/Image zawierająca depth image
            
        KROK PO KROKU:
        1. Konwertuje ROS Image na numpy array
        2. Zapisuje do zmiennej self.depth
        
        Format depth image:
        - Typ: uint16 lub float32
        - Jednostki: milimetry (mm) lub metry (m)
        - Wartość 0 = brak pomiaru głębokości
        """
        # Konwersja ROS Image -> OpenCV numpy array
        # Depth image jest zazwyczaj w formacie 16-bit unsigned int
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def det_cb(self, msg):
        """
        Callback przetwarzający detekcje 2D i obliczający pozycje 3D
        
        KROK PO KROKU:
        1. Sprawdza, czy mamy wszystkie dane (depth, kalibracja, detekcje)
        2. Dla każdej detekcji:
           a. Pobiera centrum bounding boxa (u, v)
           b. Odczytuje głębokość z depth image
           c. Konwertuje (u, v, z) na (x, y, z) używając pinhole model
           d. Tworzy wiadomość PoseStamped
           e. Publikuje pozycję 3D
        
        Args:
            msg: Wiadomość vision_msgs/Detection2DArray z detekcjami
        """
        # Sprawdzamy, czy mamy wszystkie wymagane dane
        if self.depth is None or self.fx is None:
            # Jeszcze nie otrzymaliśmy depth image lub parametrów kalibracji
            return
        
        if not msg.detections:
            # Lista detekcji jest pusta - brak wykrytych obiektów
            return

        # Bierzemy pierwszą detekcję (w przyszłości można przetworzyć wszystkie)
        det = msg.detections[0]
        
        # Pobieramy centrum bounding boxa w pikselach
        # u = współrzędna x (kolumna)
        # v = współrzędna y (rząd)
        u = int(det.bbox.center.x)
        v = int(det.bbox.center.y)
        
        # Odczytujemy głębokość w centrum obiektu
        # self.depth[v, u] - numpy używa indeksowania [row, col] czyli [y, x]
        # Dzielimy przez 1000.0 aby przekonwertować mm -> m
        z = float(self.depth[v, u]) / 1000.0
        
        # Sprawdzamy poprawność głębokości
        if z <= 0.1:
            # Głębokość <= 10cm jest nieprawidłowa (za blisko lub brak pomiaru)
            return

        # PINHOLE CAMERA MODEL - konwersja 2D (u,v) + depth (z) -> 3D (x,y,z)
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        # z pozostaje bez zmian
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # Tworzenie wiadomości PoseStamped z pozycją 3D
        pose = PoseStamped()
        
        # Ustawiamy frame_id na "camera_link" (układ współrzędnych kamery)
        # Pozycja jest względem kamery, nie robota!
        # Transformacja do układu robota odbywa się przez TF2
        pose.header.frame_id = "camera_link"
        
        # Wypełniamy pozycję w metrach
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # Orientacja neutralna (brak obrotu)
        # W przyszłości można tu dodać estymację orientacji 6D
        pose.pose.orientation.w = 1.0

        # Publikujemy pozycję 3D obiektu
        # Inne node'y (np. grasp_planner) mogą użyć tej pozy
        self.pub.publish(pose)

def main():
    """
    Funkcja główna uruchamiająca node PoseEstimator6D
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a PoseEstimator6D
    3. Uruchamia pętlę przetwarzania (czeka na detekcje)
    4. Po zakończeniu (Ctrl+C) czyści zasoby
    
    Użycie:
        ros2 run g1_pick_and_handover pose_estimator_6d
    
    Wymagania:
        - Uruchomiony object_detector
        - Uruchomiony driver kamery RGB-D (np. RealSense)
        - Publikowane topiki: /camera/depth/image_raw, /camera/color/camera_info
    """
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Uruchomienie node'a - spin() czeka na callbacki
    rclpy.spin(PoseEstimator6D())
    
    # Czyszczenie po zakończeniu
    rclpy.shutdown()
