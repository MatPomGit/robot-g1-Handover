"""
Moduł detekcji obiektów (Object Detector)

Ten node ROS 2 wykrywa obiekty na obrazie z kamery RGB używając sieci neuronowej YOLOv5.
YOLOv5 to szybki detektor obiektów, który w czasie rzeczywistym znajduje obiekty
i rysuje wokół nich prostokąty (bounding boxes).

KROK PO KROKU:
1. Odbiera obraz RGB z kamery
2. Przetwarza obraz przez sieć YOLOv5
3. Filtruje detekcje o niskiej pewności (< 60%)
4. Tworzy wiadomości ROS 2 z wykrytymi obiektami
5. Publikuje wyniki jako Detection2DArray

TOPIKI ROS 2:
- Subskrybuje: /camera/color/image_raw (sensor_msgs/Image) - obraz z kamery
- Publikuje: /object_detections (vision_msgs/Detection2DArray) - wykryte obiekty
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import torch

class ObjectDetector(Node):
    """
    Node ROS 2 do detekcji obiektów za pomocą YOLOv5
    
    Wykorzystuje pretrenowaną sieć YOLOv5s (small) do wykrywania 80 klas obiektów
    (osoby, butelki, kubki, telefony, klucze, itp.).
    """

    def __init__(self):
        """
        Inicjalizacja node'a ObjectDetector
        
        KROK PO KROKU:
        1. Tworzy subscriber dla obrazów z kamery
        2. Tworzy publisher dla wykrytych obiektów
        3. Inicjalizuje CvBridge (konwerter ROS Image <-> OpenCV)
        4. Ładuje model YOLOv5s z PyTorch Hub
        """
        super().__init__("object_detector")
        
        # Subskrybujemy topic z obrazem RGB z kamery
        # Callback image_cb będzie wywoływany przy każdym nowym obrazie
        self.sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_cb, 10)
        
        # Publisher dla wykrytych obiektów
        # Detection2DArray zawiera listę wykrytych obiektów z bounding boxes
        self.pub = self.create_publisher(
            Detection2DArray, "/object_detections", 10)

        # CvBridge konwertuje między formatem ROS (sensor_msgs/Image)
        # a formatem OpenCV (numpy array)
        self.bridge = CvBridge()
        
        # Ładowanie modelu YOLOv5s z PyTorch Hub
        # yolov5s = wersja "small" (szybka, mniej dokładna)
        # Alternatywy: yolov5m (medium), yolov5l (large), yolov5x (xlarge)
        # pretrained=True oznacza użycie wag wytrenowanych na COCO dataset
        self.model = torch.hub.load(
            "ultralytics/yolov5", "yolov5s", pretrained=True)

    def image_cb(self, msg):
        """
        Callback wywoływany przy każdym nowym obrazie z kamery
        
        KROK PO KROKU:
        1. Konwertuje ROS Image na OpenCV image (numpy array)
        2. Przepuszcza obraz przez model YOLOv5
        3. Filtruje detekcje o pewności < 60%
        4. Tworzy wiadomość Detection2DArray
        5. Publikuje wykryte obiekty
        
        Args:
            msg: Wiadomość sensor_msgs/Image z kamery
            
        Format wyników YOLOv5:
            results.xyxy[0] - tensor zawierający detekcje
            Każda detekcja: [x1, y1, x2, y2, confidence, class]
            gdzie (x1,y1) to lewy górny róg, (x2,y2) to prawy dolny róg
        """
        # Konwersja ROS Image -> OpenCV (numpy array, format BGR)
        # "bgr8" oznacza 8-bitowy obraz w formacie Blue-Green-Red
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Inference YOLOv5 - wykrywanie obiektów na obrazie
        # Model automatycznie:
        # 1. Zmienia rozmiar obrazu do 640x640
        # 2. Normalizuje wartości pikseli
        # 3. Przepuszcza przez sieć neuronową
        # 4. Stosuje Non-Maximum Suppression (usuwa duplikaty)
        results = self.model(img)

        # Przygotowanie wiadomości ROS z wykrytymi obiektami
        det_msg = Detection2DArray()
        det_msg.header = msg.header  # Kopiujemy timestamp i frame_id z obrazu

        # Iteracja przez wszystkie wykryte obiekty
        # results.xyxy[0] zawiera tensor: [[x1, y1, x2, y2, conf, cls], ...]
        for *xyxy, conf, cls in results.xyxy[0]:
            # Filtrowanie - ignorujemy detekcje o pewności < 60%
            # conf to prawdopodobieństwo, że wykryty obiekt jest poprawny (0-1)
            if conf < 0.6:
                continue
            
            # Tworzenie pojedynczej detekcji Detection2D
            # xyxy = współrzędne prostokąta [x1, y1, x2, y2]
            # conf = pewność detekcji (0.0 - 1.0)
            # cls = klasa obiektu (0-79 dla COCO dataset)
            det = self._create_detection(xyxy, conf, int(cls))
            det_msg.detections.append(det)

        # Publikujemy wszystkie wykryte obiekty na topic /object_detections
        # Inne node'y (np. pose_estimator_6d) mogą te dane wykorzystać
        self.pub.publish(det_msg)

    def _create_detection(self, bbox, conf, cls):
        """
        Tworzy wiadomość Detection2D z pojedynczego wykrycia YOLOv5
        
        KROK PO KROKU:
        1. Oblicza centrum bounding boxa
        2. Oblicza rozmiar bounding boxa
        3. Tworzy strukturę BoundingBox2D
        4. Dodaje informacje o klasie i pewności
        5. Zwraca kompletną Detection2D
        
        Args:
            bbox: Tensor [x1, y1, x2, y2] - współrzędne prostokąta w pikselach
            conf: float - pewność detekcji (0.0 - 1.0)
            cls: int - ID klasy obiektu (0-79)
            
        Returns:
            Detection2D: Wiadomość ROS zawierająca bounding box i metadane
            
        Przykładowe klasy COCO:
            0 = osoba, 39 = butelka, 41 = kubek, 46 = banan
            67 = telefon, 73 = laptop, 74 = mysz, 75 = pilot
        """
        from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
        
        # Tworzymy strukturę Detection2D
        det = Detection2D()
        
        # Tworzenie BoundingBox2D - prostokąt wokół obiektu
        bb = BoundingBox2D()
        
        # Obliczanie centrum prostokąta (środek między narożnikami)
        # bbox[0] = x1 (lewy), bbox[2] = x2 (prawy)
        # bbox[1] = y1 (górny), bbox[3] = y3 (dolny)
        bb.center.x = float((bbox[0] + bbox[2]) / 2)
        bb.center.y = float((bbox[1] + bbox[3]) / 2)
        
        # Obliczanie rozmiaru prostokąta
        bb.size_x = float(bbox[2] - bbox[0])  # szerokość w pikselach
        bb.size_y = float(bbox[3] - bbox[1])  # wysokość w pikselach
        
        det.bbox = bb

        # ObjectHypothesisWithPose - hipoteza o tym, czym jest obiekt
        # (w przyszłości można tu dodać pozę 6D obiektu)
        hyp = ObjectHypothesisWithPose()
        hyp.id = str(cls)          # ID klasy jako string (np. "41" dla kubka)
        hyp.score = float(conf)    # Pewność detekcji (0.0 - 1.0)
        
        det.results.append(hyp)
        return det

def main():
    """
    Funkcja główna uruchamiająca node ObjectDetector
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a ObjectDetector
    3. Uruchamia pętlę przetwarzania (czeka na obrazy z kamery)
    4. Po zakończeniu (Ctrl+C) czyści zasoby
    
    Użycie:
        ros2 run g1_pick_and_handover object_detector
    
    Wymagania:
        - Uruchomiony driver kamery publikujący na /camera/color/image_raw
        - Zainstalowany PyTorch i YOLOv5
        - Połączenie z internetem przy pierwszym uruchomieniu (pobieranie wag)
    """
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Uruchomienie node'a - spin() blokuje i przetwarza callbacki
    # Każdy nowy obraz wywołuje image_cb()
    rclpy.spin(ObjectDetector())
    
    # Czyszczenie po zakończeniu (Ctrl+C)
    rclpy.shutdown()
