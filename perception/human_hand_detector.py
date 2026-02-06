"""
Moduł detekcji dłoni człowieka (Human Hand Detector)

Ten node ROS 2 wykrywa pozycję dłoni człowieka w przestrzeni 3D i określa,
czy człowiek wyciąga rękę w kierunku robota (intencja przekazania/odebrania obiektu).

KROK PO KROKU:
1. Odbiera obraz z kamery RGB-D
2. Używa MediaPipe lub OpenPose do wykrycia kluczowych punktów dłoni
3. Oblicza pozycję 3D dłoni w układzie współrzędnych robota
4. Publikuje pozycję dłoni i informację o intencji człowieka

TOPIKI ROS 2:
- Publikuje: /human_hand_pose (geometry_msgs/PoseStamped) - pozycja dłoni w 3D
- Publikuje: /human_reaching (std_msgs/Bool) - czy człowiek wyciąga rękę
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class HumanHandDetector(Node):
    """
    Node ROS 2 do detekcji dłoni człowieka
    
    Wykrywa pozycję dłoni w przestrzeni 3D oraz określa intencję człowieka
    (czy wyciąga rękę w kierunku robota, aby dać lub wziąć obiekt).
    """

    def __init__(self):
        """
        Inicjalizacja node'a HumanHandDetector
        
        Tworzy:
        - Publisher dla pozycji dłoni (/human_hand_pose)
        - Publisher dla intencji człowieka (/human_reaching)
        """
        super().__init__("human_hand_detector")
        
        # Publisher dla pozycji dłoni w przestrzeni 3D
        # PoseStamped zawiera: pozycję (x,y,z) + orientację (quaternion) + timestamp
        self.pub_pose = self.create_publisher(
            PoseStamped, "/human_hand_pose", 10)
        
        # Publisher dla informacji, czy człowiek wyciąga rękę
        # True = człowiek wyciąga rękę (chce dać/wziąć obiekt)
        # False = człowiek nie wyciąga ręki (brak intencji)
        self.pub_intent = self.create_publisher(
            Bool, "/human_reaching", 10)

    def detect_hand(self, image):
        """
        Wykrywa pozycję dłoni na obrazie z kamery
        
        KROK PO KROKU:
        1. Przetwarza obraz z kamery RGB
        2. Używa MediaPipe Hand Tracking lub OpenPose
        3. Wykrywa 21 kluczowych punktów dłoni
        4. Oblicza centrum dłoni (średnia punktów)
        5. Konwertuje pozycję 2D (piksele) na 3D (metry) używając mapy głębokości
        
        Args:
            image: Obraz z kamery RGB (numpy array, format BGR)
            
        Returns:
            list: Pozycja dłoni [x, y, z] w metrach względem kamery
            
        TODO: Zintegrować prawdziwy MediaPipe/OpenPose zamiast placeholder
        """
        # PLACEHOLDER - w rzeczywistej implementacji tutaj byłby kod:
        # import mediapipe as mp
        # mp_hands = mp.solutions.hands
        # hands = mp_hands.Hands()
        # results = hands.process(image)
        # # Wyciągnij pozycję środka dłoni z results.multi_hand_landmarks
        
        # Tymczasowa stała pozycja dla testów
        # x=0.6m przed robotem, y=0.0m (centralnie), z=1.0m (wysokość biurka)
        return [0.6, 0.0, 1.0]

    def process(self):
        """
        Przetwarza detekcję i publikuje wyniki
        
        KROK PO KROKU:
        1. Tworzy wiadomość PoseStamped dla pozycji dłoni
        2. Ustawia ramkę odniesienia (base_link = baza robota)
        3. Wypełnia pozycję xyz i orientację
        4. Publikuje pozycję dłoni na topic /human_hand_pose
        5. Publikuje informację o intencji na topic /human_reaching
        
        UWAGI:
        - base_link to główny układ współrzędnych robota (środek podstawy)
        - Orientacja w = 1.0 oznacza brak obrotu (quaternion jednostkowy)
        - Ta metoda powinna być wywoływana cyklicznie (np. 30 Hz)
        """
        # Tworzymy wiadomość ROS 2 typu PoseStamped
        pose = PoseStamped()
        
        # Ustawiamy ramkę odniesienia na "base_link" (baza robota)
        # Wszystkie współrzędne będą względem środka podstawy robota
        pose.header.frame_id = "base_link"
        
        # Pozycja dłoni w metrach (w układzie base_link):
        pose.pose.position.x = 0.6  # 60 cm przed robotem
        pose.pose.position.y = 0.0  # Na środku (lewo-prawo)
        pose.pose.position.z = 1.0  # 1 metr nad ziemią (wysokość biurka)
        
        # Orientacja jako quaternion (x, y, z, w)
        # w=1.0, x=y=z=0 oznacza brak obrotu (orientacja neutralna)
        pose.pose.orientation.w = 1.0

        # Publikujemy pozycję dłoni
        # Inne node'y (np. execute_handover) mogą subskrybować ten topic
        self.pub_pose.publish(pose)
        
        # Publikujemy informację, że człowiek wyciąga rękę (True)
        # W rzeczywistej implementacji to byłoby wykrywane na podstawie:
        # - kierunku ruchu dłoni (czy porusza się w stronę robota)
        # - prędkości ruchu dłoni
        # - odległości od robota
        self.pub_intent.publish(Bool(data=True))

def main():
    """
    Funkcja główna uruchamiająca node HumanHandDetector
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a HumanHandDetector
    3. Uruchamia pętlę przetwarzania (spin) - czeka na callbacki
    4. Po zakończeniu (Ctrl+C) czyści zasoby ROS 2
    
    Użycie:
        ros2 run g1_pick_and_handover human_hand_detector
    """
    # Krok 1: Inicjalizacja ROS 2
    rclpy.init()
    
    # Krok 2: Utworzenie node'a
    node = HumanHandDetector()
    
    # Krok 3: Uruchomienie pętli przetwarzania
    # spin() blokuje wykonanie i przetwarza callbacki aż do Ctrl+C
    rclpy.spin(node)
    
    # Krok 4: Czyszczenie po zakończeniu
    rclpy.shutdown()
