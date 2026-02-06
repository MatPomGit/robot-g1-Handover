"""
Moduł wykonywania chwytania obiektów (Execute Grasp)

Ten node ROS 2 wykonuje sekwencję chwytania obiektu z użyciem automatu
stanów zarządzanego przez WMA Task Manager.

STRATEGIA CHWYTANIA (FSM - Finite State Machine):
1. IDLE: Stan początkowy, czekanie na obiekt
2. APPROACH: Podejście do pozycji pre-grasp (nad obiektem)
3. GRASP: Opuszczenie i chwyt obiektu
4. LIFT: Podniesienie obiektu do góry

KROK PO KROKU:
1. Odbiera pozycję obiektu z topic /object_pose
2. Aktualizuje stan w WMA Task Manager
3. Wykonuje akcję odpowiednią dla aktualnego stanu
4. Przechodzi do następnego stanu

TOPIKI ROS 2:
- Subskrybuje: /object_pose (geometry_msgs/PoseStamped) - pozycja obiektu 3D
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from manipulation.moveit_interface import MoveItInterface
from manipulation.grasp_planner import compute_pregrasp
from decision.wma_task_manager import WMATaskManager

class ExecuteGrasp(Node):
    """
    Node ROS 2 wykonujący sekwencję chwytania obiektu
    
    Implementuje maszynę stanów (FSM) do chwytania obiektów:
    idle -> approach -> grasp -> lift
    """

    def __init__(self):
        """
        Inicjalizacja node'a ExecuteGrasp
        
        KROK PO KROKU:
        1. Inicjalizuje interfejs MoveIt 2 (sterowanie ramieniem)
        2. Inicjalizuje WMA Task Manager (zarządzanie stanami)
        3. Subskrybuje topic z pozycją obiektu
        
        WMA TASK MANAGER:
        - Zarządza przejściami między stanami automatu
        - W uproszczonej wersji: deterministyczne przejścia
        - W pełnej wersji WMA: probabilistyczne decyzje
        """
        super().__init__("execute_grasp")
        
        # Inicjalizacja interfejsu do MoveIt 2
        # Pozwala na planowanie i wykonywanie ruchów ramienia + sterowanie chwytakiem
        self.moveit = MoveItInterface()
        
        # Inicjalizacja WMA Task Manager
        # Zarządza stanem automatu (idle -> approach -> grasp -> lift)
        self.wma = WMATaskManager()
        
        # Subskrypcja pozycji obiektu do chwycenia
        # Callback cb wywoływany przy każdej nowej pozycji obiektu
        self.sub = self.create_subscription(
            PoseStamped, "/object_pose", self.cb, 10)

    def cb(self, pose):
        """
        Callback przetwarzający pozycję obiektu i wykonujący akcję
        
        KROK PO KROKU:
        1. Aktualizuje stan w WMA Task Manager
        2. Sprawdza aktualny stan automatu
        3. Wykonuje akcję odpowiednią dla tego stanu
        4. Stan automatycznie przechodzi do następnego
        
        Args:
            pose: PoseStamped - pozycja obiektu do chwycenia
        
        AUTOMAT STANÓW (FSM):
        
        IDLE:
            - Stan początkowy
            - Robot czeka na obiekt
            - Przejście: idle -> approach
        
        APPROACH:
            - Robot podchodzi do pozycji pre-grasp (nad obiektem)
            - Pozycja: object_pose + offset w górę (0.10m)
            - Przejście: approach -> grasp
        
        GRASP:
            - Robot opuszcza się do obiektu
            - Zamyka chwytak (chwyta obiekt)
            - Przejście: grasp -> lift
        
        LIFT:
            - Robot podnosi obiekt do góry (0.15m)
            - Zabezpiecza obiekt przed upadkiem
            - Koniec sekwencji
        """
        # Aktualizuj stan w WMA Task Manager
        # WMA.update() zwraca aktualny stan po przejściu
        state = self.wma.update(pose)

        if state == "approach":
            # === STAN: APPROACH ===
            # Robot podchodzi do pozycji pre-grasp (nad obiektem)
            
            # KROK 1: Oblicz pozycję pre-grasp
            # compute_pregrasp dodaje offset w górę (default 0.10m)
            pregrasp_pose = compute_pregrasp(pose)
            
            # KROK 2: Przesuń ramię do pozycji pre-grasp
            # MoveIt 2 planuje trajektorię bez kolizji
            # Robot kończy 10 cm nad obiektem, gotowy do opuszczenia
            self.moveit.move_to_pose(pregrasp_pose)
            
            # UWAGA: Stan automatycznie przejdzie do "grasp" przy następnym wywołaniu

        elif state == "grasp":
            # === STAN: GRASP ===
            # Robot opuszcza się do obiektu i go chwyta
            
            # KROK 1: Przesuń ramię do pozycji obiektu
            # Robot opuszcza się pionowo z pre-grasp do grasp
            # Chwytak znajduje się wokół obiektu
            self.moveit.move_to_pose(pose)
            
            # KROK 2: Zamknij chwytak - chwyć obiekt
            # Palce delikatnie zamykają się wokół obiektu
            # W rzeczywistym systemie: monitoruj siłę chwytania!
            self.moveit.close_gripper()
            
            # EFEKT: Robot chwycił obiekt
            # Stan automatycznie przejdzie do "lift" przy następnym wywołaniu

        elif state == "lift":
            # === STAN: LIFT ===
            # Robot podnosi obiekt do góry
            
            # KROK 1: Utworzenie pozycji "lifted" (podniesionej)
            lifted = PoseStamped()
            lifted.header = pose.header  # Kopiuj frame_id i timestamp
            lifted.pose = pose.pose      # Kopiuj pozycję obiektu
            
            # KROK 2: Dodaj offset w górę (0.15m = 15 cm)
            # Robot podnosi obiekt powyżej stołu
            lifted.pose.position.z += 0.15
            
            # KROK 3: Przesuń ramię (z obiektem) do góry
            # MoveIt 2 planuje płynny ruch pionowy
            self.moveit.move_to_pose(lifted)
            
            # EFEKT: Robot chwycił i podniósł obiekt
            # Sekwencja chwytania zakończona!
            
            # PRZYSZŁE ROZSZERZENIA:
            # - Stan "retreat": cofnięcie ramienia do bezpiecznej pozycji
            # - Stan "place": umieszczenie obiektu w docelowej lokacji
            # - Wykrywanie błędów: czy obiekt został chwycony?

def main():
    """
    Funkcja główna uruchamiająca node ExecuteGrasp
    
    KROK PO KROKU:
    1. Inicjalizuje system ROS 2
    2. Tworzy instancję node'a ExecuteGrasp
    3. Uruchamia pętlę przetwarzania (czeka na pozycje obiektów)
    4. Po zakończeniu (Ctrl+C) czyści zasoby
    
    Użycie:
        ros2 run g1_pick_and_handover execute_grasp
    
    Wymagania:
        - Uruchomiony object_detector (detekcja obiektów)
        - Uruchomiony pose_estimator_6d (pozycja 3D)
        - Uruchomiony MoveIt 2 (planowanie ruchu)
        - Obiekt w polu widzenia kamery
    
    TESTOWANIE:
    1. Uruchom ten node
    2. Połóż obiekt przed kamerą
    3. Obserwuj sekwencję:
       - Robot podnosi się nad obiekt (approach)
       - Robot opuszcza się i chwyta (grasp)
       - Robot podnosi obiekt (lift)
    
    BEZPIECZEŃSTWO:
    - Upewnij się, że przestrzeń robocza jest wolna od przeszkód
    - Testuj najpierw w symulacji (MuJoCo, Gazebo)
    - Miej przygotowany E-STOP (emergency stop)
    """
    # Inicjalizacja ROS 2
    rclpy.init()
    
    # Uruchomienie node'a - spin() czeka na callbacki
    # Każda nowa pozycja obiektu wywołuje cb()
    rclpy.spin(ExecuteGrasp())
    
    # Czyszczenie po zakończeniu
    rclpy.shutdown()
