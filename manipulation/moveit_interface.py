"""
Interfejs MoveIt 2 (MoveIt Interface)

Ten moduł zapewnia uproszczony interfejs do planowania i wykonywania ruchów
ramienia robota oraz sterowania chwytakiem za pomocą MoveIt 2.

MoveIt 2 to zaawansowany framework do:
- Planowania trajektorii bez kolizji
- Kinematyki odwrotnej (IK) - obliczania kątów stawów dla zadanej pozycji końcówki
- Unikania kolizji z przeszkodami
- Sterowania manipulatorami

KROK PO KROKU:
1. Inicjalizuje MoveIt Commander dla ramienia i chwytaka
2. Zapewnia metody do:
   - Ruchu ramienia do zadanej pozycji (move_to_pose)
   - Zamykania chwytaka (close_gripper)
   - Otwierania chwytaka (open_gripper)

GRUPY RUCHOWE (Move Groups):
- "arm" = całe ramię robota (stawy od ramienia do nadgarstka)
- "gripper" = chwytak (palce/szczęki)

TOPIKI ROS 2:
- Publikuje/subskrybuje różne topiki MoveIt 2 (trajektorie, stany, itp.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

class MoveItInterface(Node):
    """
    Node ROS 2 zapewniający interfejs do MoveIt 2
    
    Upraszcza sterowanie ramieniem robota i chwytakiem,
    ukrywając złożoność bezpośredniego użycia MoveIt 2.
    """

    def __init__(self):
        """
        Inicjalizacja interfejsu MoveIt 2
        
        KROK PO KROKU:
        1. Inicjalizuje node ROS 2
        2. Tworzy MoveGroupCommander dla ramienia ("arm")
        3. Tworzy MoveGroupCommander dla chwytaka ("gripper")
        4. Inicjalizuje PlanningSceneInterface (scena z przeszkodami)
        
        UWAGA: Nazwy grup ("arm", "gripper") muszą odpowiadać tym zdefiniowanym
        w konfiguracji MoveIt 2 (SRDF - Semantic Robot Description Format)
        """
        super().__init__("moveit_interface")
        
        # MoveGroupCommander dla ramienia robota
        # Pozwala na planowanie i wykonywanie ruchów całego ramienia
        # Grupa "arm" powinna być zdefiniowana w pliku .srdf robota
        self.arm = MoveGroupCommander("arm")
        
        # MoveGroupCommander dla chwytaka
        # Kontroluje otwieranie/zamykanie chwytaka (palców)
        # Grupa "gripper" powinna być zdefiniowana w pliku .srdf robota
        self.gripper = MoveGroupCommander("gripper")
        
        # PlanningSceneInterface zarządza sceną planowania
        # Pozwala dodawać/usuwać przeszkody (stół, ściany, obiekty)
        # MoveIt 2 będzie planował trajektorie unikające kolizji
        self.scene = PlanningSceneInterface()

    def move_to_pose(self, pose: PoseStamped):
        """
        Przesuwa ramię robota do zadanej pozycji w przestrzeni
        
        KROK PO KROKU:
        1. Ustawia cel jako pozycję końcówki ramienia (end-effector)
        2. Wywołuje planer MoveIt 2 (np. RRTConnect)
        3. Planer oblicza trajektorię kątów stawów
        4. Sprawdza kolizje na całej trajektorii
        5. Jeśli plan jest poprawny, wykonuje ruch
        
        Args:
            pose: PoseStamped - docelowa pozycja i orientacja end-effectora
                  - position (x, y, z) w metrach
                  - orientation (quaternion) jako (x, y, z, w)
                  - header.frame_id określa układ odniesienia
        
        KINEMATYKA ODWROTNA (IK):
        MoveIt 2 automatycznie rozwiązuje problem IK:
        - Dane: pozycja końcówki (x, y, z) + orientacja
        - Oblicza: kąty wszystkich stawów ramienia
        - Sprawdza: czy rozwiązanie jest osiągalne (w przestrzeni roboczej)
        
        PLANOWANIE TRAJEKTORII:
        - Algorytm: RRTConnect (lub inne z OMPL)
        - Próbkuje losowo przestrzeń konfiguracyjną
        - Buduje drzewo połączeń bez kolizji
        - Znajduje ścieżkę od aktualnej do docelowej konfiguracji
        """
        # Ustawienie docelowej pozycji dla końcówki ramienia
        # set_pose_target automatycznie wywoła solver IK
        self.arm.set_pose_target(pose)
        
        # Planowanie trajektorii
        # plan() zwraca tuple (success: bool, trajectory: RobotTrajectory)
        # success=True jeśli znaleziono poprawną trajektorię bez kolizji
        plan = self.arm.plan()
        
        # Sprawdzamy, czy planowanie się powiodło
        if plan[0]:
            # Plan istnieje i jest poprawny
            # Wykonujemy zaplanowaną trajektorię
            # wait=True blokuje wykonanie do zakończenia ruchu
            self.arm.execute(plan[1], wait=True)
            # Robot wykonuje płynny ruch po zaplanowanej trajektorii
        # else:
            # Planowanie nie powiodło się (pozycja nieosiągalna lub kolizja)
            # W produkcyjnym kodzie należy obsłużyć ten błąd

    def close_gripper(self):
        """
        Zamyka chwytak robota (chwyta obiekt)
        
        KROK PO KROKU:
        1. Ustawia named target "closed" dla chwytaka
        2. Wykonuje ruch zamykający palce/szczęki
        3. Czeka na zakończenie ruchu
        
        NAMED TARGETS:
        Named targets to predefiniowane konfiguracje stawów zdefiniowane w SRDF:
        - "closed" = chwytak zamknięty (palce złączone)
        - "open" = chwytak otwarty (palce rozdzielone)
        
        SIŁA CHWYTANIA:
        W rzeczywistych robotach należy monitorować:
        - Siłę chwytania (force sensing)
        - Prąd silników (current sensing)
        - Feedback z czujników dotyku
        Aby nie zniszczyć delikatnych obiektów
        """
        # Ustawienie celu jako named target "closed"
        # "closed" musi być zdefiniowany w pliku .srdf chwytaka
        self.gripper.set_named_target("closed")
        
        # Wykonanie ruchu zamykającego chwytak
        # go() łączy planowanie i wykonanie w jednej komendzie
        # wait=True czeka aż chwytak się całkowicie zamknie
        self.gripper.go(wait=True)

    def open_gripper(self):
        """
        Otwiera chwytak robota (puszcza obiekt)
        
        KROK PO KROKU:
        1. Ustawia named target "open" dla chwytaka
        2. Wykonuje ruch otwierający palce/szczęki
        3. Czeka na zakończenie ruchu
        
        ZASTOSOWANIE:
        - Przed chwyceniem obiektu (pre-grasp)
        - Podczas przekazywania obiektu człowiekowi
        - Po umieszczeniu obiektu na stole
        """
        # Ustawienie celu jako named target "open"
        # "open" musi być zdefiniowany w pliku .srdf chwytaka
        self.gripper.set_named_target("open")
        
        # Wykonanie ruchu otwierającego chwytak
        # go() wykonuje ruch i czeka na zakończenie
        self.gripper.go(wait=True)
