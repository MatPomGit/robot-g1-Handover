"""
Moduł sceny planowania (Planning Scene)

Ten moduł zawiera funkcje pomocnicze do dodawania przeszkód do sceny planowania
MoveIt 2. Przeszkody są uwzględniane podczas planowania trajektorii, aby robot
unikał kolizji.

PRZESZKODY W SCENIE:
- Stół: Płaska powierzchnia, na której leżą obiekty
- Człowiek: Cylindryczny model reprezentujący obecność człowieka
- Inne: Ściany, meble, dodatkowe obiekty

KROK PO KROKU:
1. Definiuje geometrię przeszkody (box, cylinder, sphere)
2. Określa pozycję przeszkody w przestrzeni
3. Dodaje przeszkodę do PlanningSceneInterface
4. MoveIt 2 automatycznie unika kolizji z tymi przeszkodami

TYPY GEOMETRII:
- Box (prostopadłościan): add_box(name, pose, size=(x, y, z))
- Cylinder: add_cylinder(name, pose, height, radius)
- Sphere: add_sphere(name, pose, radius)
- Mesh: add_mesh(name, pose, filename) - dla złożonych kształtów

UWAGA: Przeszkody są "statyczne" - nie poruszają się podczas planowania.
Dla dynamicznych przeszkód (np. ruchomy człowiek) trzeba aktualizować
scenę w czasie rzeczywistym.
"""

from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def add_table(scene):
    """
    Dodaje stół do sceny planowania MoveIt 2
    
    KROK PO KROKU:
    1. Tworzy PoseStamped z pozycją stołu
    2. Definiuje rozmiar stołu jako prostopadłościan (box)
    3. Dodaje stół do sceny planowania
    4. MoveIt 2 będzie unikał kolizji ze stołem
    
    Args:
        scene: PlanningSceneInterface - interfejs do sceny MoveIt 2
    
    PARAMETRY STOŁU:
    - Pozycja: (0.6, 0.0, 0.35) metrów względem base_link
        - x = 0.6m przed robotem
        - y = 0.0m (centralnie)
        - z = 0.35m nad ziemią (połowa wysokości stołu)
    - Rozmiar: 1.0m x 1.0m x 0.7m (długość x szerokość x wysokość)
    
    UWAGA O POZYCJI Z:
    MoveIt używa środka prostopadłościanu jako punktu odniesienia.
    Jeśli stół ma wysokość 0.7m, jego środek jest na wysokości 0.35m
    (0.7 / 2 = 0.35), a górna powierzchnia na 0.7m.
    
    DOSTOSOWANIE:
    - Zmień rozmiar jeśli masz inny stół
    - Zmierz pozycję stołu względem bazy robota
    - Użyj TF2 aby określić transformację base_link -> table
    """
    # Tworzymy wiadomość PoseStamped opisującą pozycję stołu
    pose = PoseStamped()
    
    # Ustawiamy frame_id na "base_link" (baza robota)
    # Wszystkie współrzędne są względem środka podstawy robota
    pose.header.frame_id = "base_link"
    
    # POZYCJA STOŁU (środek prostopadłościanu)
    pose.pose.position.x = 0.6   # 60 cm przed robotem
    pose.pose.position.z = 0.35  # 35 cm nad ziemią (środek stołu o wysokości 70cm)
    # pose.pose.position.y = 0.0  # Domyślnie 0 (centralnie)
    
    # ORIENTACJA (brak obrotu - stół jest poziomy)
    # Quaternion [0, 0, 0, 1] = brak obrotu
    pose.pose.orientation.w = 1.0
    
    # Dodajemy prostopadłościan (box) do sceny
    # Parametry:
    #   - "table": nazwa przeszkody (unikalny identyfikator)
    #   - pose: pozycja i orientacja środka prostopadłościanu
    #   - size: (długość, szerokość, wysokość) w metrach
    scene.add_box("table", pose, size=(1.0, 1.0, 0.7))
    
    # REZULTAT:
    # MoveIt 2 będzie teraz planował trajektorie unikające stołu
    # Ramię nie będzie przechodziło przez stół podczas ruchu

def add_human(scene):
    """
    Dodaje model człowieka do sceny planowania MoveIt 2
    
    Człowiek jest reprezentowany jako cylinder (uproszczony model),
    aby robot unikał kolizji z człowiekiem podczas ruchu.
    
    KROK PO KROKU:
    1. Tworzy PoseStamped z pozycją człowieka
    2. Definiuje rozmiar człowieka jako cylinder
    3. Dodaje cylinder do sceny planowania
    4. MoveIt 2 będzie unikał kolizji z człowiekiem
    
    Args:
        scene: PlanningSceneInterface - interfejs do sceny MoveIt 2
    
    PARAMETRY CZŁOWIEKA:
    - Pozycja: (0.8, 0.0, 0.9) metrów względem base_link
        - x = 0.8m przed robotem (dalej niż stół)
        - y = 0.0m (centralnie)
        - z = 0.9m nad ziemią (środek wysokości cylindra)
    - Wymiary:
        - Wysokość: 1.7m (średni wzrost człowieka)
        - Promień: 0.3m (szerokość z wyciągniętymi ramionami/torsem)
    
    UWAGA O MODELU:
    To jest uproszczony model! W rzeczywistym systemie:
    - Użyj human skeleton tracking (OpenPose, MediaPipe)
    - Reprezentuj człowieka jako zbiór cylindrów/sfer (ramiona, nogi, torso)
    - Aktualizuj pozycję w czasie rzeczywistym (dynamiczna przeszkoda)
    
    BEZPIECZEŃSTWO:
    - Ten cylinder to "strefa bezpieczeństwa" wokół człowieka
    - Robot NIE BĘDZIE wchodził w tę strefę podczas planowania
    - Zwiększ promień (np. 0.5m) dla większego marginesu bezpieczeństwa
    """
    # Tworzymy wiadomość PoseStamped opisującą pozycję człowieka
    pose = PoseStamped()
    
    # Ustawiamy frame_id na "base_link"
    pose.header.frame_id = "base_link"
    
    # POZYCJA CZŁOWIEKA (środek cylindra)
    pose.pose.position.x = 0.8   # 80 cm przed robotem (za stołem)
    pose.pose.position.z = 0.9   # 90 cm nad ziemią (środek cylindra o wysokości 1.7m)
    # pose.pose.position.y = 0.0  # Domyślnie 0 (centralnie przed robotem)
    
    # ORIENTACJA (cylinder pionowy, bez obrotu)
    pose.pose.orientation.w = 1.0
    
    # Dodajemy cylinder do sceny
    # Parametry:
    #   - "human": nazwa przeszkody (unikalny identyfikator)
    #   - pose: pozycja i orientacja środka cylindra
    #   - height: wysokość cylindra w metrach (1.7m)
    #   - radius: promień cylindra w metrach (0.3m)
    scene.add_cylinder("human", pose, height=1.7, radius=0.3)
    
    # REZULTAT:
    # MoveIt 2 będzie planował trajektorie z dala od człowieka
    # Robot nie będzie uderzał w człowieka podczas przekazywania obiektów
    
    # UWAGA DLA STUDENTÓW:
    # W prawdziwym systemie handover:
    # 1. Człowiek się porusza -> trzeba aktualizować pozycję cylindra
    # 2. Podczas przekazywania obiekt MUSI zbliżyć się do człowieka
    #    -> przejściowo usuń/zmniejsz cylinder albo użyj velocity scaling
    # 3. Monitoruj collision feedback i natychmiast zatrzymaj ruch jeśli wykryto kontakt
