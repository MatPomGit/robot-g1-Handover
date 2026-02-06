"""
Planer chwytania obiektów (Grasp Planner)

Ten moduł oblicza pozycję pre-grasp (przed-chwytową), która jest używana
podczas podchodzenia do obiektu przed jego chwyceniem.

STRATEGIA CHWYTANIA:
1. Pre-grasp: Robot podchodzi nad obiekt (bezpieczna odległość)
2. Grasp: Robot opuszcza się i chwyta obiekt
3. Lift: Robot podnosi obiekt do góry

KROK PO KROKU:
1. Odbiera pozycję obiektu
2. Dodaje offset w osi Z (pionowo w górę)
3. Zwraca pozycję pre-grasp
"""

from geometry_msgs.msg import PoseStamped

def compute_pregrasp(object_pose: PoseStamped) -> PoseStamped:
    """
    Oblicza pozycję pre-grasp na podstawie pozycji obiektu
    
    KROK PO KROKU:
    1. Kopiuje pozycję obiektu
    2. Dodaje offset w osi Z (w górę)
    3. Zwraca pozycję pre-grasp
    
    Args:
        object_pose: PoseStamped - pozycja obiektu do chwycenia
                     - position (x, y, z) w metrach
                     - orientation (quaternion)
                     - header.frame_id (zwykle "camera_link" lub "base_link")
    
    Returns:
        PoseStamped - pozycja pre-grasp (nad obiektem)
    
    PRE-GRASP OFFSET:
    - Z (pionowy) += 0.10m (10 cm w górę)
    
    UZASADNIENIE:
    - 10 cm zapewnia bezpieczne podejście nad obiekt
    - Unika kolizji z obiektem i stołem podczas podchodzenia
    - Pozwala na pionowe opuszczenie chwytaka (deterministyczne)
    
    SEKWENCJA CHWYTANIA:
    1. move_to_pose(compute_pregrasp(object_pose))  ← tutaj jesteśmy
    2. move_to_pose(object_pose)                     ← opuszczenie
    3. close_gripper()                               ← chwyt
    4. move_to_pose(lifted_pose)                     ← podniesienie
    
    PRZYSZŁE ULEPSZENIA:
    - Offset zależny od wysokości obiektu
    - Różne strategie podejścia (od góry, z boku, skośnie)
    - Planowanie orientacji chwytaka (grasp pose estimation)
    - Unikanie przesłaniania obiektu przez ramię (occlusion)
    """
    # Tworzymy nową wiadomość PoseStamped dla pozycji pre-grasp
    pre = PoseStamped()
    
    # Kopiujemy header (frame_id, timestamp) z pozycji obiektu
    # Zachowujemy ten sam układ odniesienia
    pre.header = object_pose.header
    
    # Kopiujemy pozycję i orientację obiektu jako bazę
    pre.pose = object_pose.pose
    
    # PRE-GRASP OFFSET: przesuwamy w górę o 10 cm
    # Z (oś pionowa w górę) += 0.10m
    # Robot będzie 10 cm nad obiektem przed opuszczeniem się
    pre.pose.position.z += 0.10
    
    # UWAGI DLA STUDENTÓW:
    # 1. Wysokość pre-grasp (0.10m) można dostosować do:
    #    - Rozmiaru obiektu (większy obiekt = wyższy pre-grasp)
    #    - Geometrii chwytaka (długość palców)
    #    - Otoczenia (czy jest miejsce na podejście z boku?)
    # 2. Można dodać offset w XY dla podejścia pod kątem:
    #    - pre.pose.position.x -= 0.05  # podejście od tyłu
    # 3. Orientacja pozostaje taka sama jak obiektu
    #    (w rzeczywistości orientację należy obliczyć z grasp pose estimation)
    
    return pre
