"""
Planer przekazywania obiektów (Handover Planner)

Ten moduł oblicza optymalną pozycję, w której robot powinien przekazać
lub odebrać obiekt od człowieka.

KROK PO KROKU:
1. Odbiera pozycję dłoni człowieka
2. Stosuje ergonomiczny offset (przesunięcie)
3. Zwraca pozycję dla chwytaka robota

ERGONOMIA PRZEKAZYWANIA:
- Robot nie powinien podchodzić zbyt blisko dłoni (bezpieczeństwo)
- Pozycja powinna być wygodna dla człowieka
- Offset zapewnia "przestrzeń buforową" między robotem a człowiekiem
"""

from geometry_msgs.msg import PoseStamped

def compute_handover_pose(human_hand_pose):
    """
    Oblicza pozycję przekazania obiektu na podstawie pozycji dłoni człowieka
    
    KROK PO KROKU:
    1. Kopiuje pozycję dłoni człowieka
    2. Stosuje ergonomiczny offset w osi Z (w dół)
    3. Zwraca zmodyfikowaną pozycję dla robota
    
    Args:
        human_hand_pose: PoseStamped - pozycja dłoni człowieka w 3D
                        - position (x, y, z) w metrach
                        - orientation (quaternion)
                        - header zawiera frame_id i timestamp
    
    Returns:
        PoseStamped - pozycja, do której robot powinien się przemieścić
                      aby przekazać/odebrać obiekt
    
    ERGONOMICZNY OFFSET:
    - Z (pionowy) -= 0.05m (5 cm w dół)
    
    UZASADNIENIE OFFSETU:
    - 5 cm w dół umożliwia naturalne przekazanie obiektu
    - Człowiek może wygodnie złapać obiekt od góry
    - Unika kolizji między chwytakiem a dłonią
    
    PRZYSZŁE ULEPSZENIA:
    - Offset zależny od rozmiaru obiektu
    - Offset zależny od antropometrii człowieka (wzrost, długość ramienia)
    - Adaptacyjny offset na podstawie feedbacku (czy człowiek sięga dalej?)
    - Offset w płaszczyźnie XY dla lepszej dostępności
    """
    # Tworzymy nową wiadomość PoseStamped dla pozycji handover
    pose = PoseStamped()
    
    # Kopiujemy header (frame_id, timestamp) z pozycji dłoni
    # Zachowujemy ten sam układ odniesienia i znacznik czasu
    pose.header = human_hand_pose.header
    
    # Kopiujemy pozycję i orientację dłoni jako bazę
    pose.pose = human_hand_pose.pose
    
    # ERGONOMICZNY OFFSET: przesuwamy w dół o 5 cm
    # Z (oś pionowa w górę) -= 0.05m
    # Oznacza to, że robot ustawi się 5 cm poniżej pozycji dłoni
    pose.pose.position.z -= 0.05   # ergonomiczny offset
    
    # UWAGI DLA STUDENTÓW:
    # 1. Ten offset można dostosować empirycznie (testując z użytkownikami)
    # 2. Można dodać więcej offsetów:
    #    - pose.pose.position.x += 0.02  # lekko bliżej człowieka
    #    - pose.pose.position.y += 0.01  # lekko w lewo/prawo
    # 3. Orientacja pozostaje taka sama (może być do poprawy)
    
    return pose
