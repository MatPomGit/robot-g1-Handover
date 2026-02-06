# Przewodnik dla Kontrybutorów (Contributing Guide)

## 📖 Wprowadzenie

Dziękujemy za zainteresowanie projektem Robot G1 Handover! Ten dokument opisuje standardy i zasady współpracy w projekcie.

## 🏗️ Struktura Projektu

```
robot-g1-Handover/
├── perception/          # Moduł percepcji (kamery, detekcja)
├── manipulation/        # Moduł manipulacji (MoveIt 2, chwytak)
├── decision/            # Moduł decyzyjny (WMA, FSM)
├── launch/              # Pliki uruchomieniowe ROS 2
├── config/              # Pliki konfiguracyjne (YAML)
├── package.xml          # Deskryptor pakietu ROS 2
├── setup.py             # Konfiguracja instalacji Python
└── requirements.txt     # Zależności Python
```

## 📏 Standardy Kodu

### Python
- **Styl**: PEP 8
- **Docstringi**: Każda klasa i publiczna metoda musi mieć docstring
- **Komentarze**: Pisz komentarze po polsku (język projektu)
- **Nazewnictwo plików**: `snake_case.py` (małe litery, podkreślenia)
- **Nazewnictwo klas**: `PascalCase`
- **Nazewnictwo funkcji**: `snake_case`

### Pliki konfiguracyjne
- **Format**: YAML (`.yaml`)
- **Nazewnictwo**: `snake_case.yaml` (bez spacji, bez polskich znaków)
- **Komentarze**: Zawsze dodawaj jednostki w komentarzach

### Commity
- Pisz zwięzłe komunikaty commitów
- Format: `<typ>: <opis>` (np. `feat: dodaj detekcję nowych obiektów`)
- Typy: `feat`, `fix`, `docs`, `refactor`, `test`, `config`

### Launch Files
- Nazwa: `<opis>.launch.py`
- Umieszczaj w katalogu `launch/`
- Dodawaj docstring z opisem użycia

## 🔄 Workflow

### 1. Tworzenie brancha
```bash
git checkout -b feature/<nazwa-funkcji>
```

### 2. Implementacja
- Implementuj zmiany w odpowiednim module
- Dodaj docstringi i komentarze
- Przetestuj lokalnie

### 3. Pull Request
- Opisz co zmieniłeś i dlaczego
- Przypisz reviewera
- Dołącz wyniki testów (jeśli dostępne)

## 🧪 Testowanie

### Testowanie lokalne
```bash
# Zbuduj workspace
cd ~/ros2_ws && colcon build

# Uruchom pojedynczy node
ros2 run g1_pick_and_handover <node_name>

# Uruchom cały pipeline
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Sprawdzanie topików
```bash
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic hz <topic_name>
```

## 📋 Zgłaszanie Błędów

Otwórz Issue na GitHubie z informacjami:
1. **Opis problemu**
2. **Kroki do reprodukcji**
3. **Oczekiwane zachowanie**
4. **Logi** (`ros2 run ... --ros-args --log-level debug`)
5. **Środowisko** (Ubuntu, ROS 2, Python)

## 📚 Przydatne Zasoby

- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [PEP 8 Style Guide](https://peps.python.org/pep-0008/)
