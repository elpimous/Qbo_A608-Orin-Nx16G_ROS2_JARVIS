# head_tracker_node

**Suivi de cible générique par PID pour tête robotique 2-DOF (pan/tilt)**

Auteur : Vincent Foucault
Date : 28 Mars 2026
Plateforme : Jetson Orin NX 16 Go — ROS 2 Humble / Jazzy

---

## Présentation

`head_tracker_node` est un contrôleur C++ ROS 2 qui pilote une tête robotique à 2 axes (pan + tilt) équipée de servomoteurs Dynamixel AX-12A. Il reçoit une **Region of Interest (ROI)** depuis n'importe quelle source de détection (visage, balle, objet, main…), calcule l'erreur de centrage par rapport à l'image caméra, et envoie des commandes de position + vitesse aux servos via un PID avec filtre passe-bas et contrôle de vitesse proportionnel à l'erreur.

Le node est conçu pour le robot **Néo**, une tête mobile avec 2 yeux caméra, un RPLidar monté sur le dessus, et des servos Dynamixel AX-12A pour le pan et le tilt.

### Caractéristiques

- **Source-agnostique** : suit n'importe quel `sensor_msgs/RegionOfInterest` — balle rouge, visage, objet YOLO, main MediaPipe… Le switch entre sources est instantané, il suffit de changer le publisher sur `/target_roi`.
- **PID séparé pan/tilt** avec anti-windup, dérivée propre sur dt, et filtre passe-bas exponentiel.
- **Vitesse proportionnelle à l'erreur** : mouvement rapide quand la cible est loin, lent et précis à l'approche.
- **Seuil de commande** (`cmd_threshold_rad`) : évite de spammer les servos avec des micro-corrections de 1 tick.
- **Deadzone** : zone morte en pixels au centre de l'image pour éliminer le jitter quand la cible est centrée.
- **Mode recherche** : si la cible est perdue > `loss_timeout` secondes, la tête revient au neutre puis effectue des mouvements aléatoires lents.
- **Tous les paramètres sont dynamiques** : tuning PID en live via `ros2 param set`, sans redémarrer le node.
- **Compatible `qbo_dynamixel`** : publie sur `/cmd_joints` au format `JointState` (position + velocity), directement consommé par le driver Dynamixel.
- **Conversion pixel → angle** : utilise les paramètres intrinsèques caméra (`/camera_info`).

---

## Architecture ROS 2

```
┌─────────────────────┐     /target_roi      ┌─────────────────────┐     /cmd_joints      ┌─────────────────────┐
│  red_ball_tracker    │──────────────────────▶│  head_tracker_node  │──────────────────────▶│   qbo_dynamixel     │
│  face_detector       │   RegionOfInterest   │                     │     JointState        │                     │
│  yolo_detector       │                      │  PID pan + tilt     │   (pos + vel)         │  Dynamixel AX-12    │
│  ...                 │                      │  Filtre passe-bas   │                       │  head_pan_joint     │
└─────────────────────┘                      │  Recherche auto     │◀──────────────────────│  head_tilt_joint    │
                                              └─────────────────────┘     /joint_states     └─────────────────────┘
                                                       │                   JointState
                                                       │                  (pos feedback)
                                                       ▼
                                              /reset_head_neutral
                                                   std_msgs/Bool
```

### Topics souscrits

| Topic | Type | Description |
|---|---|---|
| `/target_roi` | `sensor_msgs/RegionOfInterest` | ROI de la cible à suivre (centre calculé automatiquement) |
| `/joint_states` | `sensor_msgs/JointState` | Position réelle des servos (feedback depuis qbo_dynamixel) |
| `/camera_info` | `sensor_msgs/CameraInfo` | Paramètres intrinsèques caméra (fx, fy, cx, cy) — optionnel |

### Topics publiés

| Topic | Type | Description |
|---|---|---|
| `/cmd_joints` | `sensor_msgs/JointState` | Commande position + vitesse pour chaque joint |
| `/reset_head_neutral` | `std_msgs/Bool` | `true` = tête perdue, retour neutre ; `false` = cible retrouvée |

---

## Dépendances

### Dépendances ROS 2

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `rcl_interfaces`

### Dépendances système

- ROS 2 Humble ou Jazzy
- Compilateur C++17
- `colcon` (build system)

### Packages nécessaires dans le workspace

- **`qbo_arduqbo`** — driver Dynamixel (`qbo_dynamixel` node), gère la communication série avec les AX-12.
- **`qbo_msgs`** — messages et services custom (`TorqueEnable`).
- **Un publisher ROI** — par exemple `ball_tracking_cpp` (`red_ball_tracker`) ou tout node publiant un `RegionOfInterest` sur `/target_roi`.

---

## Structure du package

```
head_tracker/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── head_tracker_params.yaml
└── src/
    └── head_tracker_node.cpp
```

## Utilisation

### Lancer le node

```bash
# Avec les paramètres par défaut
ros2 run head_tracker head_tracker_node

# Avec le fichier de config YAML
ros2 run head_tracker head_tracker_node --ros-args \
  --params-file ~/neo_ws/src/head_tracker/config/head_tracker_params.yaml
```

### Lancer la stack complète (exemple)

ps : le node camera doit être actif !!!

```bash
# Terminal 1 — Driver Dynamixel
ros2 run qbo_arduqbo qbo_dynamixel --ros-args \
  --params-file ~/neo_ws/src/qbo_arduqbo/config/dynamixel_params.yaml

# Terminal 2 — Détecteur (exemple : balle rouge)
ros2 run ball_tracking_cpp red_ball_tracker_cpp

# Terminal 3 — Head tracker
ros2 run head_tracker head_tracker_node --ros-args \
  --params-file ~/neo_ws/src/head_tracker/config/head_tracker_params.yaml
```

### Changer de source de détection

Le switch entre sources est instantané — il suffit que le nouveau détecteur publie sur `/target_roi`. Plusieurs approches possibles :

```bash
# Option 1 : remap le topic du détecteur
ros2 run face_detector face_detect_node --ros-args -r /face_roi:=/target_roi

# Option 2 : utiliser un multiplexeur de topics
# (node Python qui souscrit à /face_roi et /ball_roi et forward vers /target_roi)
```

### Tuning PID en live

```bash
# Ajuster le PID pan
ros2 param set /head_tracker_node pid_pan_kp 0.8
ros2 param set /head_tracker_node pid_pan_kd 0.05

# Ajuster le PID tilt (plus doux si charge lourde — RPLidar)
ros2 param set /head_tracker_node pid_tilt_kp 0.5
ros2 param set /head_tracker_node pid_tilt_kd 0.10
ros2 param set /head_tracker_node max_tilt_vel 2.0

# Ajuster la vitesse
ros2 param set /head_tracker_node vel_gain 10.0
ros2 param set /head_tracker_node min_vel 0.3

# Ajuster le seuil de commande
ros2 param set /head_tracker_node cmd_threshold_rad 0.02

# Voir tous les paramètres
ros2 param list /head_tracker_node
ros2 param dump /head_tracker_node
```

### Contrôle du torque

```bash
# Désactiver le couple (tête libre)
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: false}"

# Réactiver
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable "{torque_enable: true}"
```

### Monitoring

```bash
# Voir les commandes envoyées
ros2 topic echo /cmd_joints

# Voir l'état des servos
ros2 topic echo /joint_states

# Voir si la cible est perdue
ros2 topic echo /reset_head_neutral
```

### Notes de tuning

**Ratio Kd/Kp ≈ 0.06–0.12** pour un amortissement correct sans oscillation.

**Tilt avec charge lourde** (RPLidar ou capteur au-dessus de la tête) : baisser Kp à 0.5, monter Kd à 0.10, limiter max_tilt_vel à 2.0. L'inertie supplémentaire cause des oscillations si le PID est trop agressif.

**cmd_threshold_rad = 0.02** est le sweet spot validé : filtre les micro-corrections (1–2 ticks AX-12) tout en gardant un centrage précis (~3–4 pixels).

**smoothing_alpha = 0.5** : bon compromis entre réactivité et lissage. Plus bas (0.3) = plus lisse mais micro-ralentis en trajectoire. Plus haut (0.7+) = réactif mais risque de jitter.

---

## Fonctionnement interne

### Pipeline de contrôle (30 Hz)

1. **Réception ROI** → calcul du centre cible (cx, cy)
2. **Erreur pixel** → distance entre centre cible et centre image
3. **Deadzone** → si erreur < 15 px, pas de mouvement
4. **Conversion pixel → rad** → `atan2(err_px, fx)` ou fallback linéaire
5. **PID** → correction de position en radians (Kp + Ki avec anti-windup + Kd)
6. **Filtre passe-bas** → lisse la position et la vitesse (alpha exponentiel)
7. **Vitesse** → `vel = |erreur| × vel_gain`, clampée entre min_vel et max_vel
8. **Seuil de commande** → n'envoie que si `|delta_position| > cmd_threshold_rad`
9. **Publication** → `JointState` sur `/cmd_joints` avec position + velocity

### Machine d'états

```
  ROI valide          ROI vide > loss_timeout
  ┌────────┐         ┌──────────────────────┐
  │TRACKING│────────▶│  RESET TO NEUTRAL    │
  │  PID   │◀────────│  (publie reset=true) │
  └────────┘         └──────────┬───────────┘
  ROI retrouvée                 │ après reset
  (publie reset=false)          ▼
                      ┌──────────────────────┐
                      │  SEARCH (aléatoire)  │
                      │  positions random    │
                      │  vitesse lente       │
                      └──────────────────────┘
```

---

## Fichiers du projet

| Fichier | Description |
|---|---|
| `src/head_tracker_node.cpp` | Node complet (PID + LowPass + HeadTrackerNode) — fichier unique |
| `config/head_tracker_params.yaml` | Configuration par défaut validée |

---

## Licence

MIT — Vincent Foucault, 28 Mars 2026
