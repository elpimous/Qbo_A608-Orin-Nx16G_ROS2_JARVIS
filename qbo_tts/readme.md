# qbo_tts — Node ROS2 TTS pour Néo (QBo)
## Vincent Foucault Avril 2026.

Synthèse vocale française **offline, temps-réel, accélérée GPU** pour le robot Néo.

Basé sur [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) (moteur VITS + ONNX Runtime CUDA)
et [PortAudio](http://www.portaudio.com/) pour la sortie audio USB.

---

## Performances mesurées (Jetson Orin NX 16 Go / USB audio C-Media)

| Métrique | Valeur |
|---|---|
| Latence 1ère phrase (cold start GPU) | ~470 ms | -> géré par 1er parlé au démarrage du robot.
| Latence régime établi | **~130–160 ms** |
| RTF (Real-Time Factor) | **~0.04–0.09** |
| Overhead callback ROS2 | < 0.5 ms |

> RTF = 0.04 → le GPU génère **25× plus vite** que le temps réel. Valeurs sous Mode0, sans jetson_clocks

---

## Architecture

```
Thread ROS  (rclcpp::spin)
  └── speak_callback()
        ├── timestamp received_at
        ├── push TextRequest → g_text_queue
        └── retour immédiat  ← thread ROS jamais bloqué

WorkerThread  (thread dédié, unique)
  ├── Pa_OpenStream()       [une seule fois au démarrage]
  └── boucle :
        ├── wait(g_text_queue)                    [CPU idle entre les phrases]
        ├── SherpaOnnxOfflineTtsGenerateWithConfig()  [inférence GPU complète]
        ├── Pa_StartStream()                      [~2–5 ms]
        ├── Pa_WriteStream()                      [lecture bloquante]
        └── Pa_StopStream()                       [drain garanti → phrase complète]
```

### Pourquoi pas de streaming GPU→audio ?

Le GPU génère à RTF ~0.04 (25× temps réel). Appeler `Pa_WriteStream()` depuis le
callback d'inférence bloquerait le GPU en attente du DAC USB → **latence plus élevée**.
Générer tout l'audio puis jouer en un bloc est optimal dans cette configuration.

### Pourquoi Pa_OpenStream() hors de la boucle ?

Sur USB audio, `Pa_OpenStream()` coûte ~35 ms (négociation USB, allocation DMA).
Maintenu ouvert en permanence, seul `Pa_StartStream()` (~2–5 ms) est appelé
à chaque phrase → **économie de ~30 ms par phrase**.

---

## Topics

| Topic | Type | Rôle |
|---|---|---|
| `/to_speak` | `std_msgs/String` | Texte à synthétiser (abonné) |

### Exemple d'utilisation

```bash
ros2 topic pub --once /to_speak std_msgs/msg/String "data: 'Bonjour, je suis Néo !'"
```

---

## Paramètres ROS2

| Paramètre | Type | Défaut | Description |
|---|---|---|---|
| `model_path` | string | `install/.../axel.onnx` | Modèle VITS (.onnx) |
| `tokens_path` | string | `install/.../tokens.txt` | Phonèmes espeak-ng |
| `espeak_data_dir` | string | `install/.../espeak-ng-data` | Données espeak-ng |
| `num_threads` | int | `3` | Threads CPU sherpa-onnx |
| `provider` | string | `"cuda"` | Backend ONNX : `"cuda"` ou `"cpu"` |
| `sid` | int | `0` | Speaker ID (0 pour modèles mono-locuteur) |
| `speed` | double | `1.0` | Vitesse de parole (0.5–2.0) | -> 0.8 = plus lent !

`sid` et `speed` sont relus à **chaque message** → modifiables à chaud :

```bash
ros2 param set /qbo_tts_node speed 1.2
```

---

## Structure des fichiers

```
qbo_tts/
├── src/
│   └── qbo_tts_node.cpp      # node principal
├── models/                   # (non versionné, copié manuellement)
│   ├── axel.onnx             # modèle VITS français fine-tuné
│   ├── tokens.txt            # table des phonèmes
│   └── espeak-ng-data/       # données phonétiques espeak-ng
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Dépendances

| Bibliothèque | Version testée | Notes |
|---|---|---|
| ROS2 | Humble | |
| sherpa-onnx | ≥ 1.10 | API `SherpaOnnxOfflineTtsGenerateWithConfig` requise |
| ONNX Runtime | CUDA 12.6 | Fourni avec sherpa-onnx JetPack |
| PortAudio | 19 | `sudo apt install libportaudio2 portaudio19-dev` |
| CUDA | 12.6 (JetPack 6.2) | |

---

## Build

```bash
cd ~/qbo_ws
colcon build --packages-select qbo_tts
source install/setup.bash
```

---

## Lancement

ros2 run qbo_tts qbo_tts_node

```bash
ros2 run qbo_tts qbo_tts_node \
  --ros-args \
  -p model_path:=/chemin/vers/axel.onnx \
  -p tokens_path:=/chemin/vers/tokens.txt \
  -p espeak_data_dir:=/chemin/vers/espeak-ng-data \
  -p provider:=cuda \
  -p speed:=1.0
```

Ou via un fichier launch avec les paramètres dans un `.yaml`.

---

## Logs de debug

À chaque phrase, deux lignes sont émises si décommentées:

```
[TTS latency] overhead: 0.1 ms | inference: 148.0 ms | total avant audio: 148.1 ms | duree audio: 3587 ms | RTF: 0.041
[TTS latency] reception -> debut audio : 150.3 ms
```

| Champ | Description |
|---|---|
| `overhead` | Temps ROS callback → début inférence (doit être < 1 ms) |
| `inference` | Durée inférence GPU pure |
| `total avant audio` | overhead + inference = latence perçue |
| `duree audio` | Durée de la phrase générée (ms) |
| `RTF` | Real-Time Factor — plus bas = meilleur |
| `reception → debut audio` | Latence end-to-end topic → premier son DAC |

---

## Troubleshooting

**ALSA `Invalid card 'card'`**
Message inoffensif d'ALSA lors de l'énumération des devices.
Vérifier que PulseAudio est démarré : `pulseaudio --start`

**Audio vide / pas de son**
Vérifier que le device USB audio est le device par défaut :
```bash
pactl info | grep "Default Sink"
speaker-test -D plughw:1,0 -t sine
```

**Cold start > 1 s**
Normal à la première phrase (compilation JIT des kernels CUDA).
Solution : faire dire "Bonjour" à Néo au démarrage du système.

**`provider=cuda` → erreur ONNX Runtime**
Vérifier la version CUDA/cuDNN avec `nvcc --version`.
Fallback : `provider:=cpu` (latence ~3–5× plus élevée).

install de sherpa_onnx

cmake \
  -DSHERPA_ONNX_LINUX_ARM64_GPU_ONNXRUNTIME_VERSION=1.18.1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DSHERPA_ONNX_ENABLE_GPU=ON \
  -DSHERPA_ONNX_ENABLE_TRT=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DSHERPA_ONNX_ENABLE_PORTAUDIO=ON \
  -DSHERPA_ONNX_ENABLE_PYTHON=OFF \
  -DONNXRUNTIME_DIR=/usr/local \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  ..
  