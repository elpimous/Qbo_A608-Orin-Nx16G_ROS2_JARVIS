# QBO TTS - Nœud ROS2 Text-to-Speech pour Robot Néo

Nœud ROS2 de synthèse vocale utilisant **Piper TTS** avec streaming audio en temps réel.
Optimisé pour Jetson Orin NX avec sortie USB Audio Device.

## Fonctionnalités

- Synthèse vocale française avec modèle Piper ONNX
- Streaming audio temps réel (faible latence)
- Resampling automatique (22050 Hz → 48000 Hz)
- Mute/unmute automatique du microphone pendant la parole
- Configuration via YAML

---

## Installation

### Prérequis

```bash
# Dépendances Python
pip install piper-tts sounddevice scipy numpy --break-system-packages

# Dépendances système (ALSA)
sudo apt install libasound2-dev portaudio19-dev
```

### Cloner et compiler

```bash
cd ~/qbo_ws/src
# (si pas déjà cloné)
git clone <repo_url> qbo_tts

cd ~/qbo_ws
colcon build --packages-select qbo_tts
source install/setup.bash
```

### Modèle Piper

Placer le modèle ONNX et son fichier JSON dans `tts_model/` :

```
qbo_tts/
├── tts_model/
│   ├── axel.onnx
│   └── axel.onnx.json
```

Télécharger des voix françaises : https://github.com/rhasspy/piper/releases

---

## Configuration de la carte son

### Étape 1 : Lister les périphériques ALSA

```bash
aplay -L
```

Exemple de sortie :
```
hw:CARD=Device,DEV=0    USB Audio Device, USB Audio
plughw:CARD=Device,DEV=0    USB Audio Device, USB Audio
```

### Étape 2 : Lister les périphériques sounddevice (Python)

```bash
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

Exemple de sortie :
```
  0 USB Audio Device: - (hw:0,0), ALSA (1 in, 2 out)   <-- Index 0
  1 sysdefault, ALSA (128 in, 128 out)
  2 front, ALSA (0 in, 2 out)
...
* 12 default, ALSA (32 in, 32 out)
```

**Noter l'index** du périphérique USB Audio (ici `0`).

### Étape 3 : Tester la sortie audio

```bash
# Test avec ALSA
aplay -D plughw:CARD=Device,DEV=0 /usr/share/sounds/alsa/Front_Center.wav

# Test avec sounddevice (remplacer device=0 par votre index)
python3 -c "
import sounddevice as sd
import numpy as np
sr = 48000
t = np.linspace(0, 0.5, int(sr*0.5))
tone = (np.sin(2*np.pi*440*t) * 0.3).astype(np.float32)
sd.play(tone, sr, device=0)
sd.wait()
print('OK')
"
```

### Étape 4 : Configurer le fichier YAML

Éditer `config/tts.yaml` avec le bon index :

```yaml
qbo_tts_node:
  ros__parameters:
    output_device_index: 0  # <-- Votre index ici
```

---

## Utilisation

### Lancer le nœud

```bash
ros2 launch qbo_tts qbo_tts_launch.py
```

### Publier du texte à synthétiser

```bash
ros2 topic pub -1 /to_speak std_msgs/msg/String "{data: 'Bonjour, je suis Néo'}"
```

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/to_speak` | `std_msgs/String` | Input | Texte à synthétiser |
| `/is_audio_recording` | `std_msgs/Bool` | Output | Contrôle du micro (False=mute) |

### Paramètres ROS2

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `model_path` | string | `.../axel.onnx` | Chemin du modèle Piper |
| `config_path` | string | `.../axel.onnx.json` | Chemin config JSON |
| `output_device_index` | int | `0` | Index carte son (sounddevice) |

---

## Architecture du nœud

```
┌─────────────────────────────────────────────────────────────┐
│                    TTSListenerNode                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /to_speak ──► [Callback] ──► [Queue] ──► [Worker Thread]  │
│                                                │            │
│                                                ▼            │
│                                      ┌─────────────────┐    │
│                                      │ PiperTTSStreamer│    │
│                                      ├─────────────────┤    │
│                                      │ synthesize()    │    │
│                                      │ resample()      │    │
│                                      │ playback()      │    │
│                                      └────────┬────────┘    │
│                                               │             │
│                                               ▼             │
│                                      [USB Audio Device]     │
│                                                             │
│  /is_audio_recording ◄── [Mute/Unmute pendant parole]      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Flux de données

1. Message reçu sur `/to_speak`
2. Ajout à la queue interne
3. Worker thread récupère le texte
4. Mute du microphone (`/is_audio_recording` = False)
5. Synthèse Piper → chunks audio
6. Resampling 22050 Hz → 48000 Hz
7. Buffering (50ms) pour éviter les coupures
8. Playback via sounddevice
9. Unmute du microphone

---

## Tests

### Test 1 : Vérifier le nœud

```bash
ros2 node list | grep tts
# Doit afficher: /tts_
```

### Test 2 : Vérifier le topic

```bash
ros2 topic info /to_speak
# Type: std_msgs/msg/String
# Subscription count: 1
```

### Test 3 : Test de synthèse

```bash
ros2 topic pub -1 /to_speak std_msgs/msg/String "{data: 'Test de synthèse vocale'}"
```

### Test 4 : Test Piper standalone (sans ROS)

```bash
python3 -c "
from piper import PiperVoice
import numpy as np
import sounddevice as sd
from scipy.signal import resample

voice = PiperVoice.load('/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx')

audio_data = []
for chunk in voice.synthesize('Bonjour, je suis Néo'):
    if chunk:
        audio_data.append(chunk.audio_float_array)

audio = np.concatenate(audio_data)
resampled = resample(audio, int(len(audio) * 48000 / 22050))
sd.play(resampled.astype(np.float32), 48000, device=0)
sd.wait()
print('OK')
"
```

---

## Dépannage

### Pas de son

1. Vérifier l'index de la carte son :
   ```bash
   python3 -c "import sounddevice as sd; print(sd.query_devices())"
   ```

2. Tester avec aplay :
   ```bash
   aplay -D plughw:CARD=Device,DEV=0 /usr/share/sounds/alsa/Front_Center.wav
   ```

3. Vérifier le paramètre dans `config/tts.yaml`

### Erreur "PiperVoice has no attribute..."

Vérifier la version de piper-tts :
```bash
pip show piper-tts
# Version >= 1.4.0 requise
```

### Coupures audio

Augmenter le buffer dans le code :
```python
self.buffer_size_ms = 100  # Au lieu de 50
```

### Warning ONNX GPU

Le warning `GPU device discovery failed` est normal sur Jetson - Piper utilise le CPU par défaut.

---

## Auteur

**Vincent FOUCAULT** - Mars 2026

## Licence

MIT
