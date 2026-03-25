installer les dépendances :

chmod +x install_dependencies.sh
./install_dependencies.sh

pip3 install onnxruntime-gpu --index-url https://pypi.jetson-ai-lab.io/jp6/cu126


# TROUVER L INDEX DU MICROPHONE
import sounddevice as sd
import yaml

# Recherche du device micro
MIC_KEYWORDS = ['respeaker', 'lite']
device_index = None

for i, dev in enumerate(sd.query_devices()):
    name = dev['name'].lower()
    if any(key in name for key in MIC_KEYWORDS) and dev['max_input_channels'] > 0:
        device_index = i
        break

if device_index is None:
    raise RuntimeError("Micro ReSpeaker Lite USB Audio non trouvé.")

print(f"Micro trouvé : index={device_index}, nom='{sd.query_devices()[device_index]['name']}'")