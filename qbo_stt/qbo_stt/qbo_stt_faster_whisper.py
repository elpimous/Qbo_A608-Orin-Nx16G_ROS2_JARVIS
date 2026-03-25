import os
import queue
import threading
import time
from collections import deque
import numpy as np
import sounddevice as sd
import webrtcvad
from faster_whisper import WhisperModel
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# -------------------------------------------------------------------------------------------------
# Fonction utilitaire : recherche et sélection du périphérique audio
# -------------------------------------------------------------------------------------------------
def find_device(name_hint: str):
    name_hint = name_hint.lower()
    for idx, dev in enumerate(sd.query_devices()):
        if dev.get("max_input_channels", 0) < 1:
            continue
        if name_hint in dev["name"].lower():
            rate = int(dev.get("default_samplerate", 16000))
            print(f"✨ Found '{dev['name']}' at index {idx}, rate={rate}")
            return idx, rate
    dev = sd.query_devices(kind="input")
    rate = int(dev.get("default_samplerate", 16000))
    return None, rate

# -------------------------------------------------------------------------------------------------
# Classe ROS2 : écoute, détection de parole, reconnaissance Whisper, publication STT
# -------------------------------------------------------------------------------------------------
class ListenNode(Node):
    TARGET_RATE = 16000
    FRAME_MS = 20
    CHANNELS = 1
    BLOCKSIZE = int(TARGET_RATE * FRAME_MS / 1000)

    def __init__(self):
        super().__init__("qbo_stt_optimized")

        # Déclaration des paramètres ROS2
        self.declare_parameter("audio_in_device_name", "default")
        self.declare_parameter("mic_volume_percent", 70)
        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("remove_punctuation", True)
        self.declare_parameter("enable_vad_filter", False)
        self.declare_parameter("beam_size", 1)
        self.declare_parameter("max_audio_length", 10.0)

        # Lecture des paramètres
        self.lang               = self.get_parameter("system_lang").value
        self.remove_punctuation = self.get_parameter("remove_punctuation").value
        self.enable_vad_filter  = self.get_parameter("enable_vad_filter").value
        self.beam_size          = self.get_parameter("beam_size").value
        self.max_audio_length   = self.get_parameter("max_audio_length").value
        mic_volume              = self.get_parameter("mic_volume_percent").value

        # Regex pour nettoyage de la ponctuation
        if self.remove_punctuation:
            self.punct_pattern = re.compile(r'[^\w\s]')

        # Recherche du périphérique audio
        device_hint = self.get_parameter("audio_in_device_name").value
        self.device_index, rate = find_device(device_hint)
        self.samplerate = self.TARGET_RATE

        # Publisher STT
        self.result_pub = self.create_publisher(String, "/listened", 10)

        # Control audio via topic
        self.audio_lock = threading.Lock()
        self.audio_enabled = True
        self.create_subscription(Bool, "/is_audio_recording", self._audio_control_callback, 10)

        # Réglage volume micro (PulseAudio)
        try:
            import subprocess
            subprocess.run([
                "pactl", "set-source-volume",
                "alsa_input.platform-sound.analog-stereo",
                f"{mic_volume}%"
            ], check=True)
            self.get_logger().info(f"🔊 Micro volume réglé à {mic_volume}% via pactl")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Impossible de régler le volume du micro : {e}")

        # Chargement du modèle Whisper optimisé
        self.get_logger().info("🔄 Chargement du modèle Whisper...")
        self.voice_model = WhisperModel(
            self.get_parameter("whisper_model").value,
            device="cuda",
            compute_type="int8_float16",
            cpu_threads=2,
        )
        self.get_logger().info("✅ Modèle Whisper chargé.")

        # File d'attente audio
        self.buffer_queue = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()

        # Démarrage du thread d'écoute
        self.audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.audio_thread.start()

    def _audio_control_callback(self, msg: Bool):
        """Active/désactive l'écoute audio via topic"""
        with self.audio_lock:
            self.audio_enabled = msg.data
        status = "activé" if msg.data else "désactivé"
        self.get_logger().info(f"🎤 Audio {status}")

    def _clean_text(self, text: str) -> str:
        """Nettoie le texte selon les paramètres"""
        if self.remove_punctuation:
            text = self.punct_pattern.sub('', text)
        return text.strip()

    def _listen_loop(self):
        """Boucle principale : lecture audio, VAD, transcription, publication"""
        vad = webrtcvad.Vad(1)
        with sd.InputStream(
            device=self.device_index,
            samplerate=self.samplerate,
            blocksize=self.BLOCKSIZE,
            channels=self.CHANNELS,
            dtype='float32',
            latency='low'
        ) as stream:
            self.get_logger().info("🎹 Démarrage du streaming audio...")
            frames_buffer = deque()

            while not self.stop_event.is_set():
                with self.audio_lock:
                    if not self.audio_enabled:
                        time.sleep(0.1)
                        continue

                try:
                    data, overflowed = stream.read(self.BLOCKSIZE)
                    if overflowed:
                        continue  # Ignorer les blocs overflow
                except Exception as e:
                    self.get_logger().warning(f"⚠️ Erreur de lecture du stream : {e}")
                    continue

                pcm = (data[:, 0] * 32768).astype(np.int16).tobytes()
                if vad.is_speech(pcm, self.samplerate):
                    frames_buffer.append(data[:, 0])
                elif frames_buffer:
                    audio_np = np.concatenate(frames_buffer)
                    frames_buffer.clear()

                    max_samples = int(self.max_audio_length * self.samplerate)
                    if len(audio_np) > max_samples:
                        audio_np = audio_np[-max_samples:]

                    start_time = time.time()
                    segments, _ = self.voice_model.transcribe(
                        audio_np,
                        language=self.lang,
                        beam_size=self.beam_size,
                        vad_filter=self.enable_vad_filter,
                        word_timestamps=False,
                        condition_on_previous_text=False,
                        temperature=0.0,
                        compression_ratio_threshold=2.4,
                        log_prob_threshold=-1.0,
                        no_speech_threshold=0.6,
                    )
                    text = " ".join(s.text for s in segments).strip()
                    text = self._clean_text(text)
                    elapsed = time.time() - start_time

                    if text:
                        msg = String()
                        msg.data = text
                        self.result_pub.publish(msg)
                        self.get_logger().info(f"📝 '{text}' (⏱️ {elapsed:.2f}s)")

    def destroy_node(self):
        """Arrêt propre du node et du thread"""
        self.get_logger().info("⏹️ Arrêt du node STT demandé...")
        self.stop_event.set()
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2.0)
        super().destroy_node()
        self.get_logger().info("✅ Node STT arrêté.")


def main(args=None):
    rclpy.init(args=args)
    node = ListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Keyboard interrupt reçu")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
