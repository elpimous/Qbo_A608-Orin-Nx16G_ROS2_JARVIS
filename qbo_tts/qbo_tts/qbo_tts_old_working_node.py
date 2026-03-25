#!/usr/bin/env python3
"""
ROS2 Listener TTS Node

Ce noeud ROS2 écoute un topic de type String, reçoit du texte,
et utilise PiperTTSStreamer pour synthétiser et jouer la parole via sounddevice.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import queue
import time
import numpy as np
import sounddevice as sd
from scipy.signal import resample_poly
from piper import PiperVoice

class PiperTTSStreamer:
    """
    Classe de streaming TTS utilisant Piper et sounddevice.
    Optimisée pour un flux continu et une lecture stable.
    """
    def __init__(self, model_path: str, config_path: str = None, device_index: int = 1):
        self.device_index = device_index
        # Mesure du temps de chargement du modèle
        start = time.time()
        print("[TTS] Chargement du modèle...\n")
        self.voice = PiperVoice.load(model_path, config_path)
        self.model_load_time_ms = (time.time() - start) * 1000
        
        # File pour buffer audio
        self.audio_queue: queue.Queue = queue.Queue()
        self.is_playing = False
        self.sample_rate = self.voice.config.sample_rate

        print(f"[TTS] Modèle chargé en {self.model_load_time_ms:.2f} ms, sr={self.sample_rate}\n")

    def _synthesize(self, text: str, start_time: float):
        """
        Génère des chunks audio et les pousse dans la file.
        """
        try:
            stream = self.voice.synthesize_stream_raw(text)
            first_chunk = True
            for chunk in stream:
                if chunk is None:
                    continue
                # Convertir en numpy float32 [-1,1]
                data = chunk.numpy() if hasattr(chunk, 'numpy') else np.frombuffer(chunk, dtype=np.int16)
                data = data.astype(np.float32) / 32768.0
                if first_chunk:
                    delta = (time.time() - start_time) * 1000
                    #print(f"[TTS] Premier audio après {delta:.2f} ms")
                    first_chunk = False
                self.audio_queue.put(data)
        except Exception as e:
            print(f"[TTS] Erreur synthèse: {e}")
        finally:
            # Marquer fin de flux
            self.audio_queue.put(None)

    def _play(self, start_time: float):
        """
        Lit les chunks depuis la file.
        """
        first_play = True
        #print("[TTS] Démarrage lecture...")
        while self.is_playing:
            try:
                chunk = self.audio_queue.get(timeout=1.0)
                if chunk is None:
                    break
                if first_play:
                    delta = (time.time() - start_time) * 1000
                    print(f"[TTS] Début parole après {delta:.2f} ms")
                    first_play = False
                # Si la fréquence du modèle ≠ celle du périphérique, on la convertit
                target_sample_rate = 44100
                if self.sample_rate != target_sample_rate:
                    # Ratio d'interpolation (ex: 2 = 44100/22050)
                    chunk = resample_poly(chunk, target_sample_rate, self.sample_rate)
                    rate = target_sample_rate
                else:
                    rate = self.sample_rate

                sd.play(np.clip(chunk * 1.0, -1.0, 1.0), rate, device=self.device_index)
                sd.wait()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[TTS] Erreur playback: {e}")
                break
        #print("[TTS] Lecture terminée")

    def speak(self, text: str):
        """
        Lance la synthèse et la lecture en threads séparés.
        """
        # Nettoyer file
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break

        self.is_playing = True
        t0 = time.time()
        t1 = threading.Thread(target=self._synthesize, args=(text, t0))
        t2 = threading.Thread(target=self._play, args=(t0,))
        t1.start(); t2.start()
        t1.join(); t2.join()
        self.is_playing = False
        #print("[TTS] Fin parole")

    def stop(self):
        """
        Arrête la lecture en cours.
        """
        self.is_playing = False
        sd.stop()


class TTSListenerNode(Node):
    """
    Noeud ROS2 pour écouter un topic et parler le texte reçu.
    """
    def __init__(self):
        super().__init__('tts_listener')
        # Récupérer paramètres de chemin modèle
        self.declare_parameter('model_path', '/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx')
        self.declare_parameter('config_path', '/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx.json')
        model = self.get_parameter('model_path').get_parameter_value().string_value
        config = self.get_parameter('config_path').get_parameter_value().string_value
        self.declare_parameter('output_device_index', 1)
        device_index = self.get_parameter('output_device_index').get_parameter_value().integer_value


        # Initialiser le streamer TTS
        try:
            self.tts = PiperTTSStreamer(model, config, device_index)
        except Exception as e:
            self.get_logger().error(f"Échec initialisation TTS: {e}")
            raise

        # Souscription au topic 'tts_input'
        self.subscription = self.create_subscription(
            String,
            'tts_input',
            self.listener_callback,
            10)
        self.get_logger().info("TTS Listener initialisé, en attente de messages...\n")

    def listener_callback(self, msg: String):
        """
        Callback appelé à chaque message reçu.
        Lance la synthèse et la lecture du texte.
        """
        text = msg.data.strip()
        if not text:
            self.get_logger().warn('Message vide reçu, rien à dire.')
            return
        self.get_logger().info(f"Message reçu: '{text}'")

        # On lance la synthèse vocale
        thread = threading.Thread(target=self.tts.speak, args=(text,))
        thread.daemon = True
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = TTSListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur")
    finally:
        node.tts.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
