#!/usr/bin/env python3
"""
ROS2 Talker TTS Node with Streaming - Version corrigée pour USB Audio Device
Conserve le streaming pour une réponse rapide et fluide
Corrections principales :
- Resampling correct vers 48000 Hz (supporté par ta carte)
- Gestion optimisée des buffers pour éviter les coupures
- Streaming temps réel maintenu
*** Vincent FOUCAULT - Août 2025 ***
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import queue
import time
import numpy as np
import sounddevice as sd
from scipy.signal import resample
from piper import PiperVoice

class PiperTTSStreamer:
    """
    Classe de streaming TTS utilisant Piper et sounddevice.
    Version corrigée avec resampling fonctionnel pour 48000 Hz.
    """
    def __init__(self, model_path: str, config_path: str = None, device_index: int = 1):
        self.device_index = device_index
        
        # Force 48000 Hz (mieux supporté que 44100 sur ta carte)
        self.target_sr = 48000
        
        # Vérification du device
        device_info = sd.query_devices(self.device_index)
        print(f"[TTS] Device: {device_info['name']}")
        print(f"[TTS] Target sample rate: {self.target_sr} Hz")
        
        # Test que le device supporte bien 48000 Hz
        try:
            sd.check_output_settings(device=self.device_index, samplerate=self.target_sr)
            print(f"[TTS] ✓ {self.target_sr} Hz supporté")
        except:
            print(f"[TTS] Fallback à 44100 Hz")
            self.target_sr = 44100
            
        # Chargement du modèle Piper
        print("[TTS] Chargement du modèle Piper...")
        self.voice = PiperVoice.load(model_path, config_path)
        self.sample_rate = self.voice.config.sample_rate
        print(f"[TTS] Modèle chargé - Sample rate natif: {self.sample_rate} Hz")
        
        # Calcul du ratio de resampling
        self.resample_ratio = self.target_sr / self.sample_rate
        print(f"[TTS] Ratio de resampling: {self.resample_ratio:.2f}x ({self.sample_rate} → {self.target_sr} Hz)")
        
        # Queue pour le streaming
        self.audio_queue = queue.Queue()
        self.is_playing = threading.Event()
        
        # Buffer pour accumulation (évite les micro-coupures)
        self.buffer_size_ms = 50  # 50ms de buffer
        
    def _resample_chunk(self, chunk: np.ndarray) -> np.ndarray:
        """
        Resample un chunk audio vers le target sample rate.
        Utilise scipy.signal.resample pour une meilleure qualité.
        """
        if self.sample_rate == self.target_sr:
            return chunk
            
        # Calcul du nombre d'échantillons après resampling
        input_samples = len(chunk)
        output_samples = int(input_samples * self.resample_ratio)
        
        if output_samples == 0:
            return np.array([], dtype=np.float32)
            
        # Resampling avec scipy (meilleure qualité que resample_poly pour ce cas)
        resampled = resample(chunk, output_samples)
        
        return resampled.astype(np.float32)

    def _enqueue_audio(self, text: str) -> None:
        """
        Synthétise le texte et ajoute les chunks resampleés dans la file.
        Streaming temps réel maintenu.
        """
        try:
            # Utilisation du streaming de Piper
            stream = self.voice.synthesize_stream_raw(text)
            
            for chunk in stream:
                if chunk is None:
                    continue
                    
                # Conversion int16 -> float32 normalisé
                if hasattr(chunk, 'numpy'):
                    arr = chunk.numpy()
                else:
                    arr = np.frombuffer(chunk, dtype=np.int16)
                
                # Normalisation
                data_float = arr.astype(np.float32) / 32768.0
                
                # Resampling vers le target sample rate
                data_resampled = self._resample_chunk(data_float)
                
                if len(data_resampled) > 0:
                    self.audio_queue.put(data_resampled)
                    
        except Exception as e:
            print(f"[TTS] Erreur synthèse: {e}")
        finally:
            # Signal de fin de flux pour ce texte
            self.audio_queue.put(None)

    def _playback(self) -> None:
        """
        Thread de lecture streaming avec buffer optimisé.
        Accumule quelques chunks pour éviter les coupures.
        """
        buffer = []
        buffer_samples = int(self.target_sr * self.buffer_size_ms / 1000)
        
        print(f"[TTS] Thread playback démarré (buffer: {self.buffer_size_ms}ms)")
        
        while self.is_playing.is_set():
            try:
                chunk = self.audio_queue.get(timeout=0.5)
            except queue.Empty:
                # Si on a du buffer en attente et timeout, on le joue
                if buffer:
                    combined = np.concatenate(buffer)
                    try:
                        sd.play(combined, self.target_sr, device=self.device_index)
                        sd.wait()
                    except Exception as e:
                        print(f"[TTS] Erreur playback (timeout): {e}")
                    buffer = []
                continue
                
            if chunk is None:
                # Fin d'un texte - on joue ce qui reste dans le buffer
                if buffer:
                    combined = np.concatenate(buffer)
                    try:
                        sd.play(combined, self.target_sr, device=self.device_index)
                        sd.wait()
                    except Exception as e:
                        print(f"[TTS] Erreur playback (fin): {e}")
                    buffer = []
                continue
            
            # Accumulation dans le buffer
            buffer.append(chunk)
            
            # Calcul de la taille totale du buffer
            total_samples = sum(len(b) for b in buffer)
            
            # Si buffer assez grand, on joue
            if total_samples >= buffer_samples:
                combined = np.concatenate(buffer)
                try:
                    # Joue le buffer accumulé
                    sd.play(combined, self.target_sr, device=self.device_index)
                    sd.wait()
                except Exception as e:
                    print(f"[TTS] Erreur playback: {e}")
                buffer = []
        
        print("[TTS] Thread playback terminé")

    def speak(self, text_list: list) -> None:
        """
        Parle une liste de textes en streaming.
        Le premier chunk commence à jouer dès qu'il est prêt.
        """
        print(f"[TTS] Streaming de {len(text_list)} message(s)...")
        
        # Nettoyage de la queue
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break

        # Démarrage du thread de playback
        self.is_playing.set()
        th_play = threading.Thread(target=self._playback, daemon=True)
        th_play.start()

        # Synthèse en streaming de chaque texte
        for i, text in enumerate(text_list, 1):
            print(f"[TTS] Message {i}/{len(text_list)}: '{text[:50]}...'")
            self._enqueue_audio(text)
        
        # Attente que tout soit joué
        time.sleep(0.1)  # Court délai pour le dernier buffer
        self.is_playing.clear()
        th_play.join(timeout=10.0)
        print("[TTS] Streaming terminé")

    def stop(self) -> None:
        """
        Arrête la lecture et vide les buffers.
        """
        self.is_playing.clear()
        sd.stop()
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break


class TTSListenerNode(Node):
    """
    Nœud ROS2 pour streaming TTS sur robot Néo.
    Version optimisée avec resampling fonctionnel.
    """
    def __init__(self):
        super().__init__('QBO_talker')
        
        # Paramètres ROS2
        self.declare_parameter('model_path', '/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx')
        self.declare_parameter('config_path', '/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx.json')
        self.declare_parameter('output_device_index', 1)

        model = self.get_parameter('model_path').get_parameter_value().string_value
        config = self.get_parameter('config_path').get_parameter_value().string_value
        device_index = self.get_parameter('output_device_index').get_parameter_value().integer_value

        # Publisher pour contrôler l'audio du micro
        self.audio_control_publisher = self.create_publisher(
            Bool,
            '/is_audio_recording', 
            10
        )

        # Initialisation TTS avec streaming
        self.get_logger().info("Initialisation TTS avec streaming...")
        try:
            self.tts = PiperTTSStreamer(model, config, device_index)
            self.get_logger().info("✓ TTS streaming initialisé")
        except Exception as e:
            self.get_logger().error(f"✗ Échec init TTS: {e}")
            raise

        # File pour regrouper les textes
        self.text_queue = queue.Queue()
        self.running = True
        
        # Thread de traitement
        self.worker = threading.Thread(target=self._process_queue, daemon=True)
        self.worker.start()

        # Souscription au topic 'to_speak'
        self.subscription = self.create_subscription(
            String,
            'to_speak',
            self.listener_callback,
            10
        )
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("TTS Streaming prêt sur /to_speak")
        self.get_logger().info(f"Device: {device_index}, Mode: STREAMING")
        self.get_logger().info("=" * 50)

    def listener_callback(self, msg: String) -> None:
        """
        Callback pour réception des messages.
        """
        text = msg.data.strip()
        if not text:
            return
            
        self.get_logger().info(f"Message reçu: '{text[:60]}...'")
        self.text_queue.put(text)

    def _mute_microphone(self):
        """Désactive le microphone"""
        try:
            msg = Bool()
            msg.data = False
            self.audio_control_publisher.publish(msg)
        except:
            pass

    def _unmute_microphone(self):
        """Réactive le microphone"""
        try:
            msg = Bool()
            msg.data = True
            self.audio_control_publisher.publish(msg)
        except:
            pass

    def _process_queue(self) -> None:
        """
        Thread de traitement avec batching intelligent.
        """
        self.get_logger().info("Thread de streaming TTS démarré")
        
        while self.running and rclpy.ok():
            try:
                # Attente du premier message
                text = self.text_queue.get(timeout=1.0)
            except queue.Empty:
                continue
                
            if not self.running:
                break
                
            # Collecte rapide des messages supplémentaires
            batch = [text]
            time.sleep(0.01)  # Très court délai pour batching
            
            while not self.text_queue.empty() and len(batch) < 10:
                try:
                    batch.append(self.text_queue.get_nowait())
                except queue.Empty:
                    break

            self.get_logger().info(f"Streaming de {len(batch)} message(s)")
            
            # Mute micro avant de parler
            self._mute_microphone()
            time.sleep(0.1)

            try:
                # Streaming TTS
                self.tts.speak(batch)
                time.sleep(0.25)  # Délai de sécurité
            except Exception as e:
                self.get_logger().error(f"Erreur TTS: {e}")
            finally:
                # Unmute micro
                self._unmute_microphone()
        
        self.get_logger().info("Thread de streaming arrêté")

    def destroy_node(self):
        """
        Arrêt propre du nœud.
        """
        self.get_logger().info("Arrêt du nœud TTS...")
        self.running = False
        
        if hasattr(self, 'tts'):
            self.tts.stop()
        
        self._unmute_microphone()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSListenerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé (Ctrl+C)")
    except Exception as e:
        node.get_logger().error(f"Erreur: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()