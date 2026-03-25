#!/usr/bin/env python3
"""
ROS 2 node : utilise Sherpa-ONNX (GPU) pour convertir la parole en texte.
Le texte est publié sur le topic "/listened".
"""

import os
import queue
import threading
import time
import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sherpa_onnx

class SherpaSpeechListener(Node):
    def __init__(self):
        super().__init__('QBO_listener')

        # === Paramètres du modèle Sherpa-ONNX ===
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_base_path', '/home/nvidia/qbo_ws/src/qbo_stt/stt_model/sherpa_models'),
                ('sample_rate', 16000),
                ('frame_duration', 0.1),
                ('input_device', 0)
            ]
        )

        base_path = self.get_parameter('model_base_path').value
        self.SAMPLE_RATE = self.get_parameter('sample_rate').value
        self.FRAME_DURATION = self.get_parameter('frame_duration').value
        self.SAMPLES_PER_READ = int(self.SAMPLE_RATE * self.FRAME_DURATION)
        self.INPUT_DEVICE = self.get_parameter('input_device').value

        # === Chargement du modèle Sherpa-ONNX ===
        try:
            self.recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                tokens=os.path.join(base_path, 'tokens.txt'),
                encoder=os.path.join(base_path, 'encoder-epoch-29-avg-9-with-averaged-model.int8.onnx'),
                decoder=os.path.join(base_path, 'decoder-epoch-29-avg-9-with-averaged-model.int8.onnx'),
                joiner=os.path.join(base_path, 'joiner-epoch-29-avg-9-with-averaged-model.int8.onnx'),
                sample_rate=self.SAMPLE_RATE,
                feature_dim=80,
                enable_endpoint_detection=True,
                decoding_method="greedy_search",
                max_active_paths=2,
                rule1_min_trailing_silence=2.4,
                rule2_min_trailing_silence=0.3,
                rule3_min_utterance_length=300,
                provider="cuda",  # GPU
                num_threads=1,
            )
            self.get_logger().info("✅ Modèle Sherpa-ONNX chargé (GPU)")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur chargement modèle Sherpa : {e}")
            raise

        # Publisher texte reconnu
        self.pub = self.create_publisher(String, '/listened', 10)

        # Contrôle micro via topic
        self.audio_enabled = True
        self.audio_lock = threading.Lock()
        self.create_subscription(Bool, '/is_audio_recording', self._audio_control_callback, 10)

        # Stream audio et thread d'écoute
        self.stream = None
        self.shutdown_requested = False
        self.listen_thread = threading.Thread(target=self._run_recognition, daemon=True)
        self.listen_thread.start()

    def _audio_control_callback(self, msg):
        with self.audio_lock:
            self.audio_enabled = msg.data
            status = "activé" if self.audio_enabled else "désactivé"
            self.get_logger().info(f"🎙️ Micro {status}")

    def _run_recognition(self):
        try:
            with sd.InputStream(
                samplerate=self.SAMPLE_RATE,
                channels=1,
                dtype="float32",
                device=self.INPUT_DEVICE,
                blocksize=self.SAMPLES_PER_READ,
            ) as mic:

                stream = self.recognizer.create_stream()
                self.get_logger().info("🎤 Reconnaissance en cours...")

                while rclpy.ok() and not self.shutdown_requested:
                    with self.audio_lock:
                        if not self.audio_enabled:
                            time.sleep(0.1)
                            continue

                    samples, _ = mic.read(self.SAMPLES_PER_READ)
                    stream.accept_waveform(self.SAMPLE_RATE, samples.ravel())

                    while self.recognizer.is_ready(stream):
                        self.recognizer.decode_stream(stream)

                    if self.recognizer.is_endpoint(stream):
                        text = self.recognizer.get_result(stream).strip()
                        if text:
                            msg = String()
                            msg.data = text
                            self.pub.publish(msg)
                            self.get_logger().info(f"🗣️ {text}")
                        self.recognizer.reset(stream)
                        stream = self.recognizer.create_stream()
        except Exception as e:
            if not self.shutdown_requested:
                self.get_logger().error(f"❌ Erreur dans _run_recognition : {e}")

    def destroy_node(self):
        self.get_logger().info("🛑 Arrêt du nœud STT Sherpa demandé...")
        self.shutdown_requested = True
        if self.listen_thread.is_alive():
            self.listen_thread.join(timeout=2.0)
        super().destroy_node()
        self.get_logger().info("✅ Node Sherpa STT arrêté proprement.")

def main():
    rclpy.init()
    node = None
    try:
        node = SherpaSpeechListener()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('🛑 Interruption utilisateur.')
    except Exception as e:
        if node:
            node.get_logger().error(f"Erreur fatale : {e}")
        else:
            print(f"Erreur lors de l'init : {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
