"""
ROS2 node that captures audio, streams to NVIDIA Riva for speech-to-text,
and publishes recognized text on the "/listened" topic.
"""
import queue
import threading
import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import riva.client
import time

class SpeechListenerNode(Node):
    """
    ROS2 node for capturing audio from microphone and publishing recognized speech.
    Publishes String messages on the /listened topic.
    --> if : UNKNOWN: ipv4:127.0.0.1:50051: Failed to connect to remote host
    --> Riva docker isn't started
    """
    def __init__(self):
        super().__init__('speech_listener')
        
        # Flag pour contrôler l'arrêt propre
        self._shutdown_requested = False
        
        # Flag pour contrôler l'activation/désactivation du micro
        self._audio_enabled = True
        self._audio_lock = threading.Lock()
        
        # Déclaration des paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('server_uri', 'localhost:50051'),
                ('use_ssl', False),
                ('ssl_cert', ''),
                ('language_code', 'fr-FR'),
                ('model_name', 'conformer-fr-FR-asr-streaming-asr-bls-ensemble'),
                ('sample_rate', 16000),
                ('chunk_size', 1600),
                ('input_device', 0),  # Corrigé pour correspondre au launch file
            ]
        )

        # Récupération des paramètres
        self.SERVER_URI = self.get_parameter('server_uri').value
        self.USE_SSL = self.get_parameter('use_ssl').value
        self.SSL_CERT = self.get_parameter('ssl_cert').value or None
        self.LANGUAGE_CODE = self.get_parameter('language_code').value
        self.MODEL_NAME = self.get_parameter('model_name').value
        self.SAMPLE_RATE = self.get_parameter('sample_rate').value
        self.CHUNK_SIZE = self.get_parameter('chunk_size').value
        self.INPUT_DEVICE_INDEX = self.get_parameter('input_device').value

        # Publisher vers /listened
        self.publisher_ = self.create_publisher(String, '/listened', 1)
        
        # Subscriber pour le contrôle audio
        # ros2 topic pub -1 /is_audio_recording std_msgs/Bool "data: true"
        self.audio_control_subscriber = self.create_subscription(
            Bool,
            '/is_audio_recording',
            self._audio_control_callback, 10  # 10Hz
        )

        # Initialisation Riva avec gestion d'erreur
        try:
            auth = riva.client.Auth(
                ssl_cert=self.SSL_CERT,
                use_ssl=self.USE_SSL,
                uri=self.SERVER_URI
            )
            self.asr_service = riva.client.ASRService(auth)
            self.config = riva.client.StreamingRecognitionConfig(
                config=riva.client.RecognitionConfig(
                    encoding=riva.client.AudioEncoding.LINEAR_PCM,
                    language_code=self.LANGUAGE_CODE,
                    model=self.MODEL_NAME,
                    max_alternatives=2,
                    profanity_filter=False,
                    enable_automatic_punctuation=False,
                    verbatim_transcripts=False,
                    sample_rate_hertz=self.SAMPLE_RATE,
                    audio_channel_count=1,
                ),
                interim_results=False,
            )
            self.get_logger().info(f'Connexion à Riva réussie sur {self.SERVER_URI}')
        except Exception as e:
            self.get_logger().error(f'Erreur de connexion à Riva: {e}')
            raise

        # Queue et stream audio
        self.q = queue.Queue()
        self.stream = None
        
        try:
            self.stream = sd.InputStream(
                samplerate=self.SAMPLE_RATE,
                blocksize=self.CHUNK_SIZE,
                device=self.INPUT_DEVICE_INDEX,
                dtype='float32',
                channels=1,
                callback=self._audio_callback,
            )
            self.stream.start()
            self.get_logger().info(f'Audio stream démarré (device index: {self.INPUT_DEVICE_INDEX})')
        except Exception as e:
            self.get_logger().error(f'Erreur lors du démarrage du stream audio: {e}')
            raise

        # Démarrer le thread d'écoute
        self._listen_thread = threading.Thread(target=self._listen_and_publish, daemon=True)
        self._listen_thread.start()

    def _audio_control_callback(self, msg):
        """Callback pour le contrôle audio via topic"""
        with self._audio_lock:
            self._audio_enabled = msg.data
            # Si l'audio est désactivé, vider complètement la queue
            if not self._audio_enabled:
                self._clear_audio_queue()
            status = "activé" if self._audio_enabled else "désactivé"
            #self.get_logger().info(f"Audio {status}")

    def audio_control(self, enable):
        """Méthode pour contrôler l'activation/désactivation du micro"""
        with self._audio_lock:
            self._audio_enabled = enable
            # Si l'audio est désactivé, vider complètement la queue
            if not self._audio_enabled:
                self._clear_audio_queue()
            status = "activé" if self._audio_enabled else "désactivé"
            #self.get_logger().info(f"Audio {status} (contrôle direct)")

    def _clear_audio_queue(self):
        """Vide complètement la queue audio"""
        cleared_count = 0
        try:
            while not self.q.empty():
                self.q.get_nowait()
                cleared_count += 1
        except queue.Empty:
            pass
        
        if cleared_count > 0:
            self.get_logger().info(f"Queue audio vidée : {cleared_count} chunks supprimés")

    def _audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"[sounddevice] {status}")
        
        # Ne pas traiter si l'arrêt est demandé
        if self._shutdown_requested:
            return
        
        # Vérifier si l'audio est activé
        with self._audio_lock:
            if not self._audio_enabled:
                return  # Ignorer les chunks audio si désactivé
            
        pcm = (indata * 32767).astype(np.int16).tobytes()
        try:
            self.q.put(pcm, timeout=0.1)
        except queue.Full:
            #self.get_logger().warn("Queue audio pleine, abandon du chunk")
            pass

    def _listen_and_publish(self):
        try:
            audio_gen = self._audio_generator()
            responses = self.asr_service.streaming_response_generator(
                audio_chunks=audio_gen,
                streaming_config=self.config,
            )
            self.get_logger().info('Début transcription en streaming...')
            
            for response in responses:
                if self._shutdown_requested:
                    break
                    
                for result in response.results:
                    if result.is_final:
                        transcript = result.alternatives[0].transcript.strip()
                        if transcript:
                            msg = String()
                            msg.data = transcript
                            self.publisher_.publish(msg)
                            #self.get_logger().info(f"Publié: {transcript}")
                            
        except Exception as e:
            if not self._shutdown_requested:
                self.get_logger().error(f"Erreur lors de l'écoute: {e}")

    def _audio_generator(self):
        while rclpy.ok() and not self._shutdown_requested:
            try:
                yield self.q.get(timeout=1.0)
            except queue.Empty:
                continue

    def destroy_node(self):
        """Nettoyage propre des ressources"""
        self.get_logger().info("Arrêt du node STT Neo demandé...")
        
        # Signaler l'arrêt
        self._shutdown_requested = True
        
        # Arrêter le stream audio
        if hasattr(self, 'stream') and self.stream:
            try:
                self.stream.stop()
                self.stream.close()
                #self.get_logger().info("Stream audio arrêté")
            except Exception as e:
                self.get_logger().warn(f"Erreur lors de l'arrêt du stream: {e}")
        
        # Vider la queue
        try:
            while not self.q.empty():
                self.q.get_nowait()
        except queue.Empty:
            pass
        
        # Attendre que le thread se termine (avec timeout)
        if hasattr(self, '_listen_thread') and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=2.0)
            if self._listen_thread.is_alive():
                self.get_logger().warn("Le thread d'écoute n'a pas pu être arrêté proprement")
        
        super().destroy_node()
        self.get_logger().info("Node STT Neo arrêté correctement !")


def main():
    rclpy.init()
    node = None
    try:
        node = SpeechListenerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Arrêt par l\'utilisateur.')
    except Exception as e:
        if node:
            node.get_logger().error(f'Erreur fatale: {e}')
        else:
            print(f'Erreur lors de l\'initialisation: {e}')
    finally:
        # Nettoyer proprement
        if node:
            node.destroy_node()
        
        # Shutdown seulement si rclpy est encore actif
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()