#!/usr/bin/env python3
"""
╔═════════════════════════════════════════════════╗
║   qbo_stt — Speech-To-Text Node — Robot Néo     ║
║   NVIDIA Parakeet TDT 0.6B v3 — TensorRT FP16   ║
║   VAD : Silero ONNX (CPU)                       ║
║   Author : Vincent Foucault — Avril 2026        ║
╚═════════════════════════════════════════════════╝

Topics publiés :
  /listened              (std_msgs/String) — texte transcrit
Topics souscrits :
  /is_listening    (std_msgs/Bool)   — True = écoute active
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool

import numpy as np
import threading
import queue
import time
import os

# ── TensorRT ──────────────────────────────────────────────────────────────────
import tensorrt as trt
import pycuda.driver as cuda
cuda.init()   # init globale uniquement — make_context() dans le thread inference

# ── ONNX Runtime — Silero VAD uniquement ──────────────────────────────────────
import onnxruntime as ort

# ── Audio ─────────────────────────────────────────────────────────────────────
import pyaudio
import librosa

# ── Chemins par défaut ────────────────────────────────────────────────────────
ENGINE_DIR     = '/models/engine_model'
ENCODER_ENGINE = os.path.join(ENGINE_DIR, 'encoder.engine')
DECODER_ENGINE = os.path.join(ENGINE_DIR, 'decoder_joint.engine')
VOCAB_FILE     = os.path.join(ENGINE_DIR, 'vocab.txt')
VAD_MODEL      = os.path.join(ENGINE_DIR, 'silero_vad.onnx')

# ── Constantes NeMo mel ───────────────────────────────────────────────────────
SR         = 16000
N_FFT      = 512
HOP_LENGTH = 160
WIN_LENGTH = 320
N_MELS     = 128
FMIN       = 0
FMAX       = 8000

# ── Décodage TDT ──────────────────────────────────────────────────────────────
DURATIONS = [0, 1, 2, 3, 4]


# ══════════════════════════════════════════════════════════════════════════════
#  TRT Engine — buffers GPU pré-alloués, shape-aware
# ══════════════════════════════════════════════════════════════════════════════
class TRTEngine:
    """
    Wrapper TensorRT avec pré-allocation des buffers GPU.
    Réallocation uniquement si la shape change → zéro malloc
    au runtime pour le decoder (shape toujours fixe 1x1).
    """
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    def __init__(self, path: str):
        with open(path, 'rb') as f:
            self.engine  = trt.Runtime(self.TRT_LOGGER).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream  = cuda.Stream()
        self._d_in   = {}
        self._d_out  = {}
        self._h_out  = {}

    def infer(self, inputs: dict) -> dict:
        # ── Host → Device ─────────────────────────────────────────────────────
        for name, arr in inputs.items():
            arr = np.ascontiguousarray(arr)
            self.context.set_input_shape(name, arr.shape)
            if name not in self._d_in or self._d_in[name][1] != arr.nbytes:
                if name in self._d_in:
                    self._d_in[name][0].free()
                self._d_in[name] = (cuda.mem_alloc(arr.nbytes), arr.nbytes)
            cuda.memcpy_htod_async(self._d_in[name][0], arr, self.stream)

        # ── Allocation outputs ────────────────────────────────────────────────
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                shape  = tuple(self.context.get_tensor_shape(name))
                dtype  = trt.nptype(self.engine.get_tensor_dtype(name))
                nbytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
                if name not in self._d_out or self._d_out[name][1] != nbytes:
                    if name in self._d_out:
                        self._d_out[name][0].free()
                    self._d_out[name] = (cuda.mem_alloc(nbytes), nbytes)
                    self._h_out[name] = np.empty(shape, dtype=dtype)

        # ── Set tensor addresses ──────────────────────────────────────────────
        for name, (d, _) in self._d_in.items():
            self.context.set_tensor_address(name, int(d))
        for name, (d, _) in self._d_out.items():
            self.context.set_tensor_address(name, int(d))

        # ── Inférence async ───────────────────────────────────────────────────
        self.context.execute_async_v3(self.stream.handle)

        # ── Device → Host ─────────────────────────────────────────────────────
        for name, (d, _) in self._d_out.items():
            cuda.memcpy_dtoh_async(self._h_out[name], d, self.stream)
        self.stream.synchronize()

        return {name: self._h_out[name] for name in self._d_out}

    def destroy(self):
        """Libérer tous les buffers GPU et le stream AVANT ctx.pop()."""
        self.stream.synchronize()
        for _, (d, _) in self._d_in.items():
            d.free()
        for _, (d, _) in self._d_out.items():
            d.free()
        self._d_in.clear()
        self._d_out.clear()
        self._h_out.clear()
        del self.context
        del self.engine


# ══════════════════════════════════════════════════════════════════════════════
#  Silero VAD — ONNX Runtime CPU
#  Entrées : x [1,512]  h [2,1,64]  c [2,1,64]
#  Sorties : prob [1,1]  new_h  new_c
# ══════════════════════════════════════════════════════════════════════════════
class SileroVAD:

    def __init__(self, model_path: str, threshold: float = 0.5):
        self.threshold = threshold
        self.session   = ort.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']  # CPU → GPU libre pour ASR TRT
        )
        self._h = np.zeros((2, 1, 64), dtype=np.float32)
        self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def reset(self):
        self._h = np.zeros((2, 1, 64), dtype=np.float32)
        self._c = np.zeros((2, 1, 64), dtype=np.float32)

    def is_speech(self, chunk: np.ndarray) -> bool:
        """chunk : float32 [512] à 16kHz → True si parole détectée"""
        out             = self.session.run(None, {
            'x': chunk[np.newaxis].astype(np.float32),
            'h': self._h,
            'c': self._c,
        })
        prob, self._h, self._c = out[0], out[1], out[2]
        return float(prob[0][0]) >= self.threshold


# ══════════════════════════════════════════════════════════════════════════════
#  Mel features — NeMo-compatible, 100% librosa/numpy
# ══════════════════════════════════════════════════════════════════════════════
def extract_mel_nemo(wav: np.ndarray) -> tuple:
    mel = librosa.feature.melspectrogram(
        y=wav, sr=SR, n_fft=N_FFT, hop_length=HOP_LENGTH,
        win_length=WIN_LENGTH, window='hann', n_mels=N_MELS,
        fmin=FMIN, fmax=FMAX, power=2.0, center=True,
    )
    mel  = np.log(mel + 2 ** -24)
    mean = mel.mean(axis=1, keepdims=True)
    std  = mel.std(axis=1, keepdims=True)
    mel  = (mel - mean) / (std + 1e-5)
    mel  = mel[np.newaxis].astype(np.float32)

    # ── Padding si trop court pour l'engine (minShapes=20 frames) ────────────
    MIN_FRAMES = 20  # 0.2s → principalement pour détecter "oui" ou "non"
    if mel.shape[2] < MIN_FRAMES:
        pad = MIN_FRAMES - mel.shape[2]
        mel = np.pad(mel, ((0,0),(0,0),(0,pad)), mode='constant')

    # ── Clip max engine (1000 frames ≈ 10s) ──────────────────────────────────
    if mel.shape[2] > 1000:
        mel = mel[:, :, :1000]

    length = np.array([mel.shape[2]], dtype=np.int64)
    return mel, length


# ══════════════════════════════════════════════════════════════════════════════
#  Parakeet ASR — encoder + greedy TDT decoder, 100% TRT
# ══════════════════════════════════════════════════════════════════════════════
class ParakeetASR:

    def __init__(self, encoder: TRTEngine, decoder: TRTEngine, vocab: dict):
        self.encoder    = encoder
        self.decoder    = decoder
        self.vocab      = vocab
        self.blank_id   = max(vocab.keys())
        self.vocab_size = self.blank_id + 1

    def transcribe(self, wav: np.ndarray) -> tuple:
        """wav : float32 mono 16kHz → (texte, rtf)"""
        t0 = time.perf_counter()

        mel, length = extract_mel_nemo(wav)
        enc_out     = self.encoder.infer({'audio_signal': mel, 'length': length})
        tokens      = self._greedy_decode(enc_out['outputs'])
        text        = self._tokens_to_text(tokens)

        dt  = time.perf_counter() - t0
        dur = len(wav) / SR
        return text, (dt / dur if dur > 0 else 0.0)

    def _greedy_decode(self, enc_out: np.ndarray) -> list:
        T      = enc_out.shape[2]
        tokens = []
        state1  = np.zeros((2, 1, 640), dtype=np.float16)
        state2  = np.zeros((2, 1, 640), dtype=np.float16)
        target  = np.array([[self.blank_id]], dtype=np.int64)
        tgt_len = np.array([1], dtype=np.int64)
        t = 0
        while t < T:
            out = self.decoder.infer({
                'encoder_outputs': enc_out[:, :, t:t+1],
                'targets':         target,
                'target_length':   tgt_len,
                'input_states_1':  state1,
                'input_states_2':  state2,
            })
            logits = out['outputs'][0, 0, 0]
            state1 = out['output_states_1']
            state2 = out['output_states_2']
            pred_token = int(np.argmax(logits[:self.vocab_size]))
            pred_dur   = DURATIONS[int(np.argmax(logits[self.vocab_size:]))]
            if pred_token == self.blank_id:
                t += max(pred_dur, 1)
            else:
                tokens.append(pred_token)
                target = np.array([[pred_token]], dtype=np.int64)
                t += pred_dur
        return tokens

    def _tokens_to_text(self, tokens: list) -> str:
        return ''.join(self.vocab.get(t, '') for t in tokens).replace('▁', ' ').strip()


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 STT Node
# ══════════════════════════════════════════════════════════════════════════════
class STTNode(Node):

    def __init__(self):
        super().__init__('stt_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Paramètres ROS2 ───────────────────────────────────────────────────
        self.declare_parameter('encoder_engine',     ENCODER_ENGINE)
        self.declare_parameter('decoder_engine',     DECODER_ENGINE)
        self.declare_parameter('vocab_file',         VOCAB_FILE)
        self.declare_parameter('vad_model',          VAD_MODEL)
        self.declare_parameter('vad_threshold',      0.5)
        self.declare_parameter('vad_min_silence_ms', 600)
        self.declare_parameter('vad_min_speech_ms',  200)
        self.declare_parameter('vad_max_speech_s',   10.0)
        self.declare_parameter('vad_pre_buffer_ms',  256)
        self.declare_parameter('audio_device_index', -1)
        self.declare_parameter('chunk_size',         512)

        enc_path   = os.path.expanduser(self.get_parameter('encoder_engine').value)
        dec_path   = os.path.expanduser(self.get_parameter('decoder_engine').value)
        vocab_path = os.path.expanduser(self.get_parameter('vocab_file').value)
        vad_path   = os.path.expanduser(self.get_parameter('vad_model').value)
        vad_thr    = self.get_parameter('vad_threshold').value

        self.min_silence   = self.get_parameter('vad_min_silence_ms').value / 1000.0
        self.min_speech    = self.get_parameter('vad_min_speech_ms').value  / 1000.0
        self.max_speech    = self.get_parameter('vad_max_speech_s').value
        self.pre_buffer_ms = self.get_parameter('vad_pre_buffer_ms').value  / 1000.0
        self.device_index  = self.get_parameter('audio_device_index').value
        self.chunk_size    = self.get_parameter('chunk_size').value

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub          = self.create_publisher(String, '/listened',            10)
        # ── Subscription micro ────────────────────────────────────────────────
        self.audio_enabled = True
        self.audio_lock    = threading.Lock()
        self.create_subscription(
            Bool, '/is_listening',
            self._audio_control_callback, 10,
            callback_group=self.cb_group
        )

        # ── État interne ──────────────────────────────────────────────────────
        self.audio_queue     = queue.Queue(maxsize=4)
        self._running        = True
        self._models_ready   = threading.Event()
        self._inference_done = threading.Event()

        # ── Chemins stockés pour le thread inference ──────────────────────────
        self._enc_path   = enc_path
        self._dec_path   = dec_path
        self._vocab_path = vocab_path
        self._vad_path   = vad_path
        self._vad_thr    = vad_thr

        # ── Lancer inference en premier (crée le contexte CUDA) ──────────────
        threading.Thread(target=self._inference_loop, daemon=True).start()

        # ── Attendre modèles prêts avant d'ouvrir le micro ───────────────────
        if not self._models_ready.wait(timeout=60.0):
            self.get_logger().error('Timeout chargement modèles !')
            return

        threading.Thread(target=self._audio_capture_loop, daemon=True).start()

    # ──────────────────────────────────────────────────────────────────────────
    def _audio_control_callback(self, msg: Bool):
        with self.audio_lock:
            self.audio_enabled = msg.data
        state = '🎙️  activé' if msg.data else '🔇 désactivé'
        self.get_logger().info(f'Micro {state}')

    # ──────────────────────────────────────────────────────────────────────────
    def _audio_capture_loop(self):
        pa     = pyaudio.PyAudio()
        kwargs = dict(
            format=pyaudio.paInt16, channels=1,
            rate=SR, input=True, frames_per_buffer=self.chunk_size,
        )
        if self.device_index >= 0:
            kwargs['input_device_index'] = self.device_index

        stream = pa.open(**kwargs)
        self.get_logger().info(f'🎙️  Micro ouvert (device={self.device_index})')

        chunk_dur      = self.chunk_size / SR
        silence_needed = int(self.min_silence   / chunk_dur)
        speech_min     = int(self.min_speech    / chunk_dur)
        speech_max     = int(self.max_speech    / chunk_dur)
        pre_buf_max    = int(self.pre_buffer_ms / chunk_dur)  # ~8 chunks = 256ms

        speech_buffer  = []
        pre_buffer     = []   # ring buffer pré-parole → évite de couper le début des mots
        silence_frames = 0
        speech_frames  = 0
        in_speech      = False
        self.vad.reset()

        while self._running:
            with self.audio_lock:
                enabled = self.audio_enabled

            if not enabled:
                try:
                    stream.read(self.chunk_size, exception_on_overflow=False)
                except Exception:
                    pass
                speech_buffer = []; pre_buffer = []
                speech_frames = silence_frames = 0
                if in_speech:
                    in_speech = False
                    self.vad.reset()
                time.sleep(0.05)
                continue

            try:
                raw   = stream.read(self.chunk_size, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                is_sp = self.vad.is_speech(chunk)

                if is_sp:
                    silence_frames  = 0
                    speech_frames  += 1
                    speech_buffer.append(chunk)

                    if not in_speech and speech_frames >= speech_min:
                        in_speech     = True
                        # Préfixer le segment avec les chunks pré-parole
                        speech_buffer = list(pre_buffer) + speech_buffer
                        pre_buffer    = []

                    # Sécurité durée max
                    if speech_frames >= speech_max:
                        self._flush(speech_buffer)
                        speech_buffer = []; pre_buffer = []
                        speech_frames = silence_frames = 0
                        in_speech     = False
                        self.vad.reset()

                else:
                    if in_speech:
                        silence_frames += 1
                        speech_buffer.append(chunk)
                        if silence_frames >= silence_needed:
                            self._flush(speech_buffer)
                            speech_buffer = []; pre_buffer = []
                            speech_frames = silence_frames = 0
                            in_speech     = False
                            self.vad.reset()
                    else:
                        # Maintenir le ring buffer pré-parole
                        pre_buffer.append(chunk)
                        if len(pre_buffer) > pre_buf_max:
                            pre_buffer.pop(0)
                        speech_buffer = []
                        speech_frames = 0

            except Exception as e:
                self.get_logger().warn(f'Audio error : {e}')

        stream.stop_stream()
        stream.close()
        pa.terminate()

    # ──────────────────────────────────────────────────────────────────────────
    def _flush(self, buffer: list):
        if len(buffer) < 2:
            return
        wav = np.concatenate(buffer)
        try:
            self.audio_queue.put_nowait(wav)
        except queue.Full:
            self.get_logger().warn('Queue pleine — segment ignoré')

    # ──────────────────────────────────────────────────────────────────────────
    def _inference_loop(self):
        """Tout CUDA vit et meurt dans ce thread uniquement."""
        ctx     = cuda.Device(0).make_context()
        encoder = None
        decoder = None
        try:
            self.get_logger().info('⚡ Chargement engines TensorRT...')
            encoder = TRTEngine(self._enc_path)
            self.get_logger().info(f'  ✅ Encoder : {os.path.basename(self._enc_path)}')
            decoder = TRTEngine(self._dec_path)
            self.get_logger().info(f'  ✅ Decoder : {os.path.basename(self._dec_path)}')
            vocab = self._load_vocab(self._vocab_path)
            self.get_logger().info(f'  ✅ Vocab   : {len(vocab)} tokens')
            self.asr = ParakeetASR(encoder, decoder, vocab)
            self.vad = SileroVAD(self._vad_path, self._vad_thr)
            self.get_logger().info(f'  ✅ VAD     : {os.path.basename(self._vad_path)}')

            # ── Warmup — élimine le RTF anormal sur la 1ère transcription ────
            self.get_logger().info('🔥 Warmup TRT...')
            self.asr.transcribe(np.zeros(SR, dtype=np.float32))
            self.get_logger().info('✅ STT prêt — en écoute')
            self._models_ready.set()

            while self._running:
                try:
                    wav = self.audio_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                text, rtf = self.asr.transcribe(wav)
                dur = len(wav) / SR
                if text:
                    self.get_logger().info(f'💬 [{dur:.2f}s | RTF={rtf:.3f}] → {text}')
                    msg      = String()
                    msg.data = text
                    self.pub.publish(msg)
                else:
                    self.get_logger().debug('Segment vide — ignoré')

        except Exception as e:
            self.get_logger().error(f'Inference thread fatal : {e}')
            self._models_ready.set()  # débloquer __init__ même en cas d'erreur
        finally:
            # ── Libérer TRT explicitement AVANT ctx.pop() ─────────────────────
            # Sans ça, le GC Python détruit les engines après ctx.pop()
            # → "context is destroyed" / Myelin stream error → segfault
            if encoder is not None:
                encoder.destroy()
            if decoder is not None:
                decoder.destroy()
            ctx.synchronize()
            ctx.pop()
            ctx.detach()
            self._inference_done.set()

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _load_vocab(path: str) -> dict:
        vocab = {}
        with open(path) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    vocab[int(parts[1])] = parts[0]
        return vocab

    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        # Attendre que le thread inference ait nettoyé CUDA proprement
        self._inference_done.wait(timeout=3.0)
        # Aucun appel CUDA ici — tout est géré dans _inference_loop
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()