/**
 * @file    qbo_tts_node.cpp
 * @brief   Node ROS2 TTS pour le robot Néo (plateforme QBo)
 *
 * Synthèse vocale française offline via sherpa-onnx (moteur VITS, accélération
 * CUDA sur Jetson Orin NX 16 Go) et lecture audio via PortAudio (USB audio).
 *
 * ── Architecture ──────────────────────────────────────────────────────────────
 *
 *   Thread ROS (rclcpp::spin)
 *     └── speak_callback()
 *           Reçoit le texte sur /to_speak, l'empile dans g_text_queue,
 *           et retourne IMMÉDIATEMENT. Le thread ROS n'est jamais bloqué.
 *
 *   WorkerThread (thread dédié, unique)
 *     └── attend g_text_queue (condition_variable)
 *     └── inférence GPU complète  →  SherpaOnnxOfflineTtsGenerateWithConfig()
 *     └── lecture audio           →  Pa_WriteStream() bloquant
 *     └── traite les requêtes une par une dans l'ordre FIFO
 *
 * ── Choix de conception ───────────────────────────────────────────────────────
 *
 *   • Inférence complète avant lecture (pas de streaming GPU→audio) :
 *     Le GPU génère à RTF ~0.04 (25× temps réel), donc toute la phrase est prête
 *     en ~150 ms. Le streaming n'apporte rien ici et sérialiserait le GPU avec
 *     les I/O audio (Pa_WriteStream bloquant dans le callback = pire latence).
 *
 *   • Pa_OpenStream() une seule fois au démarrage du WorkerThread :
 *     Sur USB audio, Pa_OpenStream() coûte ~35 ms. On le garde ouvert en
 *     permanence et on n'utilise que Pa_StartStream/StopStream autour de
 *     chaque phrase → overhead réduit à ~2–5 ms par phrase.
 *
 *   • Pa_StopStream() après Pa_WriteStream() :
 *     Pa_StopStream() draine le buffer audio hardware avant de retourner.
 *     La phrase suivante ne commence qu'après que la précédente est
 *     intégralement jouée → comportement déterministe garanti.
 *
 * ── Performances mesurées (Jetson Orin NX, USB audio C-Media) ─────────────
 *
 *   Cold start (1ère phrase)  : ~470 ms  (JIT CUDA, inévitable)
 *   Régime établi             : ~130–160 ms avant le premier son
 *   RTF moyen                 : 0.04–0.09
 *   Overhead ROS callback     : < 0.5 ms
 *
 * ── Topics ───────────────────────────────────────────────────────────────────
 *
 *   Abonné  /to_speak  [std_msgs/String]  Texte à synthétiser
 *
 * ── Paramètres ROS2 ──────────────────────────────────────────────────────────
 *
 *   model_path       Chemin vers le fichier .onnx du modèle VITS
 *   tokens_path      Chemin vers tokens.txt (phonèmes espeak-ng)
 *   espeak_data_dir  Répertoire espeak-ng-data
 *   num_threads      Nombre de threads CPU pour sherpa-onnx (défaut : 3)
 *   provider         Backend ONNX Runtime : "cuda" ou "cpu" (défaut : "cuda")
 *   sid              Speaker ID pour modèles multi-locuteurs (défaut : 0)
 *   speed            Vitesse de parole, 1.0 = normale (défaut : 1.0)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <portaudio.h>
#include "sherpa-onnx/c-api/c-api.h"

#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <string>
#include <chrono>

// ─── Requête texte ────────────────────────────────────────────────────────────
// Regroupe toutes les informations nécessaires pour générer et jouer une phrase.
// Le timestamp received_at permet de mesurer la latence end-to-end.
struct TextRequest {
  std::string                           text;         // texte à synthétiser
  int32_t                               sid;          // speaker ID
  float                                 speed;        // vitesse de parole
  std::chrono::steady_clock::time_point received_at;  // horodatage réception topic
};

// ─── File d'attente texte thread-safe ────────────────────────────────────────
// Partagée entre speak_callback() (producteur) et WorkerThread() (consommateur).
// Le mutex protège la queue ; la condition_variable évite le busy-wait.
struct TextQueue {
  std::queue<TextRequest>  queue;
  std::mutex               mutex;
  std::condition_variable  cv;
};

static TextQueue         g_text_queue;   // file partagée globale
static std::atomic<bool> g_killed{false}; // signal d'arrêt propre du WorkerThread

// ─── WorkerThread ─────────────────────────────────────────────────────────────
/**
 * @brief Thread unique responsable de l'inférence GPU et de la lecture audio.
 *
 * Boucle infinie :
 *   1. Attend une requête dans g_text_queue (bloquant, CPU idle entre les phrases)
 *   2. Lance l'inférence GPU via SherpaOnnxOfflineTtsGenerateWithConfig()
 *   3. Joue l'audio complet via Pa_WriteStream() (bloquant)
 *   4. Passe à la requête suivante
 *
 * @param tts         Handle sherpa-onnx initialisé dans QboTTSNode
 * @param sample_rate Fréquence d'échantillonnage du modèle (typiquement 22050 Hz)
 */
static void WorkerThread(const SherpaOnnxOfflineTts* tts, int32_t sample_rate) {

  Pa_Initialize();

  // Déterminer le device de sortie par défaut (USB audio C-Media sur Néo)
  PaDeviceIndex device = Pa_GetDefaultOutputDevice();
  if (device == paNoDevice) {
    RCLCPP_ERROR(rclcpp::get_logger("qbo_tts"),
                 "Aucun device audio de sortie trouvé !");
    Pa_Terminate();
    return;
  }

  // Paramètres PortAudio — initialisés une seule fois, réutilisés à chaque phrase.
  // channelCount = 1  : VITS génère du mono
  // sampleFormat      : float32 natif sherpa-onnx, pas de conversion nécessaire
  // suggestedLatency  : latence minimale du device (pas hardcodée à 0)
  PaStreamParameters pa_params{};
  pa_params.device           = device;
  pa_params.channelCount     = 1;
  pa_params.sampleFormat     = paFloat32;
  pa_params.suggestedLatency = Pa_GetDeviceInfo(device)->defaultLowOutputLatency;

  // Ouvrir le stream PortAudio UNE SEULE FOIS.
  // Sur USB audio, Pa_OpenStream() coûte ~35 ms (négociation USB, allocation DMA).
  // En le gardant ouvert, seul Pa_StartStream() (~2–5 ms) est appelé par phrase.
  // nullptr, nullptr = mode bloquant (pas de callback asynchrone)
  // framesPerBuffer = 512 : taille des blocs DMA (~23 ms @ 22050 Hz)
  PaStream* pa_stream = nullptr;
  PaError pa_err = Pa_OpenStream(&pa_stream, nullptr, &pa_params,
                                  sample_rate, 512, paClipOff,
                                  nullptr, nullptr);
  if (pa_err != paNoError) {
    RCLCPP_ERROR(rclcpp::get_logger("qbo_tts"),
                 "Pa_OpenStream failed: %s", Pa_GetErrorText(pa_err));
    Pa_Terminate();
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
              "WorkerThread démarré — device: %s",
              Pa_GetDeviceInfo(device)->name);

  while (!g_killed) {

    // ── Étape 1 : attendre une requête ───────────────────────────────────────
    // condition_variable.wait() libère le mutex et bloque le thread jusqu'à ce
    // que speak_callback() appelle notify_one(). CPU idle entre les phrases.
    TextRequest req;
    {
      std::unique_lock<std::mutex> lock(g_text_queue.mutex);
      g_text_queue.cv.wait(lock, [] {
        return !g_text_queue.queue.empty() || g_killed.load();
      });
      if (g_killed) break;

      // Récupérer la requête la plus ancienne (FIFO)
      req = std::move(g_text_queue.queue.front());
      g_text_queue.queue.pop();
    }

    // ── Étape 2 : inférence GPU complète ─────────────────────────────────────
    // SherpaOnnxOfflineTtsGenerateWithConfig() utilise le backend CUDA (via
    // ONNX Runtime) et génère TOUT l'audio avant de retourner.
    // Avantage : le GPU n'est jamais interrompu → RTF optimal (~0.04).
    // callback=nullptr : pas de streaming, on attend la fin complète.
    auto t_infer_start = std::chrono::steady_clock::now();

    SherpaOnnxGenerationConfig gen_cfg{};
    gen_cfg.sid             = req.sid;
    gen_cfg.speed           = req.speed;
    gen_cfg.silence_scale   = 0.2f;    // vince : silence inter-phrases (secondes)
    gen_cfg.reference_audio = nullptr; // pas de voice cloning

    const SherpaOnnxGeneratedAudio* audio =
      SherpaOnnxOfflineTtsGenerateWithConfig(
        tts, req.text.c_str(), &gen_cfg, nullptr, nullptr);

    auto t_infer_end = std::chrono::steady_clock::now();

    // Vérification : audio vide = texte non reconnu ou erreur modèle
    if (!audio || audio->n == 0) {
      RCLCPP_WARN(rclcpp::get_logger("qbo_tts"),
                  "Audio vide pour : \"%s\"", req.text.c_str());
      if (audio) SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
      continue;
    }

    // ── Étape 3 : métriques de latence ───────────────────────────────────────
    // overhead  = temps entre réception topic et début inférence (overhead ROS)
    // inference = durée pure de l'inférence GPU
    // total     = overhead + inference = latence avant le premier son
    // RTF       = inference / duree_audio  (< 1.0 = plus rapide que temps réel)
    double ms_overhead  = std::chrono::duration<double, std::milli>(
                            t_infer_start - req.received_at).count();
    double ms_inference = std::chrono::duration<double, std::milli>(
                            t_infer_end - t_infer_start).count();
    double ms_total     = std::chrono::duration<double, std::milli>(
                            t_infer_end - req.received_at).count();
    double audio_ms     = (static_cast<double>(audio->n) / sample_rate) * 1000.0;
    double rtf          = ms_inference / audio_ms;

    /*
    RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
      "[TTS latency] overhead: %.1f ms | inference: %.1f ms | "
      "total avant audio: %.1f ms | duree audio: %.0f ms | RTF: %.3f",
      ms_overhead, ms_inference, ms_total, audio_ms, rtf);
    */

    // ── Étape 4 : lecture audio ───────────────────────────────────────────────
    // Pa_StartStream() : active le hardware (~2–5 ms, sans réouvrir le stream)
    // Pa_WriteStream() : envoie tous les samples en un bloc, bloquant jusqu'au
    //                    dernier sample consommé par le DAC
    // Pa_StopStream()  : draine le buffer hardware → retourne seulement quand
    //                    la phrase est intégralement jouée
    Pa_StartStream(pa_stream);

    // Log de la latence totale réception → premier son
    /*
    RCLCPP_INFO(rclcpp::get_logger("qbo_tts"),
      "[TTS latency] reception -> debut audio : %.1f ms",
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - req.received_at).count());
    */

    PaError err = Pa_WriteStream(pa_stream, audio->samples,
                                  static_cast<unsigned long>(audio->n));
    if (err != paNoError && err != paOutputUnderflowed) {
      RCLCPP_WARN(rclcpp::get_logger("qbo_tts"),
                  "Pa_WriteStream: %s", Pa_GetErrorText(err));
    }

    // Drain garanti : la phrase suivante ne commence qu'après celle-ci
    Pa_StopStream(pa_stream);

    // Libérer la mémoire audio allouée par sherpa-onnx
    SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);

  } // fin while(!g_killed)

  // Nettoyage PortAudio à l'arrêt du node
  Pa_CloseStream(pa_stream);
  Pa_Terminate();
}

// ─── QboTTSNode ───────────────────────────────────────────────────────────────
/**
 * @brief Node ROS2 de synthèse vocale pour Néo.
 *
 * Initialise le modèle TTS sherpa-onnx, démarre WorkerThread,
 * et s'abonne à /to_speak pour empiler les requêtes.
 */
class QboTTSNode : public rclcpp::Node {
public:
  QboTTSNode() : Node("qbo_tts_node") {

    // Déclaration des paramètres ROS2 avec valeurs par défaut
    // Les chemins sont relatifs à la racine du workspace après colcon build
    this->declare_parameter<std::string>("model_path",
      "install/qbo_tts/share/qbo_tts/models/axel.onnx");
    this->declare_parameter<std::string>("tokens_path",
      "install/qbo_tts/share/qbo_tts/models/tokens.txt");
    this->declare_parameter<std::string>("espeak_data_dir",
      "install/qbo_tts/share/qbo_tts/models/espeak-ng-data");
    this->declare_parameter<int>("num_threads", 3);      // threads CPU sherpa-onnx
    this->declare_parameter<std::string>("provider", "cuda"); // "cuda" ou "cpu"
    this->declare_parameter<int>("sid", 0);              // speaker ID (modèle mono = 0)
    this->declare_parameter<double>("speed", 1.0);       // 1.0 = vitesse normale

    std::string model_path  = this->get_parameter("model_path").as_string();
    std::string tokens_path = this->get_parameter("tokens_path").as_string();
    std::string espeak_dir  = this->get_parameter("espeak_data_dir").as_string();
    int         num_threads = this->get_parameter("num_threads").as_int();
    std::string provider    = this->get_parameter("provider").as_string();

    RCLCPP_INFO(get_logger(), "Chargement du modèle TTS : %s", model_path.c_str());

    // Configuration sherpa-onnx VITS offline
    // max_num_sentences = 2 : sherpa-onnx découpe les textes longs en 2 phrases max
    SherpaOnnxOfflineTtsConfig config{};
    config.model.vits.model    = model_path.c_str();
    config.model.vits.tokens   = tokens_path.c_str();
    config.model.vits.data_dir = espeak_dir.c_str();  // données phonétiques espeak-ng
    config.model.num_threads   = num_threads;
    config.model.provider      = provider.c_str();    // "cuda" → ONNX Runtime CUDA EP
    config.max_num_sentences   = 2;

    tts_ = SherpaOnnxCreateOfflineTts(&config);
    if (!tts_) {
      RCLCPP_ERROR(get_logger(),
                   "Impossible de charger le modèle TTS ! "
                   "Vérifier model_path, tokens_path et espeak_data_dir.");
      rclcpp::shutdown();
      return;
    }

    // Récupérer la fréquence d'échantillonnage du modèle chargé
    // (22050 Hz pour les modèles VITS standard)
    sample_rate_ = SherpaOnnxOfflineTtsSampleRate(tts_);
    RCLCPP_INFO(get_logger(), "TTS prêt — %d Hz | provider: %s",
                sample_rate_, provider.c_str());

    // Démarrer le thread worker avant de s'abonner au topic
    // pour éviter une race condition si un message arrive immédiatement
    worker_thread_ = std::thread(WorkerThread, tts_, sample_rate_);

    // Abonnement au topic /to_speak (QoS depth=10 : buffer 10 phrases max)
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/to_speak", 10,
      std::bind(&QboTTSNode::speak_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "qbo_tts_node prêt — en écoute sur /to_speak");
  }

  ~QboTTSNode() {
    // Signaler l'arrêt au WorkerThread et le débloquer s'il attend sur la cv
    g_killed = true;
    g_text_queue.cv.notify_all();

    // Attendre la fin propre du thread avant de détruire le handle TTS
    if (worker_thread_.joinable()) worker_thread_.join();

    // Libérer le modèle TTS
    if (tts_) SherpaOnnxDestroyOfflineTts(tts_);
  }

private:
  /**
   * @brief Callback ROS2 pour le topic /to_speak.
   *
   * Empile la requête dans g_text_queue et retourne IMMÉDIATEMENT.
   * L'inférence et la lecture sont entièrement déléguées à WorkerThread.
   * Le thread ROS (rclcpp::spin) n'est jamais bloqué.
   *
   * @param msg Texte à synthétiser
   */
  void speak_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;

    // RCLCPP_INFO(get_logger(), "Speaking: %s", msg->data.c_str());

    // Lire sid et speed à chaque appel : permet de les modifier dynamiquement
    // via ros2 param set sans redémarrer le node
    int   sid   = this->get_parameter("sid").as_int();
    float speed = static_cast<float>(this->get_parameter("speed").as_double());

    // Empiler la requête dans la file — WorkerThread sera réveillé par notify_one()
    // Les phrases s'accumulent en ordre FIFO et sont toutes jouées intégralement
    {
      std::lock_guard<std::mutex> lock(g_text_queue.mutex);
      g_text_queue.queue.push({
        msg->data,
        sid,
        speed,
        std::chrono::steady_clock::now()  // timestamp pour mesure latence
      });
    }
    g_text_queue.cv.notify_one();
    // Retour immédiat — le thread ROS reprend son spin
  }

  const SherpaOnnxOfflineTts* tts_ = nullptr; // handle modèle sherpa-onnx
  int32_t sample_rate_ = 22050;               // Hz, lu depuis le modèle chargé

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::thread worker_thread_; // thread unique inférence + audio
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QboTTSNode>());
  rclcpp::shutdown();
  return 0;
}