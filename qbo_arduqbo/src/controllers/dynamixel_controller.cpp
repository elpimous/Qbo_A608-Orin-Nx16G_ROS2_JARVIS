            // =============================================================================
            // dynamixel_controller.cpp — Contrôleur Dynamixel AX-12A pour Néo (QBo)
            // =============================================================================
            //
            // ARCHITECTURE — Protocol 1.0 (AX-12A) UNIQUEMENT
            // ─────────────────────────────────────────────────
            //
            // 1. SYNC WRITE (écriture simultanée multi-servo)
            //    ┌─────────────────────────────────────────────────────────┐
            //    │  Au lieu de :                                           │
            //    │    WRITE(id=1, Goal_Position=512)  → 1 trame série     │
            //    │    WRITE(id=1, Moving_Speed=300)   → 1 trame série     │
            //    │    WRITE(id=2, Goal_Position=600)  → 1 trame série     │
            //    │    WRITE(id=2, Moving_Speed=300)   → 1 trame série     │
            //    │  On envoie :                                            │
            //    │    SYNC_WRITE([id1, id2], [pos1, spd1, pos2, spd2])   │
            //    │                                          → 1 trame !   │
            //    └─────────────────────────────────────────────────────────┘
            //    Protocol 1.0 AX-12A — adresses contiguës :
            //      Addr 30 (0x1E) : Goal_Position [2 bytes]
            //      Addr 32 (0x20) : Moving_Speed  [2 bytes]
            //      → addSyncWriteHandler(30, 4) couvre les deux en 1 handler
            //
            //    SYNC_WRITE : ✅ supporté Protocol 1.0
            //    SYNC_READ  : ❌ Protocol 2.0 uniquement → lectures individuelles itemRead()
            //
            // 2. PRÉ-ALLOCATION DU MESSAGE JointState
            //    Alloué une fois à l'init (resize), réutilisé à chaque tick 50Hz.
            //    Zéro allocation heap dans le hot path.
            //
            // 3. LOOKUP O(1) DES SERVOS
            //    unordered_map<string, DynamixelServo*> dans jointCmdCallback.
            //
            // 4. ÉCRITURE TORQUE_LIMIT CONDITIONNELLE
            //    writeRegister(Torque_Limit) uniquement si la valeur change.
            //
            // 5. COMMANDE GROUPÉE dans flushSyncWrite()
            //    Toutes les commandes du cycle → 1 seul SYNC_WRITE sur le bus.
            //
            // =============================================================================

            #include "qbo_arduqbo/controllers/dynamixel_controller.hpp"
            #include <geometry_msgs/msg/transform_stamped.hpp>
            #include <tf2/LinearMath/Quaternion.h>
            #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
            #include "rclcpp/node_interfaces/node_parameters_interface.hpp"

            using std::placeholders::_1;
            using std::placeholders::_2;
            using namespace std::chrono_literals;

            // =============================================================================
            // 🔧 DynamixelServo — Représente un servo individuel AX-12A
            // =============================================================================

            DynamixelServo::DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                                        const std::string & name,
                                        DynamixelWorkbench* wb)
                : id_(1),
                name_(name),
                joint_name_(name),
                model_number_(0),

                invert_(false),
                neutral_(512),
                ticks_(1024),
                max_angle_(radians(20.0)),
                min_angle_(radians(-20.0)),
                rad_per_tick_(radians(40.0) / 1024),
                max_speed_(radians(180.0)),
                range_(radians(40.0)),
                torque_limit_(1023),

                angle_(0.0f),
                last_written_torque_limit_(-1),

                node_(node),
                dxl_wb_(wb),
                torque_enabled_(false),
                goal_ticks_(0)
            {
            }

            DynamixelServo::~DynamixelServo() = default;

            // ─── Service : active / désactive le torque du servo ─────────────────────────
            bool DynamixelServo::servoTorqueEnable(
                const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
                std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res)
            {
                if (req->torque_enable) {
                    res->success = dxl_wb_->torqueOn(id_);
                    torque_enabled_ = res->success;
                } else {
                    res->success = dxl_wb_->itemWrite(id_, "Torque_Enable", 0);
                    // vince : itemWrite réussit (true) → torque bien désactivé → false
                    torque_enabled_ = res->success ? false : true;
                }
                return true;
            }

            // ─── Chargement des paramètres ROS2 pour ce servo ────────────────────────────
            void DynamixelServo::setParams(const std::string & joint_key)
            {
                const std::string base = "dynamixel.motors." + joint_key + ".";

                node_->get_parameter(base + "id",      id_);
                node_->get_parameter(base + "invert",  invert_);
                node_->get_parameter(base + "neutral", neutral_);
                node_->get_parameter(base + "ticks",   ticks_);

                double max_deg = 30.0;
                node_->get_parameter(base + "max_angle_degrees", max_deg);
                max_angle_ = radians(max_deg);

                double min_deg = -30.0;
                node_->get_parameter(base + "min_angle_degrees", min_deg);
                min_angle_ = radians(min_deg);

                double range_deg = 60.0;
                node_->get_parameter(base + "range", range_deg);
                range_ = radians(range_deg);

                double speed_val = radians(180.0);
                node_->get_parameter(base + "max_speed", speed_val);
                max_speed_ = speed_val;

                int torque_limit = 1023;
                node_->get_parameter(base + "torque_limit", torque_limit);
                torque_limit_ = torque_limit;

                // vince : joint_name_ = valeur du param "name" (ex: "head_pan_joint")
                // et non la clé YAML (ex: "motor_1") — corrige le lookup dans servo_map_
                std::string jname = name_;
                node_->get_parameter(base + "name", jname);
                joint_name_ = jname;

                // vince : rad_per_tick_ calculé UNE SEULE FOIS après lecture des params.
                // Recalculé uniquement si range_ ou ticks_ change via onParameterChange().
                rad_per_tick_ = range_ / static_cast<float>(ticks_);

                servo_torque_enable_srv_ = node_->create_service<qbo_msgs::srv::TorqueEnable>(
                    joint_name_ + "/torque_enable",
                    std::bind(&DynamixelServo::servoTorqueEnable, this, _1, _2)
                );

                param_callback_handle_ = node_->add_on_set_parameters_callback(
                    std::bind(&DynamixelServo::onParameterChange, this, _1)
                );
                
                RCLCPP_INFO(node_->get_logger(),
                    "[%s] Params: id=%d invert=%s neutral=%d min=%.3frad max=%.3frad "
                    "ticks=%d range=%.3frad rad_per_tick=%.6f torque_limit=%d speed=%.3frad/s",
                    joint_key.c_str(), id_, invert_ ? "true" : "false", neutral_,
                    min_angle_, max_angle_, ticks_, range_, rad_per_tick_, torque_limit_, max_speed_);
            }

            // ─── Callback paramètre dynamique (ros2 param set ...) ───────────────────────
            rcl_interfaces::msg::SetParametersResult DynamixelServo::onParameterChange(
                const std::vector<rclcpp::Parameter> & parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto & param : parameters)
                {
                    const std::string base = "dynamixel.motors." + name_ + ".";
                    const auto & key = param.get_name();

                    // vince : rad_per_tick_ recalculé UNIQUEMENT si range_ ou ticks_ change
                    if (key == base + "max_angle_degrees") {
                        max_angle_ = radians(param.as_double());
                    }
                    else if (key == base + "min_angle_degrees") {
                        min_angle_ = radians(param.as_double());
                    }
                    else if (key == base + "neutral") {
                        neutral_ = param.as_int();
                    }
                    else if (key == base + "range") {
                        range_ = radians(param.as_double());
                        rad_per_tick_ = range_ / static_cast<float>(ticks_);  // vince : recalcul nécessaire
                    }
                    else if (key == base + "ticks") {
                        ticks_ = param.as_int();
                        rad_per_tick_ = range_ / static_cast<float>(ticks_);  // vince : recalcul nécessaire
                    }
                    else if (key == base + "torque_limit") {
                        torque_limit_ = param.as_int();
                        // vince : écriture directe + mise à jour du cache
                        dxl_wb_->writeRegister(id_, "Torque_Limit", torque_limit_);
                        last_written_torque_limit_ = torque_limit_;
                    }

                    RCLCPP_INFO(node_->get_logger(), "[%s] Param updated: %s", name_.c_str(), key.c_str());
                }

                return result;
            }

            // ─── Commande d'angle ─────────────────────────────────────────────────────────
            //
            // vince : NE WRITE PAS sur le bus. Remplit pending_goal_ / pending_speed_
            // et lève pending_dirty_. Le write réel est fait par flushSyncWrite()
            // de manière groupée pour tous les servos en 1 seule trame SYNC_WRITE.
            //
            void DynamixelServo::setAngle(float ang, float velocity)
            {
                // ─── Clamp + inversion mécanique ─────────────────────────────────────────
                ang = std::clamp(ang, min_angle_, max_angle_);
                if (invert_) ang = -ang;

                // ─── Conversion angle → ticks (Goal_Position AX-12A addr 30) ────────────
                // AX-12A : 0–1023 ticks sur 300°, neutral_ = centre (typiquement 512)
                pending_goal_ = static_cast<int32_t>(std::round(ang / rad_per_tick_)) + neutral_;
                goal_ticks_   = pending_goal_;  // vince : sync pour getGoalTicks()

                // ─── Conversion vitesse → Moving_Speed AX-12A (addr 32, 0–1023) ─────────
                // MAX_AX_SPEED_RAD ≈ 12.0 rad/s = vitesse max physique AX-12A
                // On force min 1 pour garder le contrôle (0 = vitesse max non contrôlée)
                constexpr float MAX_AX_SPEED_RAD = 12.0f;
                const float speed_ratio = std::clamp(velocity / MAX_AX_SPEED_RAD, 0.0f, 1.0f);
                pending_speed_ = std::max(1, static_cast<int>(std::round(speed_ratio * 1023)));

                // ─── Torque_Limit : écriture CONDITIONNELLE ───────────────────────────────
                // vince : écrit sur le bus UNIQUEMENT si la valeur a changé.
                // last_written_torque_limit_ = -1 au démarrage → force le 1er write.
                // Avant : writeRegister() à chaque setAngle() = 50Hz de trafic inutile.
                if (torque_limit_ != last_written_torque_limit_) {
                    dxl_wb_->writeRegister(id_, "Torque_Limit", torque_limit_);
                    last_written_torque_limit_ = torque_limit_;
                    RCLCPP_DEBUG(node_->get_logger(),
                        "[%s] Torque_Limit écrit : %d", name_.c_str(), torque_limit_);
                }

                angle_         = ang;     // vince : valeur souhaitée, corrigée par itemRead() ensuite
                pending_dirty_ = true;    // vince : signale à flushSyncWrite() qu'il y a une commande
            }

            // =============================================================================
            // 🤖 DynamixelController — Gestion globale du bus Dynamixel
            // =============================================================================
            //
            // FLUX PRINCIPAL (50Hz) :
            //
            //   [Topic /cmd_joints]
            //        │
            //        ▼
            //   jointCmdCallback()
            //     → lookup O(1) dans servo_map_
            //     → servo->setAngle() → pending_goal_ / pending_speed_ / pending_dirty_
            //        │
            //        ▼                   (appelé par publishJointStates() chaque tick)
            //   flushSyncWrite()
            //     → collecte servos dirty → sync_ids_ / sync_data_
            //     → 1 SYNC_WRITE RS-485 pour tous les servos
            //        │
            //        ▼
            //   itemRead() individuel par servo (Protocol 1.0 — pas de Sync Read)
            //     → publishJointStates() → /joint_states + TF
            //
            // =============================================================================

            DynamixelController::DynamixelController(const std::shared_ptr<rclcpp::Node> & node)
                : node_(node)
            {
                // ─── Lecture paramètres ───────────────────────────────────────────────────
                usb_port_         = node_->get_parameter("dynamixel.usb_port").as_string();
                baud_rate_        = node_->get_parameter("dynamixel.baud_rate").as_int();
                protocol_version_ = node_->get_parameter("dynamixel.protocol_version").as_double();
                auto_torque_off_  = node_->get_parameter("auto_torque_off").as_bool();
                timeout_sec_      = node_->get_parameter("auto_torque_off_timeout").as_double();

                // ─── Init bus Dynamixel ───────────────────────────────────────────────────
                if (!dxl_wb_.init(usb_port_.c_str(), baud_rate_)) {
                    throw std::runtime_error("DynamixelWorkbench init failed sur " + usb_port_);
                }
                dxl_wb_.setPacketHandler(protocol_version_);

                RCLCPP_INFO(node_->get_logger(),
                    "🔌 Bus Dynamixel initialisé : port=%s baud=%d proto=%.1f",
                    usb_port_.c_str(), baud_rate_, protocol_version_);

                // ─── Publishers / Subscribers ─────────────────────────────────────────────
                joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
                joint_cmd_sub_   = node_->create_subscription<sensor_msgs::msg::JointState>(
                    "/cmd_joints", 10,
                    std::bind(&DynamixelController::jointCmdCallback, this, _1));

                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

                // ─── Diagnostics
                diagnostics_ = std::make_shared<diagnostic_updater::Updater>(node_);
                diagnostics_->setHardwareID("Dynamixel Bus");

                // ─── Init servos ──────────────────────────────────────────────────────────
                std::vector<std::string> keys;
                node_->get_parameter("dynamixel.motor_keys", keys);

                // Lecture des seuils de diagnostic (comme dans l'ancienne version)
                double temp_warn = 60.0, volt_min = 8.0, volt_max = 12.5;
                int pos_error = 20;
                node_->get_parameter_or("diagnostic_temp_warn", temp_warn, 60.0);
                node_->get_parameter_or("diagnostic_voltage_min", volt_min, 8.0);
                node_->get_parameter_or("diagnostic_voltage_max", volt_max, 12.5);
                node_->get_parameter_or("diagnostic_position_error", pos_error, 20);

                for (const auto & key : keys) {
                    auto servo = std::make_unique<DynamixelServo>(node_, key, &dxl_wb_);
                    servo->setParams(key);

                    if (dxl_wb_.ping(servo->id_, &servo->model_number_)) {
                        RCLCPP_INFO(node_->get_logger(),
                            "✔️  Servo [%s] détecté — ID=%d model=%d",
                            key.c_str(), servo->id_, servo->model_number_);
                    } else {
                        RCLCPP_ERROR(node_->get_logger(),
                            "❌ Ping échoué pour [%s] (ID=%d)", key.c_str(), servo->id_);
                    }

                    // vince : servo_map_ = lookup O(1), pointeurs non-owning vers servos_
                    servo_map_[servo->joint_name_] = servo.get();  // vince : indexé par joint_name_ après setParams()

                    // ====================== DIAGNOSTICS PAR SERVO ======================
                    // (ajouté ici sans rien supprimer de l'original)
                    diagnostics_->add(servo->joint_name_,
                    [this, servo_raw = servo.get(), temp_warn, volt_min, volt_max, pos_error]
                    (diagnostic_updater::DiagnosticStatusWrapper & stat) 
                    {

                        int32_t goal = 0, position = 0, load = 0, temp = 0, volt_raw = 0, moving = 0;

                        dxl_wb_.itemRead(servo_raw->id_, "Goal_Position", &goal);
                        dxl_wb_.itemRead(servo_raw->id_, "Present_Position", &position);
                        dxl_wb_.itemRead(servo_raw->id_, "Present_Load", &load);
                        dxl_wb_.itemRead(servo_raw->id_, "Present_Temperature", &temp);
                        dxl_wb_.itemRead(servo_raw->id_, "Present_Voltage", &volt_raw);
                        dxl_wb_.itemRead(servo_raw->id_, "Moving", &moving);

                        float voltage = static_cast<float>(volt_raw) / 10.0f;
                        int error = goal - position;

                        stat.add("ID", servo_raw->id_);
                        stat.add("Température (°C)", temp);
                        stat.add("Tension (V)", voltage);
                        stat.add("Erreur position (ticks)", error);
                        stat.add("Moving", moving ? "Yes" : "No");
                        stat.add("Torque activé", servo_raw->isTorqueEnabled() ? "Yes" : "No");
                        stat.add("Torque Limit", servo_raw->torque_limit_);

                        // Estimation puissance
                        float load_ratio = static_cast<float>(load & 0x3FF) / 1023.0f;
                        float current_est = load_ratio * 1.5f;
                        float power = voltage * current_est;
                        stat.addf("Puissance estimée", "%.2f W", power);

                        // Niveau de diagnostic
                        if (temp > temp_warn)
                            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Température élevée");
                        else if (voltage < volt_min || voltage > volt_max)
                            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Tension hors plage");
                        else if (std::abs(error) > pos_error)
                            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Erreur de position importante");
                        else
                            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
                    });
                    // ===================================================================

                    servo->setAngle(0.0f, 0.8f);
                    servos_.push_back(std::move(servo));
                }

                // ─── PRÉ-ALLOCATION du message JointState ────────────────────────────────
                // vince : resize() une fois → zéro allocation dans le hot path 50Hz.
                // Noms pré-remplis (constants, jamais modifiés ensuite).
                {
                    const size_t n = servos_.size();
                    joint_state_msg_.name.resize(n);
                    joint_state_msg_.position.resize(n, 0.0);
                    joint_state_msg_.velocity.resize(n, 0.0);  // AX-12A : pas d'encodeur vitesse
                    joint_state_msg_.effort.resize(n, 0.0);    // AX-12A : pas de capteur couple
                    for (size_t i = 0; i < n; ++i)
                        joint_state_msg_.name[i] = servos_[i]->joint_name_;
                    RCLCPP_INFO(node_->get_logger(), "📦 JointState pré-alloué pour %zu servos", n);
                }

                // ─── Init SYNC WRITE Handler ──────────────────────────────────────────────
                //
                // vince : Protocol 1.0 — trame SYNC_WRITE (instruction 0x83) :
                //   [0xFF][0xFF][0xFE][len][0x83][start_addr][data_len]
                //   [id1][data1a][data1b]...[id2][data2a][data2b]...[checksum]
                //
                // Adresses AX-12A contiguës → 1 seul handler sur 4 bytes depuis addr 30 :
                //   Addr 30 : Goal_Position L+H (2 bytes)
                //   Addr 32 : Moving_Speed  L+H (2 bytes)
                //
                // Gain : 2 servos × 2 registres = 4 trames/cycle → 1 trame/cycle (4× moins)
                //
                const char * log = nullptr;
                sync_write_handler_idx_ = 0;

                if (!dxl_wb_.addSyncWriteHandler(30, 4, &log)) {
                    // vince : échec → fallback automatique sur itemWrite() dans flushSyncWrite()
                    RCLCPP_WARN(node_->get_logger(),
                        "⚠️  addSyncWriteHandler échoué : %s — fallback writes individuels",
                        log ? log : "?");
                    sync_write_available_ = false;
                } else {
                    sync_write_available_ = true;
                    RCLCPP_INFO(node_->get_logger(),
                        "✅ Sync Write Handler initialisé (addr=30, 4 bytes, idx=%d)",
                        sync_write_handler_idx_);
                }

                // ─── Pré-allocation des buffers syncWrite ─────────────────────────────────
                // vince : resize() une fois → zéro alloc dans flushSyncWrite()
                //   sync_ids_  : [id0, id1, ...]
                //   sync_data_ : [goal0, speed0, goal1, speed1, ...]  (2 int32 par servo)
                {
                    const size_t n = servos_.size();
                    sync_ids_.resize(n);
                    sync_data_.resize(n * 2);
                }

                // ─── TF configs ───────────────────────────────────────────────────────────
                // vince : lookup table → remplace if/else dans publishJointStates()
                // Ajouter un joint = 1 ligne ici, zéro modification du hot path
                tf_configs_["head_pan_joint"]  = TfConfig{"base_link",     "base_pan_link",  0.045, 0.0, 0.35, TfAxis::YAW};
                tf_configs_["head_tilt_joint"] = TfConfig{"base_pan_link", "head_tilt_link", 0.0,   0.0, 0.0,  TfAxis::PITCH};

                // ─── Timers ───────────────────────────────────────────────────────────────
                int joint_timer_rate = 50;
                node_->get_parameter("dynamixel_joint_rate_hz", joint_timer_rate);
                joint_state_timer_ = node_->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / joint_timer_rate)),
                    std::bind(&DynamixelController::publishJointStates, this));
                RCLCPP_INFO(node_->get_logger(), "⏱️  Joint state timer : %d Hz", joint_timer_rate);

                last_cmd_time_ = steady_clock_.now();
                inactivity_timer_ = node_->create_wall_timer(
                    200ms, std::bind(&DynamixelController::checkInactivity, this));

                // Timer diagnostics (1 Hz) — fait apparaître les servos dans /diagnostics
                diagnostics_timer_ = node_->create_wall_timer(
                    1s, std::bind(&DynamixelController::publishDiagnostics, this));
                
            }

            // ─── Callback /cmd_joints ─────────────────────────────────────────────────────
            //
            // vince : lookup O(1) → pas de write bus ici.
            // Remplit uniquement pending_* dans chaque servo.
            // Le write réel est groupé dans flushSyncWrite() au tick suivant du timer.
            //
            void DynamixelController::jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                last_cmd_time_ = steady_clock_.now();

                const bool velocity_included = (msg->velocity.size() == msg->position.size());

                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    auto it = servo_map_.find(msg->name[i]);
                    if (it == servo_map_.end()) {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Joint inconnu dans /cmd_joints : '%s'", msg->name[i].c_str());
                        continue;
                    }

                    DynamixelServo * servo = it->second;
                    const float pos = static_cast<float>(msg->position[i]);
                    const float vel = velocity_included ? static_cast<float>(msg->velocity[i]) : 1.0f;

                    // vince : activation torque directe — sans shared_ptr Request/Response
                    // (évite 2 allocations heap par commande quand auto_torque_off_ actif)
                    if (!servo->isTorqueEnabled() && auto_torque_off_) {
                        if (dxl_wb_.torqueOn(servo->id_)) {
                            servo->setTorqueEnabled(true);
                            RCLCPP_DEBUG(node_->get_logger(),
                                "⚡ Torque activé pour [%s]", msg->name[i].c_str());
                        }
                    }

                    servo->setAngle(pos, vel);
                }

                // vince : décommenter pour flush immédiat (latence minimale hors timer) :
                flushSyncWrite();
            }

            // ─── FLUSH SYNC WRITE ─────────────────────────────────────────────────────────
            //
            // vince : collecte les servos dirty → 1 seule trame SYNC_WRITE RS-485.
            //
            // Sans sync : 2 servos × 2 registres × 50Hz = 200 trames/s
            // Avec sync : 1 trame × 50Hz = 50 trames/s → 4× moins de trafic bus
            //
            // Fallback sur itemWrite() individuels si :
            //   - sync_write_available_ = false (addSyncWriteHandler a échoué)
            //   - count == 1 (sync write n'apporte rien pour 1 seul servo)
            //   - syncWrite() échoue à l'exécution
            //
            void DynamixelController::flushSyncWrite()
            {
                size_t count = 0;

                for (const auto & servo : servos_)
                {
                    if (!servo->pending_dirty_) continue;

                    sync_ids_[count]          = servo->id_;
                    // vince : sync_data_ organisé [goal0, speed0, goal1, speed1, ...]
                    // correspond aux 4 bytes contigus addr 30–33 de l'AX-12A
                    sync_data_[count * 2]     = servo->pending_goal_;
                    sync_data_[count * 2 + 1] = servo->pending_speed_;
                    servo->pending_dirty_     = false;
                    ++count;
                }

                if (count == 0) return;

                const char * log = nullptr;

                if (sync_write_available_ && count > 1)
                {
                    // ─── CAS OPTIMAL : 1 trame SYNC_WRITE pour N servos ──────────────────
                    //
                    // vince : signature dxl_wb_.syncWrite() :
                    //   bool syncWrite(uint8_t  handler_idx,
                    //                  uint8_t* ids,      uint8_t  id_cnt,
                    //                  int32_t* data,     uint16_t data_cnt,
                    //                  const char** log)
                    //   data_cnt = id_cnt × valeurs_par_servo (ici × 2)
                    //
                    if (!dxl_wb_.syncWrite(
                            sync_write_handler_idx_,
                            sync_ids_.data(),
                            static_cast<uint8_t>(count),
                            sync_data_.data(),
                            2,
                            &log))
                    {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "⚠️  syncWrite échoué (%zu servos) : %s — fallback individuel",
                            count, log ? log : "?");
                        goto fallback_individual_write;
                    }
                    return;
                }

                // ─── CAS FALLBACK : itemWrite() individuel ────────────────────────────────
                // vince : 1 seul servo dirty, sync non dispo, ou syncWrite a échoué.
                // 2 trames séquentielles par servo — impossible de faire mieux en Protocol 1.0.
                fallback_individual_write:
                for (size_t i = 0; i < count; ++i)
                {
                    dxl_wb_.itemWrite(sync_ids_[i], "Goal_Position", sync_data_[i * 2]);
                    dxl_wb_.itemWrite(sync_ids_[i], "Moving_Speed",  sync_data_[i * 2 + 1]);
                }
            }

            // ─── Publication /joint_states + TF (hot path, 50Hz) ─────────────────────────
            //
            // vince : zéro allocation heap :
            //   - joint_state_msg_ pré-alloué (resize à l'init)
            //   - tf_configs_ : lecture seule
            //   - tf_msg : stack-allocated
            //   - lectures : itemRead() individuel par servo (Protocol 1.0, pas de Sync Read)
            //     Flux par servo (addr 36 = Present_Position, 2 bytes) :
            //       Master → [0xFF][0xFF][id][4][0x02][0x24][0x02][checksum]
            //       Servo  → [0xFF][0xFF][id][4][0x00][pos_L][pos_H][checksum]
            //     Temps : ~0.5ms/servo à 500kbaud → 2 servos ≈ 1ms → négligeable à 50Hz
            //
            void DynamixelController::publishJointStates()
            {
                const auto now = node_->now();

                // vince : Write AVANT Read → on lit la position après que le servo
                // a reçu sa commande (ordre logique : Write → Read → Publish)
                flushSyncWrite();

                joint_state_msg_.header.stamp = now;

                for (size_t i = 0; i < servos_.size(); ++i)
                {
                    const auto & servo = servos_[i];
                    int32_t ticks = 0;

                    if (dxl_wb_.itemRead(servo->id_, "Present_Position", &ticks))
                    {
                        float angle = static_cast<float>(ticks - servo->neutral_) * servo->rad_per_tick_;
                        if (servo->invert_) angle = -angle;
                        joint_state_msg_.position[i] = angle;
                        servo->angle_ = angle;
                    } else {
                        // vince : fallback sur dernière valeur connue
                        // DEBUG uniquement pour ne pas saturer les logs à 50Hz
                        RCLCPP_DEBUG(node_->get_logger(),
                            "❓ itemRead échoué [%s] ID=%d — valeur cached",
                            servo->name_.c_str(), servo->id_);
                        joint_state_msg_.position[i] = servo->angle_;
                    }
                }

                joint_state_pub_->publish(joint_state_msg_);

                // ─── Publication TF ───────────────────────────────────────────────────────
                // vince : tf_configs_ lookup O(1) — pas de if/else, extensible sans modifier ce code
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = now;

                for (size_t i = 0; i < servos_.size(); ++i)
                {
                    const auto & servo = servos_[i];
                    auto cfg_it = tf_configs_.find(servo->joint_name_);
                    if (cfg_it == tf_configs_.end()) continue;

                    const TfConfig & cfg = cfg_it->second;
                    tf_msg.header.frame_id = cfg.parent_frame;
                    tf_msg.child_frame_id  = cfg.child_frame;
                    tf_msg.transform.translation.x = cfg.tx;
                    tf_msg.transform.translation.y = cfg.ty;
                    tf_msg.transform.translation.z = cfg.tz;

                    tf2::Quaternion q;
                    const float angle = static_cast<float>(joint_state_msg_.position[i]);
                    switch (cfg.axis) {
                        case TfAxis::YAW:   q.setRPY(0.0, 0.0, angle); break;  // pan  = Z
                        case TfAxis::PITCH: q.setRPY(0.0, angle, 0.0); break;  // tilt = Y
                        case TfAxis::ROLL:  q.setRPY(angle, 0.0, 0.0); break;
                    }

                    tf_msg.transform.rotation = tf2::toMsg(q);
                    tf_broadcaster_->sendTransform(tf_msg);
                }
            }

            // ─── Timer diagnostics (nouveau) ─────────────────────────────────────────────
            void DynamixelController::publishDiagnostics()
            {
                diagnostics_->force_update();
            }

            // ─── Timer inactivité : désactive le torque après timeout ────────────────────
            // vince : 5Hz (200ms) — appel direct itemWrite(), sans shared_ptr alloués
            void DynamixelController::checkInactivity()
            {
                if (!auto_torque_off_) return;
                if ((steady_clock_.now() - last_cmd_time_).seconds() <= timeout_sec_) return;

                for (auto & servo : servos_)
                {
                    if (!servo->isTorqueEnabled()) continue;
                    if (dxl_wb_.itemWrite(servo->id_, "Torque_Enable", 0)) {
                        servo->setTorqueEnabled(false);
                        RCLCPP_INFO(node_->get_logger(),
                            "💤 Torque désactivé (inactivité) pour [%s]", servo->name_.c_str());
                    }
                }
            }

            // ─── Callback paramètre dynamique (niveau contrôleur) ────────────────────────
            rcl_interfaces::msg::SetParametersResult DynamixelController::onParameterChange(
                const std::vector<rclcpp::Parameter> & parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto & param : parameters) {
                    if (param.get_name() == "auto_torque_off")
                        auto_torque_off_ = param.as_bool();
                    else if (param.get_name() == "auto_torque_off_timeout")
                        timeout_sec_ = param.as_double();
                }

                return result;
            }

            DynamixelController::~DynamixelController() = default;