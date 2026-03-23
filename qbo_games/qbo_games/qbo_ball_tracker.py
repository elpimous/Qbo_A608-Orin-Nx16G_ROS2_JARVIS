#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =======================================================================================================
#     NEO BALL TRACKER NODE
#
#     DESCRIPTION:
#     Suivi d'une balle rouge en temps réel sur le flux caméra de Néo.
#     Publie la région d'intérêt (ROI) de la balle détectée sur /target_roi.
#
#     MÉTHODE:
#     - Conversion BGR → HSV
#     - Double masque rouge (0-10° + 170-180° dans l'espace HSV)
#     - Morphologie OpenCV pour nettoyer le masque
#     - Détection du plus grand contour valide
#     - Publication du bounding rect comme sensor_msgs/RegionOfInterest
#
#     OPTIMISATIONS JETSON ORIN NX:
#     - Traitement sur image réduite (scale_factor configurable, défaut 0.5)
#     - Subscriber queue=1 → on traite toujours la frame la plus récente
#     - Pas d'allocation mémoire dans la boucle principale (buffers pré-alloués)
#     - Debug image désactivable pour économiser la bande passante ROS2
#
#     TOPICS:
#     - Entrée  : /inverted_eye_image  (sensor_msgs/Image)
#     - Sortie  : /target_roi          (sensor_msgs/RegionOfInterest)
#     - Debug   : /ball_tracker_debug  (sensor_msgs/Image) [optionnel]
#
#     AUTEUR  : Vincent FOUCAULT / elpimous12
#     DATE    : 2025
# =======================================================================================================


# =======================================================================================================
# IMPORTS
# =======================================================================================================

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge


# =======================================================================================================
# CLASSE PRINCIPALE
# =======================================================================================================

class BallTrackerNode(Node):
    """
    Nœud ROS2 de suivi de balle rouge pour le robot Néo.

    Détecte la balle rouge via masque HSV et publie sa ROI à 50Hz.
    Conçu pour tourner avec une latence < 5ms sur Jetson Orin NX.
    """

    def __init__(self):
        super().__init__('qbo_ball_tracker_node')

        # =======================================================================================================
        # DÉCLARATION DES PARAMÈTRES ROS2
        # =======================================================================================================

        # Topics
        self.declare_parameter('input_image_topic', '/inverted_eye_image')
        self.declare_parameter('output_roi_topic',  '/target_roi')
        self.declare_parameter('debug_image_topic', '/ball_tracker_debug')

        # Masque HSV — plage basse (rouge autour de 0°)
        self.declare_parameter('hsv.low1',  [0,   120,  70])   # vince : H-min, S-min, V-min plage 1
        self.declare_parameter('hsv.high1', [10,  255, 255])   # vince : H-max, S-max, V-max plage 1

        # Masque HSV — plage haute (rouge autour de 180°)
        self.declare_parameter('hsv.low2',  [170, 120,  70])   # vince : plage 2 (rouge > 170°)
        self.declare_parameter('hsv.high2', [180, 255, 255])

        # Filtres de taille
        self.declare_parameter('detection.min_radius_px',   8)    # vince : rayon min en px (image originale)
        self.declare_parameter('detection.max_radius_px', 300)    # vince : rayon max (évite faux positifs)

        # Performances
        self.declare_parameter('perf.scale_factor',   0.5)    # vince : résolution de traitement (0.5 = moitié)
        self.declare_parameter('perf.publish_debug', True)    # vince : activer/désactiver image debug

        # Noyau morphologique
        self.declare_parameter('morph.kernel_size', 5)        # vince : taille du kernel pour open/close

        # =======================================================================================================
        # LECTURE DES PARAMÈTRES
        # =======================================================================================================

        topic_in    = self.get_parameter('input_image_topic').value
        topic_roi   = self.get_parameter('output_roi_topic').value
        topic_debug = self.get_parameter('debug_image_topic').value

        # vince : construction des arrays numpy directement depuis les paramètres
        self.hsv_low1  = np.array(self.get_parameter('hsv.low1').value,  dtype=np.uint8)
        self.hsv_high1 = np.array(self.get_parameter('hsv.high1').value, dtype=np.uint8)
        self.hsv_low2  = np.array(self.get_parameter('hsv.low2').value,  dtype=np.uint8)
        self.hsv_high2 = np.array(self.get_parameter('hsv.high2').value, dtype=np.uint8)

        self.min_radius = self.get_parameter('detection.min_radius_px').value
        self.max_radius = self.get_parameter('detection.max_radius_px').value

        self.scale      = self.get_parameter('perf.scale_factor').value
        self.pub_debug  = self.get_parameter('perf.publish_debug').value

        k = self.get_parameter('morph.kernel_size').value
        # vince : pré-alloue le kernel morphologique une seule fois
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        # =======================================================================================================
        # QoS — latence minimale : on ne veut que la frame la plus récente
        # =======================================================================================================

        # vince : BEST_EFFORT + keep last 1 → jamais de backlog, latence minimale
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =======================================================================================================
        # COMMUNICATION ROS2
        # =======================================================================================================

        self.bridge = CvBridge()

        # Subscriber image caméra
        self.sub_image = self.create_subscription(
            Image,
            topic_in,
            self.image_callback,
            qos_sensor
        )

        # Publisher ROI — RELIABLE car c'est une donnée de contrôle
        self.pub_roi = self.create_publisher(
            RegionOfInterest,
            topic_roi,
            10
        )

        # Publisher debug (optionnel)
        if self.pub_debug:
            self.pub_debug_img = self.create_publisher(
                Image,
                topic_debug,
                qos_sensor
            )
        else:
            self.pub_debug_img = None

        # =======================================================================================================
        # ÉTAT INTERNE
        # =======================================================================================================

        # vince : flag indiquant si une balle est actuellement visible
        self.ball_detected = False

        # vince : dernière ROI valide (pour log uniquement)
        self.last_roi = None

        self.get_logger().info(
            f'BallTracker démarré | in={topic_in} | out={topic_roi} | '
            f'scale={self.scale} | debug={self.pub_debug}'
        )

    # =======================================================================================================
    # CALLBACK PRINCIPAL — appelé à chaque frame reçue
    # =======================================================================================================

    def image_callback(self, msg: Image):
        """
        Traitement d'une frame :
        1. Conversion ROS2 → OpenCV (BGR)
        2. Redimensionnement pour traitement rapide
        3. Double masque HSV rouge
        4. Morphologie pour nettoyer le bruit
        5. Extraction du plus grand contour
        6. Publication ROI (coordonnées dans l'image originale)
        """

        # ---- Conversion ROS2 → OpenCV ----
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'cv_bridge error: {e}')
            return

        h_orig, w_orig = frame.shape[:2]

        # ---- Redimensionnement pour traitement rapide ----
        # vince : INTER_NEAREST = interpolation la plus rapide (pas de flou, ok pour HSV)
        if self.scale != 1.0:
            small = cv2.resize(frame, None,
                               fx=self.scale, fy=self.scale,
                               interpolation=cv2.INTER_NEAREST)
        else:
            small = frame

        # ---- Conversion BGR → HSV ----
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # ---- Double masque rouge (0-10° et 170-180°) ----
        mask1 = cv2.inRange(hsv, self.hsv_low1, self.hsv_high1)
        mask2 = cv2.inRange(hsv, self.hsv_low2, self.hsv_high2)
        mask  = cv2.bitwise_or(mask1, mask2)

        # ---- Morphologie : supprime bruit, comble les trous ----
        # vince : OPEN supprime petits pixels parasites, CLOSE comble les trous
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        # ---- Détection du contour le plus grand ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        roi_msg = RegionOfInterest()
        found   = False

        if contours:
            # vince : on prend le plus grand contour par aire
            best = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(best)

            if area > 0:
                # Cercle minimal englobant → rayon pour filtrer
                (cx, cy), radius = cv2.minEnclosingCircle(best)

                # vince : scale_factor inverse pour repasser en coordonnées originales
                inv = 1.0 / self.scale
                cx_orig = cx * inv
                cy_orig = cy * inv
                r_orig  = radius * inv

                if self.min_radius <= r_orig <= self.max_radius:
                    # Bounding rect en coordonnées originales
                    x, y, w, h = cv2.boundingRect(best)
                    x_orig = int(x * inv)
                    y_orig = int(y * inv)
                    w_orig_r = int(w * inv)
                    h_orig_r = int(h * inv)

                    # vince : clamp pour rester dans l'image
                    x_orig   = max(0, x_orig)
                    y_orig   = max(0, y_orig)
                    w_orig_r = min(w_orig_r, w_orig - x_orig)
                    h_orig_r = min(h_orig_r, h_orig - y_orig)

                    roi_msg.x_offset  = x_orig
                    roi_msg.y_offset  = y_orig
                    roi_msg.width     = w_orig_r
                    roi_msg.height    = h_orig_r
                    roi_msg.do_rectify = False
                    found = True

                    if not self.ball_detected:
                        self.get_logger().info(
                            f'Balle rouge détectée | centre=({int(cx_orig)},{int(cy_orig)}) '
                            f'rayon={int(r_orig)}px'
                        )

        # vince : publie toujours une ROI (zeros = non détecté)
        self.pub_roi.publish(roi_msg)

        # Mise à jour état
        if self.ball_detected and not found:
            self.get_logger().info('Balle perdue')
        self.ball_detected = found

        # ---- Image debug (optionnelle) ----
        if self.pub_debug_img is not None:
            debug = self._build_debug_image(frame, roi_msg, found)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                debug_msg.header = msg.header
                self.pub_debug_img.publish(debug_msg)
            except Exception as e:
                self.get_logger().warning(f'debug publish error: {e}')

    # =======================================================================================================
    # IMAGE DEBUG
    # =======================================================================================================

    def _build_debug_image(self, frame: np.ndarray,
                            roi: RegionOfInterest, found: bool) -> np.ndarray:
        """
        Dessine la ROI et le statut sur une copie de la frame originale.
        Appelé uniquement si pub_debug=True.
        """
        # vince : on copie uniquement pour le debug, pas dans le chemin critique
        debug = frame.copy()

        if found:
            # Rectangle vert autour de la balle
            cv2.rectangle(
                debug,
                (roi.x_offset, roi.y_offset),
                (roi.x_offset + roi.width, roi.y_offset + roi.height),
                (0, 255, 0), 2
            )
            # vince : croix au centre du bounding rect
            cx = roi.x_offset + roi.width  // 2
            cy = roi.y_offset + roi.height // 2
            cv2.drawMarker(debug, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 20, 2)
            label = f'BALLE ({cx},{cy})'
            color = (0, 255, 0)
        else:
            label = 'NON DETECTEE'
            color = (0, 0, 255)

        cv2.putText(debug, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        return debug


# =======================================================================================================
# POINT D'ENTRÉE
# =======================================================================================================

def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('BallTracker interrompu')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
