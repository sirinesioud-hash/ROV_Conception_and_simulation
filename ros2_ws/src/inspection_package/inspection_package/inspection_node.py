"""
===============================================================================
Module: inspection_node.py
Author: Sirine SIOUD (ENICarthage - Mechatronics)
Project: Subsea Pipeline Inspection ROV

Description:
    Synchronizes telemetry across multiple subsea sensor streams:
    - Camera Feed (sensor_msgs/msg/Image)
    - Sonar Point Cloud (sensor_msgs/msg/PointCloud2)
    - USBL Positioning (sensor_msgs/msg/NavSatFix)
    
    Triggered via the /take_snapshot topic to export timestamped,
    georeferenced inspection reports and defect imagery.
===============================================================================
"""
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix  
from sensor_msgs_py.point_cloud2 import read_points
import math
from std_msgs.msg import Empty
import cv2
from cv_bridge import CvBridge
import datetime
import os
import json

class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_logger')
        
        self.latest_image = None
        self.latest_sonar = None
        self.latest_gps = None
        self.bridge = CvBridge()

        # Répertoire racine du projet
        self.project_dir = "dossier_projet_inspections"
        os.makedirs(self.project_dir, exist_ok=True)

        # === Mise à jour des abonnements selon ton Bridge ===
        self.create_subscription(Image, '/camera', self.img_callback, 10)
        self.create_subscription(PointCloud2, '/sonar_sensor/points', self.sonar_callback, 10)
        self.create_subscription(NavSatFix, '/bluerov2/usbl/position', self.gps_callback, 10)
        
        # Déclencheur
        self.create_subscription(Empty, '/take_snapshot', self.snapshot_callback, 10)
        
        self.get_logger().info('Noeud d\'inspection mis à jour avec succès et à l\'écoute.')

    def img_callback(self, msg):
        self.latest_image = msg

    def sonar_callback(self, msg):
        self.latest_sonar = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def snapshot_callback(self, msg):
        if not all([self.latest_image, self.latest_sonar, self.latest_gps]):
            self.get_logger().warning("Données capteurs incomplètes. Enregistrement impossible.")
            return
            
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Création de l'arborescence des dossiers
        snapshot_dir_name = f"snapshot_{timestamp}"
        snapshot_dir_path = os.path.join(self.project_dir, snapshot_dir_name)
        os.makedirs(snapshot_dir_path, exist_ok=True)
        
        img_filename = f"inspection_{timestamp}.png"
        json_filename = f"report_{timestamp}.json"
        
        full_img_path = os.path.join(snapshot_dir_path, img_filename)
        full_json_path = os.path.join(snapshot_dir_path, json_filename)
        
        # 1. Sauvegarde de la trame caméra OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        cv2.imwrite(full_img_path, cv_img)
        canvas_size = 500
        sonar_heatmap = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        center = canvas_size // 2

        # Max range of your sonar in meters (matches your SDF configuration)
        max_range = 10.0 
        scale = (canvas_size // 2) / max_range  # Pixels per meter

        extracted_points = []
        try:
            for p in read_points(self.latest_sonar, field_names=("x", "y", "z"), skip_nans=True):
                # Conversion explicite en float standard Python pour éviter les erreurs de sérialisation JSON
                extracted_points.append([float(p[0]), float(p[1]), float(p[2])])
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la lecture du PointCloud2: {str(e)}")
            return

        for x, y, z in extracted_points:
            # SAFETY CHECK: Ignore points that are infinite or NaN
            if not (math.isfinite(x) and math.isfinite(y)):
                continue

            # Calculate distance (Range) from the sensor
            distance = np.sqrt(x**2 + y**2)
            
            # Map the real-world meters to pixel coordinates relative to the center
            pixel_x = int(center + (x * scale))
            pixel_y = int(center - (y * scale)) # Invert Y for image coordinates
            
            if 0 <= pixel_x < canvas_size and 0 <= pixel_y < canvas_size:
                # Calculate a normalized intensity value (0 to 255) based on proximity
                intensity = int(255 * (1.0 - (distance / max_range)))
                intensity = max(0, min(255, intensity)) # Clamp values
                
                # Draw the point on our grayscale canvas
                sonar_heatmap[pixel_y, pixel_x] = [intensity, intensity, intensity]

        # 2. Apply OpenCV's built-in Heatmap color effect (COLORMAP_JET is the classic blue-to-red)
        sonar_heatmap = cv2.applyColorMap(sonar_heatmap, cv2.COLORMAP_JET)
        heatmap_filename = f"sonar_heatmap_{timestamp}.png"
        full_heatmap_path = os.path.join(snapshot_dir_path, heatmap_filename)
        cv2.imwrite(full_heatmap_path, sonar_heatmap)
        
        # 2. Structuration des données et conversion du LaserScan en liste JSON
        report_data = {
            "mission_metadata": {
                "timestamp": timestamp,
                "status": "Anomalie ou fissure détectée"
            },
            "georeferencing": {
                "latitude": self.latest_gps.latitude,
                "longitude": self.latest_gps.longitude,
                "depth_altitude": self.latest_gps.altitude
            },
            "associated_files": {
                "image": img_filename
            }
        }
        
        # 3. Écriture du fichier JSON final
        with open(full_json_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, ensure_ascii=False, indent=4)
            
        self.get_logger().info(f"Rapport géoréférencé enregistré : {snapshot_dir_path}")

def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()