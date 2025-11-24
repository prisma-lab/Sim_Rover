# Basic ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import tf2_ros
import tf2_geometry_msgs

# Executor and callback imports
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 interfaces
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String

# Image msg parser
import cv2
import os
from cv_bridge import CvBridge

# Vision model
from ultralytics import YOLO
# from utils.compiled_creator import CompiledModel

# Others
import numpy as np
import open3d as o3d
import time, json, torch
from ament_index_python.packages import get_package_share_directory


class Yolov11Node(Node):
    
    def __init__(self):
        super().__init__("yolov11_node")
        rclpy.logging.set_logger_level('yolov11_node', rclpy.logging.LoggingSeverity.INFO)
        
        ## Declare parameters for node
        # Modifica il parametro per usare solo il nome del file
        self.declare_parameter("model", "yolo11n-seg.pt")
        model_filename = self.get_parameter("model").get_parameter_value().string_value
        
        self.declare_parameter("device", "cuda:0")
        self.device = self.get_parameter("device").get_parameter_value().string_value
        
        self.declare_parameter("depth_threshold", 8.0)
        self.depth_threshold = self.get_parameter("depth_threshold").get_parameter_value().double_value
        
        self.declare_parameter("threshold", 0.6)
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        
        self.declare_parameter("enable_yolo", True)
        self.enable_yolo = self.get_parameter("enable_yolo").get_parameter_value().bool_value

        # Cerca il modello nella cartella models del package
        package_share_directory = get_package_share_directory('yolov11_ros2')  # Sostituisci con il nome del tuo package
        models_dir = os.path.join(package_share_directory, 'models')
        model_path = os.path.join(models_dir, model_filename)
        
        # Verifica se il file esiste
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found: {model_path}")
            self.get_logger().error(f"Models directory contents: {os.listdir(models_dir) if os.path.exists(models_dir) else 'Directory not found'}")
            # Fallback: usa il nome del modello come prima (scaricherà da internet)
            model_path = model_filename
        else:
            self.get_logger().info(f"Using local model: {model_path}")
        
        #Publisher della posa
        self.pose_array_pub = self.create_publisher(PoseArray, 'detected_objects_poses', 10)       
        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        # TF2 listener per trasformare da camera_link a base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.tf_world_to_camera = np.array([[-0.000, -1.000,  0.000, -0.017], [0.559,  0.000,  0.829, -0.272], [-0.829,  0.000,  0.559,  0.725], [0.000,  0.000,  0.000,  1.000]])
        self.tf_camera_to_optical = np.array([[-0.003,  0.001,  1.000,  0.000], [-1.000, -0.002, -0.003,  0.015], [0.002, -1.000,  0.001, -0.000], [0.000,  0.000,  0.000,  1.000]])
        self.tf_world_to_optical = np.matmul(self.tf_world_to_camera, self.tf_camera_to_optical)

        
        ## other inits
        self.group_1 = MutuallyExclusiveCallbackGroup() # camera subscribers
        self.group_2 = MutuallyExclusiveCallbackGroup() # vision timer
        
        self.cv_bridge = CvBridge()
        # Carica il modello dal percorso locale
        self.yolo = YOLO(model_path)  # Usa model_path invece di model_filename
        self.yolo.fuse()

        self.color_image_msg = None
        self.depth_image_msg = None
        self.camera_intrinsics = None
        self.pred_image_msg = Image()
        self.pred_compressed_msg = CompressedImage()  # Aggiungi questa linea
        
        # Set clipping distance for background removal
        depth_scale = 0.001
        self.depth_threshold = self.depth_threshold/depth_scale
        
        
        # Publishers
        self._item_dict_pub = self.create_publisher(String, "/yolo/prediction/item_dict", 10)
        self._pred_pub = self.create_publisher(CompressedImage, "/yolo/prediction/image/compressed", 10)
        
        # Subscribers
        self._color_image_sub = self.create_subscription(CompressedImage, "/rover/camera/color/image_raw/compressed", self.color_image_callback, qos_profile_sensor_data, callback_group=self.group_1)
        self._depth_image_sub = self.create_subscription(Image, "/rover/camera/aligned_depth_to_color/image_raw", self.depth_image_callback, qos_profile_sensor_data, callback_group=self.group_1)
        self._camera_info_subscriber = self.create_subscription(CameraInfo, '/rover/camera/color/camera_info', self.camera_info_callback, QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group_1)
        self.bridge = CvBridge()

        # Timers
        self._vision_timer = self.create_timer(0.04, self.object_segmentation, callback_group=self.group_2) # 25 hz

    
    def color_image_callback(self, msg: CompressedImage):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.color_image_msg = image_msg
        
    def depth_image_callback(self, msg):
        self.depth_image_msg = msg
    
    def camera_info_callback(self, msg):
        try:
            if self.camera_intrinsics is None:
                # Set intrinsics in o3d object
                self.camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
                self.camera_intrinsics.set_intrinsics(msg.width,    #msg.width
                                                  msg.height,       #msg.height
                                                  msg.k[0],         #msg.K[0] -> fx
                                                  msg.k[4],         #msg.K[4] -> fy
                                                  msg.k[2],         #msg.K[2] -> cx
                                                  msg.k[5] )        #msg.K[5] -> cy
                self.get_logger().info('Camera intrinsics have been set!')
                self.fx = msg.k[0]
                self.fy = msg.k[4]
                self.cx = msg.k[2]
                self.cy = msg.k[5]
            
        except Exception as e:
            self.get_logger().error(f'camera_info_callback Error: {e}')


    def bg_removal(self, color_img_msg: Image, depth_img_msg: Image):
        if self.color_image_msg is not None and self.depth_image_msg is not None:
        
            # Convert color image msg
            cv_color_image = self.cv_bridge.imgmsg_to_cv2(color_img_msg, desired_encoding='bgr8')
            np_color_image = np.array(cv_color_image, dtype=np.uint8)

            # Convert depth image msg
            cv_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding='passthrough')
            np_depth_image = np.array(cv_depth_image, dtype=np.uint16)

            # bg removal
            grey_color = 153
            depth_image_3d = np.dstack((np_depth_image, np_depth_image, np_depth_image)) # depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.depth_threshold) | (depth_image_3d != depth_image_3d), grey_color, np_color_image)
            
            return bg_removed, np_color_image, np_depth_image
        self.get_logger().error("Background removal error, color or depth msg was None")
    
    
    def filter_depth_object_img(self, img, starting_mask, deviation):
        """
        parameters 
        img: np depth image 
        deviation: the deviation allowed
        returns 
        filteref image 

        Works by removing pixels too far from the median value
        """
        mdv = np.median(img[starting_mask]) #median depth value
        u_lim = mdv + mdv*deviation #upper limit

        uidx = (img >= u_lim)

        #we stack the two masks and then takes the max in the new axis
        out_img = img
        zero_img = np.zeros_like(img)
        out_img[uidx] = zero_img[uidx]
        return out_img
    
    
    def object_segmentation(self):
        if self.enable_yolo and self.color_image_msg is not None and self.depth_image_msg is not None:
            self.get_logger().debug("Succesfully acquired color and depth image msgs")
            
            # Remove background
            bg_removed, np_color_image, np_depth_image = self.bg_removal(self.color_image_msg, self.depth_image_msg)
            
            # Predict on image "bg_removed"
            results = self.yolo.predict(
                source=np_color_image, #bg_removed
                classes=[ 9, 10, 58, 77], #oggetti Leonardo: traffic light, fire hydrant, potted plant , teddy bear
                show=False,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            self.get_logger().debug("Succesfully predicted")
            
            
            # Go through detections in prediction results
            for detection in results:
                
                # Extract image with yolo predictions
                pred_img = detection.plot()
                
                # MODIFICA: Converti l'immagine in CompressedImage invece di Image
                self.pred_compressed_msg = self.bridge.cv2_to_compressed_imgmsg(pred_img, dst_format="jpg")  # Modifica questa linea
                self.pred_compressed_msg.header.stamp = self.get_clock().now().to_msg()  # Aggiungi timestamp
                self.pred_compressed_msg.header.frame_id = "rover/camera_rgb_optical_frame"  # Aggiungi frame_id opzionale
                self._pred_pub.publish(self.pred_compressed_msg)  # Pubblica il compressed image
                
                # Get number of objects in the scene
                object_boxes = detection.boxes.xyxy.cpu().numpy()
                n_objects = object_boxes.shape[0]

                try:
                    masks = detection.masks.data
                except AttributeError:
                    continue
                
                npmasks = masks.cpu().numpy().astype(np.uint8)
                self.get_logger().debug("Succesfully extracted boxes and masks")
                
                # Declare variables used later
                objects_global_point_clouds = []
                objects_median_center = []
                objects_median_center_transform = []

                for i in range(n_objects):
                    # Mask restituita da YOLO (tipicamente 384x640 dopo preprocessing)
                    single_selection_mask = npmasks[i]

                    # Resize alla risoluzione della depth (848x480)
                    single_selection_mask_resized = cv2.resize(
                        single_selection_mask.astype(np.uint8),
                        (np_depth_image.shape[1], np_depth_image.shape[0]),  # (width, height)
                        interpolation=cv2.INTER_NEAREST
                    )

                    # Binary mask con stessa shape della depth
                    idx = (single_selection_mask_resized == 1)

                    # Depth mask inizializzata
                    single_object_depth = np.zeros_like(np_depth_image)

                    # Ora le dimensioni matchano
                    single_object_depth[idx] = np_depth_image[idx]

                    # Applica il filtro depth
                    single_object_depth = self.filter_depth_object_img(single_object_depth, idx, 0.15)


                    ### Get the pointcloud for the i'th object
                    depth_raw = o3d.geometry.Image(single_object_depth.astype(np.uint16))
                    object_pointcloud = o3d.geometry.PointCloud.create_from_depth_image(depth_raw, self.camera_intrinsics)


                    ## Reduce precision of pointcloud to improve performance
                    voxel_grid = object_pointcloud.voxel_down_sample(0.01)
                    voxel_grid, _ = voxel_grid.remove_radius_outlier(nb_points=30, radius=0.05)

                    # Save i'th object pointcloud to list
                    objects_global_point_clouds.append(voxel_grid)


                    ## Extract pointcloud points to numpy array
                    np_pointcloud = np.asarray(object_pointcloud.points)

                    # Get median xyz value
                    median_center = np.median(np_pointcloud, axis=0)
                    median_center = np.append(median_center, 1)
                    median_center_transformed = np.matmul(self.tf_world_to_optical, median_center)

                    # Save i'th object pointcloud median center to list
                    objects_median_center.append(median_center)
                    objects_median_center_transform.append(median_center_transformed)


                # Item dict creation
                item_dict = {}
                detection_class = detection.boxes.cls.cpu().numpy()
                detection_conf = detection.boxes.conf.cpu().numpy()
                
                for item, n, median_tf in zip(detection_class, range(n_objects), objects_median_center_transform):

                    item_dict[f'item_{n}'] = {'class': detection.names[item],
                                             'position': median_tf.tolist()}
                
                self.item_dict = item_dict
                self.item_dict_str = json.dumps(self.item_dict)
                self.get_logger().info(f"Yolo detected items: {[detection.names[item] for item in detection_class]}")
                
                item_dict_msg = String()
                item_dict_msg.data = self.item_dict_str
                self._item_dict_pub.publish(item_dict_msg)
                detections = []
                for i, box in enumerate(object_boxes):
                    xmin, ymin, xmax, ymax = box
                    label = detection.names[int(detection_class[i])]
                    conf = float(detection_conf[i])
                    detections.append({
                        "xmin": int(xmin),
                        "ymin": int(ymin),
                        "xmax": int(xmax),
                        "ymax": int(ymax),
                        "label": label,
                        "conf": conf
                    })

                self.process_yolo_detections(self.color_image_msg, self.depth_image_msg, detections)
                self.get_logger().debug("Item dictionary succesfully created and published")
            

    def process_yolo_detections(self, color_img_msg, depth_img_msg, detections):
        if self.fx is None:
            self.get_logger().warn("No camera intrinsics yet")
            return

        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="passthrough")

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'rover/camera_rgb_optical_frame'

        for det in detections:
            xmin, ymin, xmax, ymax = det["xmin"], det["ymin"], det["xmax"], det["ymax"]
            label = det["label"]

            u = int((xmin + xmax) / 2.0)
            v = int((ymin + ymax) / 2.0)

            if u < 0 or v < 0 or u >= depth_image.shape[1] or v >= depth_image.shape[0]:
                continue

            depth = depth_image[v, u] / 1000.0  # mm -> m
            if depth == 0 or np.isnan(depth):
                self.get_logger().warn(f"No valid depth for {label}")
                continue

            # Pixel -> camera frame
            X = (u - self.cx) * depth / self.fx
            Y = (v - self.cy) * depth / self.fy
            Z = depth

            pose = Pose()
            pose.position.x = float(X)
            pose.position.y = float(Y)
            pose.position.z = float(Z)
            pose.orientation.w = 1.0

            # Trasforma in base_link da fare se posestamped
            # try:
            #     transform = self.tf_buffer.lookup_transform(
            #         'rover/map', 'rover/camera_rgb_optical_frame', rclpy.time.Time())
            #     pose_stamped = PoseStamped()
            #     pose_stamped.pose = pose
            #     pose_stamped.header = pose_array.header
            #     pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            #     pose = pose_transformed.pose
                
            # except Exception as e:
            #     self.get_logger().warn(f"TF transform failed: {e}")

            pose_array.poses.append(pose)
            self.get_logger().info(f"Added {label} at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})")

        # Pubblica tutte le pose in un singolo array
        self.pose_array_pub.publish(pose_array)

    def shutdown_callback(self):
        self.get_logger().warn("Shutting down...")
        
        

def main(args=None):
    rclpy.init(args=args)

    # Instansiate node class
    vision_node = Yolov11Node()

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(vision_node)

    
    try:
        # Run executor
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        # Shutdown executor
        vision_node.shutdown_callback()
        executor.shutdown()




if __name__ == "__main__":
    main()