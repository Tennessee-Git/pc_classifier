import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pc_c_interfaces.msg import LidarData
from pc_c.Utils import convert_to_array, ade_palette
from pc_c.Constants import *
from transformers import AutoImageProcessor, UperNetForSemanticSegmentation

import io
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from PIL import Image as PImage


class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier')
        self.subscription = self.create_subscription(
            LidarData,
            LIDAR_DATA_TOPIC,
            self.classify_callback,
            10
        )
        self.br = CvBridge()
        self.publisher_ = self.create_publisher(Image, CLASSIFIED_IMAGE_TOPIC, 10)
        print("----------Initializing model---------")
        self.image_processor = AutoImageProcessor.from_pretrained("openmmlab/upernet-convnext-small")
        self.model = UperNetForSemanticSegmentation.from_pretrained("openmmlab/upernet-convnext-small")


    def classify_callback(self, msg):
        xyz = convert_to_array(msg.points)
        self.get_logger().info(f'RECEIVED {xyz.shape}')
        img = self.lidar_to_2d_front_view(xyz)
        outputs = self.segment_image(img)
        self.publisher_.publish(
            self.br.cv2_to_imgmsg(self.visualize_output(outputs, img), 'bgr8')
        )

    def lidar_to_2d_front_view(self, points):
        """ Takes points in 3D space from LIDAR data and projects them to a 2D
            "front view" image, and saves that image.

        Args:
            points: (np array)
                The numpy array containing the lidar points.
                The shape should be Nx4
                - Where N is the number of points, and
                - each point is specified by 4 values (x, y, z, reflectance)
            v_res: (float)
                vertical resolution of the lidar sensor used.
            h_res: (float)
                horizontal resolution of the lidar sensor used.
            v_fov: (tuple of two floats)
                (minimum_negative_angle, max_positive_angle)
            y_fudge: (float)
                A hacky fudge factor to use if the theoretical calculations of
                vertical range do not match the actual data.

                For a Velodyne HDL 64E, set this value to 5.
        """

        x_lidar = points[:, 0]
        y_lidar = points[:, 1]
        z_lidar = points[:, 2]
        # Distance relative to origin when looked from top
        d_lidar = np.sqrt(x_lidar ** 2 + y_lidar ** 2)
        # Absolute distance relative to origin
        # d_lidar = np.sqrt(x_lidar ** 2 + y_lidar ** 2 + z_lidar ** 2)

        v_fov_total = -LIDAR_V_FOV[0] + LIDAR_V_FOV[1]

        # Convert to Radians
        v_res_rad = LIDAR_V_RES * (np.pi/180)
        h_res_rad = LIDAR_H_RES * (np.pi/180)

        # PROJECT INTO IMAGE COORDINATES
        x_img = np.arctan2(-y_lidar, x_lidar)/ h_res_rad
        y_img = np.arctan2(z_lidar, d_lidar)/ v_res_rad

        # SHIFT COORDINATES TO MAKE 0,0 THE MINIMUM
        x_min = -360.0 / LIDAR_H_RES / 2  # Theoretical min x value based on sensor specs
        x_img -= x_min              # Shift
        x_max = 360.0 / LIDAR_H_RES       # Theoretical max x value after shifting

        y_min = LIDAR_V_FOV[0] / LIDAR_V_RES    # theoretical min y value based on sensor specs
        y_img -= y_min              # Shift
        y_max = v_fov_total / LIDAR_V_RES # Theoretical max x value after shifting

        y_max += LIDAR_Y_EXTRA_SIZE            # Fudge factor if the calculations based on
                                    # spec sheet do not match the range of
                                    # angles collected by in the data.

        pixel_values = -d_lidar

        # PLOT THE IMAGE
        cmap = "Dark2"            # Color map to use
        dpi = 500               # Image resolution
        fig, ax = plt.subplots(figsize=(x_max/dpi, y_max/dpi), dpi=dpi)
        ax.scatter(x_img,y_img, s=1, c=pixel_values, linewidths=0, alpha=1, cmap=cmap)
        ax.axis('scaled')              # {equal, scaled}
        ax.xaxis.set_visible(False)    # Do not draw axis tick marks
        ax.yaxis.set_visible(False)    # Do not draw axis tick marks
        plt.xlim([0, x_max])   # prevent drawing empty space outside of horizontal FOV
        plt.ylim([0, y_max])   # prevent drawing empty space outside of vertical FOV
        ax.set_facecolor("black")
        buf = io.BytesIO()
        fig.savefig(buf, dpi=500, bbox_inches='tight', pad_inches=0)
        buf.seek(0)
        img = PImage.open(buf)
        # fig.show()
        return img.convert("RGB")

    def segment_image(self, img):
        pixel_values = self.image_processor(images=img.convert("RGB"), return_tensors="pt").pixel_values
        outputs = self.model(pixel_values)
        return outputs

    def visualize_output(self, outputs, img):
        seg = self.image_processor.post_process_semantic_segmentation(outputs, target_sizes=[img.size[::-1]])[0]
        color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8)
        palette = ade_palette()
        for label, color in enumerate(palette):
            color_seg[seg==label, :] = color
        img = np.array(img) * 0.3 + color_seg * 0.7
        img = img.astype(np.uint8)
        return img


def main(args=None):
    rclpy.init(args=args)
    classifier = ClassifierNode()
    rclpy.spin(classifier)

    classifier.vis.destroy_window()
    classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
