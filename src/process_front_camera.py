import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class ImageProcessor(threading.Thread):
    """Separate thread for heavy OpenCV processing."""
    def __init__(self):
        super().__init__()
        self.cv_image = None
        self.running = True
        self.lock = threading.Lock()

    def set_image(self, cv_image):
        """Safely set a new image for processing."""
        with self.lock:
            self.cv_image = cv_image.copy()

    def run(self):
        """Continuously process images in a separate thread."""
        while self.running:
            if self.cv_image is not None:
                with self.lock:
                    img_copy = self.cv_image.copy()

                processed_image = self.process_image(img_copy)
                self.display_image(processed_image)

            time.sleep(0.03)  # Prevent CPU overuse

    def process_image(self, cv_image):
        """Heavy OpenCV processing function."""
        try:
            # ================================
            # Replace this with any heavy OpenCV processing
            # processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # processed_image = cv2.Canny(processed_image, 100, 200)
            processed_image = cv_image
            # =================================
            return processed_image
        except Exception as e:
            print(f"OpenCV processing error: {e}")
            return cv_image  # Return original if processing fails

    def display_image(self, cv_image):
        """Display processed image using OpenCV."""
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)  # Required for OpenCV to update window


class ImageSubscriber(Node):
    def __init__(self, processor):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/front_camera',  # Change if using a different topic
            self.image_callback,
            10
        )
        self.br = CvBridge()
        self.processor = processor

    def image_callback(self, msg):
        """Converts ROS 2 Image message to OpenCV format and updates processing thread."""
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.processor.set_image(cv_image)  # Send image to processing thread
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main():
    rclpy.init()
    processor = ImageProcessor()
    processor.start()  # Start image processing thread

    node = ImageSubscriber(processor)

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        processor.running = False
        processor.join()  # Ensure the processing thread stops
        cv2.destroyAllWindows()  # Close OpenCV windows
        rclpy.shutdown()


if __name__ == '__main__':
    main()
