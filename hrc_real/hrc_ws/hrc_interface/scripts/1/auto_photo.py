import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pynput import keyboard

class PhotoTakerNode:
    def __init__(self):
        rospy.init_node('photo_taker_node', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber to the RGB image topic
        self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image, self.rgb_callback)

        # Create a directory to save photos if it doesn't exist
        self.save_path = os.path.expanduser('~/yolo_photo')
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        # Variable to store the latest image
        self.latest_rgb = None

        # Set up a listener for the 's' keypress
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def rgb_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error processing RGB image: {e}")

    def save_image(self, filename):
        if self.latest_rgb is not None:
            # Resize image to 1920x1080
            resized_image = cv2.resize(self.latest_rgb, (1920, 1080))
            
            # Save the resized image
            file_path = os.path.join(self.save_path, filename)
            cv2.imwrite(file_path, resized_image)
            rospy.loginfo(f"Image saved as {file_path}")
        else:
            rospy.logwarn("No image to save")

    def on_press(self, key):
        try:
            if key.char == 's':  # Check if the pressed key is 's'
                rospy.loginfo("Taking a photo!")
                timestamp = rospy.get_time()
                filename = f"photo_{timestamp:.2f}.jpg"
                self.save_image(filename)
        except AttributeError:
            pass  # Handle special keys, if any

    def run(self):
        rospy.spin()  # Keep the ROS node running

if __name__ == "__main__":
    try:
        node = PhotoTakerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
