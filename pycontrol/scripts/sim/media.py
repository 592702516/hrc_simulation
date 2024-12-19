import rospy
from pycontrol.camera_sim import AzureKinectCamera  # Replace 'your_package_name' with the actual package name

def main():
    rospy.init_node("camera_data_check_node")
    
    # Initialize the camera class
    camera = AzureKinectCamera()
    rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

    color_received = False
    depth_received = False

    while not rospy.is_shutdown():
        # Retrieve the latest image and depth data
        rgb_data = camera.get_image()
        depth_data = camera.get_depth()

        # Check if RGB data is available and print only once
        if rgb_data is not None and not color_received:
            rospy.loginfo("Color data received successfully.")
            rospy.loginfo(f"Color data: {rgb_data}")
            color_received = True

        # Check if depth data is available and print only once
        if depth_data is not None and not depth_received:
            rospy.loginfo("Depth data received successfully.")
            rospy.loginfo(f"Depth data: {depth_data}")       
            depth_received = True

        # If both data types have been received, print success and break the loop
        if color_received and depth_received:
            rospy.loginfo("Color and depth data both received: Success")
            break

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == "__main__":
    main()

