import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            
        self.bridge = CvBridge()
        
        # --- MEMORY FOR STEP F ---
        self.last_error = 0.0 
        
        self.get_logger().info("Full Visual Servoing System Online!")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape
            image_center_x = width // 2
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get the radius for Step G
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    error = cx - image_center_x
                    
                    # Update memory for Step F every frame we see the ball
                    self.last_error = error 
                    
                    # Draw visuals
                    cv2.circle(cv_image, (cx, cy), int(radius), (0, 255, 255), 2) 
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1) 
                    cv2.putText(cv_image, f"Radius: {int(radius)} px", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                    
                    # Send data to controller
                    self.control_robot(error, radius, object_found=True)
            else:
                # We lost the object! Trigger recovery.
                self.control_robot(0.0, 0.0, object_found=False)
                
            cv2.imshow("Turtlebot Vision", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def control_robot(self, error, radius, object_found):
        twist = Twist()
        
        if object_found:
            # 1. P-Controller for steering
            Kp = 0.002
            twist.angular.z = -float(Kp * error)
            
            # 2. Forward movement logic
            if abs(error) < 30: # If we are looking mostly straight at it
                
                # --- STEP G: STOPPING CONDITION ---
                if radius > 120: 
                    twist.linear.x = 0.0
                    self.get_logger().info("Target reached! Stopping.")
                else:
                    twist.linear.x = 0.15 
                    self.get_logger().info("Target centered! Approaching...")
            else:
                # Still turning to face it, don't move forward yet
                twist.linear.x = 0.0  
                
        else:
            # --- STEP F: RECOVERY MODE ---
            twist.linear.x = 0.0 # Don't drive blind
            search_speed = 0.4   # Spin relatively fast to find it
            
            if self.last_error > 0:
                self.get_logger().info("Lost it on the right! Spinning right...")
                twist.angular.z = -search_speed 
            else:
                self.get_logger().info("Lost it on the left! Spinning left...")
                twist.angular.z = search_speed
            
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
