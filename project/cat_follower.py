import rclpy
from geometry_msgs.msg import Twist, Vector3, Point
from rclpy.node import Node

import tf2_ros

import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from rcl_interfaces.msg import SetParametersResult

CV2_THRESHOLD = 127

class CatFollower(Node):
    def __init__(self):
        super().__init__('object_recognition')

        # TODO: Podaci o kameri - provjerite na linku jesu li OK
        # https://www.raspberrypi.com/documentation/accessories/camera.html
        # https://stackoverflow.com/questions/70695409/how-to-go-from-pixel-coordinates-to-angle-off-the-optical-axis-object-detection
        self.f = 2571.43
        self.u0 = 959.5

        # Parameter declaration
        self.declare_parameter('video_source', 0)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('threshold', 0.3)

        self.declare_parameter('weights_file', '')
        self.declare_parameter('cfg_file', '')
        self.declare_parameter('classes_file', '')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_detected_objects')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing important values
        self.get_logger().info(f'Video source: {self.get_parameter("video_source").value}')
        self.get_logger().info(f'Frame rate: {self.get_parameter("frame_rate").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Outputing weights file path
        self.get_logger().info(f'Weights file: {os.path.expanduser(self.get_parameter("weights_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('weights_file').value)):
            self.get_logger().error(f'Weights file path is not correct')
            exit()

        # Outputing config file path
        self.get_logger().info(f'Config file: {os.path.expanduser(self.get_parameter("cfg_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('cfg_file').value)):
            self.get_logger().error(f'Config file path is not correct')
            exit()

        # Outputing classes file path
        self.get_logger().info(f'Classes file: {os.path.expanduser(self.get_parameter("classes_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('classes_file').value)):
            self.get_logger().error(f'Classes file path is not correct')
            exit()

        # Initializing the video capture for a specific file camera index
        self.video_capture = cv2.VideoCapture(int(self.get_parameter("video_source").value))

        # Check if the video is properly opened
        if not self.video_capture.isOpened():
            self.get_logger().error(f'Cannot open the video source')
            exit()

        # Get the FPS value from the parameter or the video FPS
        frame_rate = self.get_framerate(int(self.get_parameter("frame_rate").value))
        self.get_logger().info(f'Video FPS: {frame_rate}')

        # Initializing a timer based callback
        self.timer = self.create_timer(1.0 / frame_rate, self.timer_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output image with detected people
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        # Loading class labels
        self.classes = open(os.path.expanduser(self.get_parameter("classes_file").value)).read().strip().split("\n")
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # Initialize the CNN darknet detector
        self.net_detector = cv2.dnn.readNetFromDarknet(os.path.expanduser(self.get_parameter('cfg_file').value), os.path.expanduser(self.get_parameter('weights_file').value))

        self.cmd_vel_publisher = self.create_publisher(Twist, 'robot/cmd_vel', 10)
        self.cmd_vel_publisher  # prevent unused variable warning

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.laser_subscription  # prevent unused variable warning

    def _euler(self, q):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = np.arctan2(t3, t4)

        return Vector3(x=roll_x, y=pitch_y, z=yaw_z)

    def _point_transform(self, point, trans, rot):
        return Point(x=(point.x * np.cos(rot) - point.y * np.sin(rot) + trans.x),
                    y=(point.x * np.sin(rot) + point.y * np.cos(rot) + trans.y),
                    z=(point.z + trans.z))

    def get_framerate(self, frame_rate):
        if frame_rate <= 0:
            self.get_logger().error(f'Frame rate must be more then 0')
            exit()

        return frame_rate

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            # If one of the changed parameters is frame_rate period of the timer should be changed accordingly
            if p.name == 'frame_rate':
                self.timer.timer_period_ns = 1000000000 // self.get_framerate(int(p.value))
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        # Fetch the next frame from the video
        ret, frame = self.video_capture.read()

        # If return value is not True, there was an error
        if not ret:
            self.get_logger().error(f'Error while reading the video source')
            return

        # Convert the fetched OpenCV image frame to Image message
        msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Setup the Image message header - including the frame number and timestamp
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = str(self.video_capture.get(cv2.CAP_PROP_POS_FRAMES))

        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)
        (h, w) = frame.shape[:2]

        # Determine only the output layer names from YOLO
        layer_names = self.net_detector.getLayerNames()
        layer_names = [layer_names[i - 1] for i in self.net_detector.getUnconnectedOutLayers()]

        # Construct a blob from the input image and then perform a forward pass of the YOLO object detector, resulting in bounding boxes and associated probabilities
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net_detector.setInput(blob)
        layerOutputs = self.net_detector.forward(layer_names)

        boxes = []
        confidences = []
        classIDs = []

        # Loop over each of the layer outputs
        for output in layerOutputs:
            # Loop over each of the detections
            for detection in output:
                # Extract the class ID and confidence (i.e., probability) of the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # Filter out weak predictions
                # by ensuring the detected probability is greater than the minimum probability
                if confidence > float(self.get_parameter('confidence').value):
                    # Scale the bounding box coordinates back relative to the size of the image,
                    # keeping in mind that YOLO actually returns the center (x, y)-coordinates of the bounding box
                    # followed by the boxes' width and height
                    box = detection[0:4] * np.array([w, h, w, h])
                    (centerX, centerY, width, height) = box.astype("int")

                    # TODO: Ovdje tražiti mačku?
                    # Predati centerX trackeru
                    # možete handleati slučaj di ima više mačaka pa uzme najbližu
                    # ili samo uzeti prvo očitanje

                    # Use the center (x, y)-coordinates to derive the top left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # Update list of bounding box coordinates, confidences, and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # Removing some boxes using non maximum supression
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, float(self.get_parameter('confidence').value), float(self.get_parameter('threshold').value))

        # Ensure at least one detection exists
        if len(idxs) > 0:
            for i in idxs.flatten():
                # Extract the bounding box coordinates
                (x, y, w, h) = boxes[i]

                # Draw a bounding box rectangle on the image
                color = [int(c) for c in self.colors[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # Writing the class label and confidence score on the image
                text = "{}: {:.3f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Convert the OpenCV image frame with detected boxes to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the image
        self.image_publisher.publish(image_msg)

        # TODO: Možda može biti korisno za praćenje mačke
        # yaw_vel = np.atan((self.u0 - x) / self.f)
        #
        # velocity = Twist()
        # velocity.linear = Vector3(x=0, y=0.0, z=0.0)
        # velocity.angular = Vector3(x=0.0, y=0.0, z=yaw_vel)

    # TODO: Mjeri udaljenosti do očitanja
    # prilagodite brzine, usporavanje, zaustavljanje i sl. ovisno o očitanjima
    def laser_callback(self, msg):
        if not msg.ranges:
            return

        closest = min(msg.ranges)

        if closest >= float('inf'):
            return

        closest_angle = msg.ranges.index(closest) * msg.angle_increment

        try:
            trans = self.tfBuffer.lookup_transform('odom',
                                                   'base_scan',
                                                   rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        yaw = self._euler(trans.transform.rotation).z

        goal_x = closest * np.cos(closest_angle)
        goal_y = closest * np.sin(closest_angle)
        goal = Point(x=goal_x, y=goal_y, z=0.0)

        self.goal = self._point_transform(goal, trans.transform.translation, yaw)
        self.goal.z = 0.0
        self.marker_maker()

    # TODO: image callback
    def image_callback(self, msg):
        frame = CvBridge().imgmsg_to_cv2(msg)
        gray_picture = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        	
        (thresh, black_and_white_picture) = cv2.threshold(gray_picture, CV2_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        cv2_moments = cv2.moments(black_and_white_picture)

        if cv2_moments["m00"] != 0:
            x = int(cv2_moments["m10"] / cv2_moments["m00"])
            y = int(cv2_moments["m01"] / cv2_moments["m00"])
            self.is_cat_in_frame = True
        else:
            x = 0
            y = 0

        if self.is_cat_in_frame: self.angular_speed = np.arctan((self.u0 - x) / self.f)

        cv2.circle(black_and_white_picture, (x, y), 3, (118, 208, 16), -1)

        image_message = CvBridge().cv2_to_imgmsg(black_and_white_picture, encoding="mono8")
        image_message.header = msg.header
        self.frame_pub.publish(image_message)

        cmdMsg = Twist() 
        cmdMsg.angular.z = float(self.angular_speed)

        self.cmd_pub.publish(cmdMsg)


def main(args=None):
    rclpy.init(args=args)

    cat_follower = CatFollower()
    rclpy.spin(cat_follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()