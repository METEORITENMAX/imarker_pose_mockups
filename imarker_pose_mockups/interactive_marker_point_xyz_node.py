import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rclpy.publisher
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import math
from visualization_msgs.msg import MarkerArray

class InteractiveMarkerPointXYZNode(Node):


    # Set class attributes based on parameter values
    def set_parameters(self):

        ## NOTE: There is also a function 'get_parameters' but this code approach seems to be cleaner

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.marker_name = self.get_parameter('marker_name').get_parameter_value().string_value
        self.marker_description = self.get_parameter('marker_description').get_parameter_value().string_value
        # TODO: Check if marker type is valid
        self.marker_type = self.get_parameter('marker_type').get_parameter_value().integer_value
        self.initial_pose_x = self.get_parameter('initial_pose.x').get_parameter_value().double_value
        self.initial_pose_y = self.get_parameter('initial_pose.y').get_parameter_value().double_value
        self.initial_pose_z = self.get_parameter('initial_pose.z').get_parameter_value().double_value
        self.box_scale_x = self.get_parameter('box_scale.x').get_parameter_value().double_value
        self.box_scale_y = self.get_parameter('box_scale.y').get_parameter_value().double_value
        self.box_scale_z = self.get_parameter('box_scale.z').get_parameter_value().double_value
        self.box_color_r = self.get_parameter('box_color.r').get_parameter_value().double_value
        self.box_color_g = self.get_parameter('box_color.g').get_parameter_value().double_value
        self.box_color_b = self.get_parameter('box_color.b').get_parameter_value().double_value
        self.box_color_a = self.get_parameter('box_color.a').get_parameter_value().double_value



    def add_move_control(self, name, w, x, y, z, mode):
        move_control = InteractiveMarkerControl()
        move_control.name = name
        move_control.orientation.w = w
        move_control.orientation.x = x
        move_control.orientation.y = y
        move_control.orientation.z = z
        move_control.interaction_mode = mode
        self.marker.controls.append(move_control)



    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        self.get_logger().info('Feedback from marker: {}'.format(feedback.marker_name))

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            __msg__pose3d = Pose()

            __msg__pose3d.position.x = feedback.pose.position.x
            __msg__pose3d.position.y = feedback.pose.position.y
            __msg__pose3d.position.z = feedback.pose.position.z

            if not self.flag_useTimer:
                self.__pub__pose3d.publish(__msg__pose3d)



    def __init__(self):

        super().__init__('interactive_marker_point_xyz_node')

        self.get_logger().info('[Setup] Begin')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id', 'base_link'),
                ('marker_name', 'my_marker'),
                ('marker_description', 'Simple 3-DOF Control'),
                # NOTE: 2 should be a SPHERE
                ('marker_type', 2),
                ('initial_pose.x', 0.0),
                ('initial_pose.y', 0.0),
                ('initial_pose.z', 0.0),
                ('box_scale.x', 0.1),
                ('box_scale.y', 0.1),
                ('box_scale.z', 0.1),
                ('box_color.r', 0.0),
                ('box_color.g', 1.0),
                ('box_color.b', 0.0),
                ('box_color.a', 1.0),
            ]
        )

        self.set_parameters()

        ## TODO: Replace with parameter
        self.flag_useTimer = False
        self.__topic__pose3d = self.get_name()+"/out/pose3d/"+self.marker_name

        self.__pub__pose3d = self.create_publisher(Pose, self.__topic__pose3d, 10)

        # Setup interactive marker server
        self.server = InteractiveMarkerServer(self, 'interactive_marker_server'+'_'+self.marker_name)

        self.marker = InteractiveMarker()
        self.marker.header.frame_id = self.frame_id
        self.marker.name = self.marker_name
        self.marker.description = self.marker_description
        self.marker.pose.position.x = self.initial_pose_x
        self.marker.pose.position.y = self.initial_pose_y
        self.marker.pose.position.z = self.initial_pose_z
        self.marker.scale = 1.5

        # Set the initial pose of the marker
        self.marker.pose.position = Point(x=self.initial_pose_x, y=self.initial_pose_y, z=self.initial_pose_z)

        # Create a box marker
        box_marker = Marker()
        box_marker.type = self.marker_type
        box_marker.scale.x = self.box_scale_x
        box_marker.scale.y = self.box_scale_y
        box_marker.scale.z = self.box_scale_z
        box_marker.color.r = self.box_color_r
        box_marker.color.g = self.box_color_g
        box_marker.color.b = self.box_color_b
        box_marker.color.a = self.box_color_a

        # Create a control that contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        box_control.independent_marker_orientation = True


        box_control.markers.append(box_marker)
        self.marker.controls.append(box_control)

        # Create move controls for X, Y, and Z axes
        self.add_move_control("move_x", 1.0, 1.0, 0.0, 0.0, InteractiveMarkerControl.MOVE_AXIS)
        self.add_move_control("move_y", 1.0, 0.0, 1.0, 0.0, InteractiveMarkerControl.MOVE_AXIS)
        self.add_move_control("move_z", 1.0, 0.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS)


        self.server.insert(self.marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()
        # Subscription to MarkerArray topic
        self.marker_array_sub = self.create_subscription(MarkerArray, '/astar/example/octomapViz', self.marker_array_callback, 10)

        self.create_subscription(Pose, '/update_marker_pose/'+self.marker_name, self.update_marker_pose_callback, 10)
        self.marker_array = []
        self.get_logger().info('[Setup] End')

    def marker_array_callback(self, marker_array: MarkerArray):
        self.marker_array = marker_array

    def update_marker_pose_callback(self, msg: Pose):
        if not msg:
            self.get_logger().warn('No Pose received.')
            return

        self.get_logger().info(f'Received message: {msg}')

        self.marker.pose = msg
        self.server.insert(self.marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

        feedback = InteractiveMarkerFeedback()
        feedback.marker_name = self.marker_name
        feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        feedback.pose = self.marker.pose
        self.process_feedback(feedback)
        self.get_logger().warn('Marker Updated.')


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerPointXYZNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()