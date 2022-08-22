"""
PandaROSRobot module defines the PandaROSRobot type

PandaROSRobot provides robot-specific callbacks for recovery and setting impedance.

.. codeauthor:: Gavin Suddreys
"""
import rospy
import actionlib
import roboticstoolbox as rtb

from armer.robots import ROSRobot

from std_srvs.srv import EmptyRequest, EmptyResponse
from controller_manager_msgs.srv import SwitchController

from armer_msgs.msg import ManipulatorState

from armer_msgs.srv import \
    SetCartesianImpedanceRequest, \
    SetCartesianImpedanceResponse

from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal
from franka_msgs.msg import FrankaState

class PandaROSRobot(ROSRobot):
    def __init__(self,
                 robot: rtb.robot.Robot,
                 controller_name: str = None,
                 recover_on_estop: bool = True,
                 *args,
                 **kwargs):

        super().__init__(robot, *args, **kwargs)
        self.controller_name = controller_name \
            if controller_name else self.joint_velocity_topic.split('/')[1]

        self.recover_on_estop = recover_on_estop
        self.last_estop_state = 0

        self.qr = [0.01785449910869703, -0.782578660063219, -0.010252165552294044, -2.356678446585672, -0.009573905508763608, 1.5746592243451132, 0.7909650121199191]

        # Controller switcher needed to temporarily disable controller while setting impedance
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switcher_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Franka state subscriber
        self.franka_state_subscriber = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.franka_state_cb
        )
        self.last_estop_state = 0
        self.franka_state = None

        # Error recovery action server
        self.reset_client = actionlib.SimpleActionClient('/franka_control/error_recovery', ErrorRecoveryAction)

    def recover_cb(self, req: EmptyRequest) -> EmptyResponse: # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs

        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        print('Recovering')
        self.reset_client.send_goal(ErrorRecoveryGoal())
        self.reset_client.wait_for_result()
        return EmptyResponse()

    def set_cartesian_impedance_cb(  # pylint: disable=no-self-use
            self,
            request: SetCartesianImpedanceRequest) -> SetCartesianImpedanceResponse:
        """
        ROS Service Callback
        Set the 6-DOF impedance of the end-effector. Higher values should increase the stiffness
        of the robot while lower values should increase compliance

        :param request: The numeric values representing the EE impedance (6-DOF) that
        should be set on the arm
        :type request: GetNamedPoseConfigsRequest
        :return: True if the impedence values were updated successfully
        :rtype: GetNamedPoseConfigsResponse
        """
        if self.moving:
            self.preempt()

        with self.lock():
            rospy.sleep(0.1)

            self.switcher.switch_controller(None)
            result = self.cartesian_impedance_proxy(request.cartesian_impedance)
            self.switcher.switch_controller(self.controller_name)

            return SetCartesianImpedanceResponse(success=result.success, error=result.error)

    def get_state(self):
        state = super().get_state()

        if self.franka_state:
            state.cartesian_contact = self.franka_state.cartesian_contact
            state.cartesian_collision = self.franka_state.cartesian_collision

            state.errors |= ManipulatorState.LOCKED if self.franka_state.robot_mode == FrankaState.ROBOT_MODE_OTHER else 0
            state.errors |= ManipulatorState.ESTOP if self.franka_state.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED else 0
            state.errors |= ManipulatorState.COLLISION if any(state.cartesian_collision) else 0

            for n in self.franka_state.last_motion_errors.__slots__:
                if self.franka_state.robot_mode != 2 and getattr(self.franka_state.last_motion_errors, n):
                    if n in ['joint_position_limits_violation',
                            'joint_velocity_violation',
                            'joint_position_motion_generator_start_pose_invalid',
                            'joint_motion_generator_position_limits_violation',
                            'joint_motion_generator_velocity_limits_violation',
                            'joint_motion_generator_velocity_discontinuity',
                            'joint_motion_generator_acceleration_discontinuity']:
                        state.errors |= ManipulatorState.JOINT_LIMIT_VIOLATION

                    elif n in ['cartesian_position_limits_violation',
                            'cartesian_velocity_violation',
                            'cartesian_velocity_profile_safety_violation',
                            'cartesian_position_motion_generator_start_pose_invalid',
                            'cartesian_motion_generator_elbow_limit_violation',
                            'cartesian_motion_generator_velocity_limits_violation',
                            'cartesian_motion_generator_velocity_discontinuity',
                            'cartesian_motion_generator_acceleration_discontinuity',
                            'cartesian_motion_generator_elbow_sign_inconsistent',
                            'cartesian_motion_generator_start_elbow_invalid',
                            'cartesian_motion_generator_joint_position_limits_violation',
                            'cartesian_motion_generator_joint_velocity_limits_violation',
                            'cartesian_motion_generator_joint_velocity_discontinuity',
                            'cartesian_motion_generator_joint_acceleration_discontinuity',
                            'cartesian_position_motion_generator_invalid_frame']:
                        state.errors |= ManipulatorState.CARTESIAN_LIMIT_VIOLATION

                    elif n in ['force_control_safety_violation',
                            'joint_reflex',
                            'cartesian_reflex',
                            'force_controller_desired_force_tolerance_violation'
                            'joint_p2p_insufficient_torque_for_planning'
                            'tau_j_range_violation']:
                        state.errors |= ManipulatorState.TORQUE_LIMIT_VIOLATION

                    elif state.errors == 0:
                        state.errors |= ManipulatorState.OTHER

        if self.franka_state and self.franka_state.robot_mode == FrankaState.ROBOT_MODE_IDLE:
            if self.recover_on_estop and self.last_estop_state == 1:
                self.recover_cb(EmptyRequest())
        else:
            if state.errors & ManipulatorState.OTHER == ManipulatorState.OTHER:
                self.recover_cb(EmptyRequest())

        self.last_estop_state = 1 if self.franka_state and \
            self.franka_state.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED else 0

        return state

    def franka_state_cb(self, msg):
        self.franka_state = msg
