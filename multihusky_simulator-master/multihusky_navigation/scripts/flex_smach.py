#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult



def main():
    rospy.init_node('mbf_state_machine')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    # Define userdata
    sm.userdata.goal = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = True
    sm.userdata.error_status = None
    recovery_behaviors = [entry['name'] for entry in rospy.get_param('/move_base/recovery_behaviors')]
    # SM_target_pose = None
    with sm:
        # Goal callback for state WAIT_FOR_GOAL
        def goal_cb(userdata, msg):
            userdata.goal = msg
            return False

        # Monitor topic to get MeshGoal from RViz plugin
        smach.StateMachine.add(
            'WAIT_FOR_GOAL',
            smach_ros.MonitorState(
                '/move_base_simple/goal',
                PoseStamped,
                goal_cb,
                output_keys=['goal']
            ),
            transitions={
                'invalid': 'GET_PATH',
                'valid': 'WAIT_FOR_GOAL',
                'preempted': 'preempted'
            }
        )

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base/get_path',
                GetPathAction,
                goal_slots=['target_pose'],
                result_slots=['path'],
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'WAIT_FOR_GOAL',
                'preempted': 'WAIT_FOR_GOAL'
            },
            remapping={
                'target_pose': 'goal'
            }
        )

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base/exe_path',
                ExePathAction,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'WAIT_FOR_GOAL',
                'aborted': 'RECOVERY',
                'preempted': 'WAIT_FOR_GOAL'
            }
        )

        # Goal callback for state RECOVERY
        def recovery_path_goal_cb(userdata, goal):
            # Cycle through all behaviors
            behavior = recovery_behaviors[0]
            print 'Using recovery behavior: ', behavior
            goal.behavior = behavior

        def recovery_result_cb(userdata, status, result):
            if result.outcome == RecoveryResult.SUCCESS:
                return 'succeeded'
            elif result.outcome == RecoveryResult.CANCELED:
                return 'preempted'
            else:
                return 'aborted'

        # Recovery
        #@cb_interface(output_keys=['outcome', 'message'], outcomes=['succeeded', 'failure'])
        smach.StateMachine.add(
            'RECOVERY',
            smach_ros.SimpleActionState(
                'move_base/recovery',
                RecoveryAction,
                goal_cb=recovery_path_goal_cb,
                result_cb=recovery_result_cb
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

    def goal_callback(target_pose):
        """
        Called, if a new goal is given. If the statemachine is running, it will be preempted
        and as a result restarted in the main loop
        """
        if sm is not None and sm.get_active_states()[0] != 'WAIT_FOR_GOAL':
            sm.request_preempt()
            # Save target_pose for the restart
            global SM_target_pose
            SM_target_pose = target_pose
    subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    # Create and start introspection server

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    rospy.loginfo("Restarting state machine")

    # Execute SMACH plan
    sm.execute()

    # Wait for interrupt and stop introspection server
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
