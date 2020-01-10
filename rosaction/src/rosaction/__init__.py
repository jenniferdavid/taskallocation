# -*- coding: utf-8 -*-

"""ROS action client."""

import yaml
import roslib
import StringIO
import rosgraph
import rostopic
import actionlib
from actionlib_msgs.msg import GoalStatus


__author__ = "Anass Al-Wohoush"


def get_action_class(action):
    """Gets the corresponding ROS action class.

    Args:
        action: ROS action name.

    Returns:
        Action message class. None if not found.
    """
    goal_msg_type = rostopic.get_topic_type("{}/goal".format(action))[0]

    # Verify goal message was found.
    if not goal_msg_type:
        return None

    # Goal message name is the same as the action message name + 'Goal'.
    action_msg_type = goal_msg_type[:-4]

    return roslib.message.get_message_class(action_msg_type)


def get_goal_type(action):
    """Gets the corresponding ROS action goal type.

    Args:
        action: ROS action name.

    Returns:
        Goal message type. None if not found.
    """
    msg_type = rostopic.get_topic_type("{}/goal".format(action))[0]

    # Replace 'ActionGoal' with 'Goal'.
    return msg_type[:-10] + "Goal"


def get_goal_class(action):
    """Gets the corresponding ROS action goal message class.

    Args:
        action: ROS action name.

    Returns:
        Goal message class. None if not found.
    """
    msg_type = get_goal_type(action)

    # Verify message type was found.
    if not msg_type:
        return None

    return roslib.message.get_message_class(msg_type)


def get_feedback_type(action):
    """Gets the corresponding ROS action feedback type.

    Args:
        action: ROS action name.

    Returns:
        Feedback message type. None not found.
    """
    msg_type = rostopic.get_topic_type("{}/feedback".format(action))[0]

    # Replace 'ActionFeedback' with 'Feedback'.
    return msg_type[:-14] + "Feedback"


def get_feedback_class(action):
    """Gets the corresponding ROS action feedback message class.

    Args:
        action: ROS action name.

    Returns:
        Feedback message class. None not found.
    """
    msg_type = get_feedback_type(action)

    # Verify message type was found.
    if not msg_type:
        return None

    return roslib.message.get_message_class(msg_type)

def get_result_type(action):
    """Gets the corresponding ROS action result type.

    Args:
        action: ROS action name.

    Returns:
        Result message type. None if not found.
    """
    msg_type = rostopic.get_topic_type("{}/result".format(action))[0]

    # Replace 'ActionResult' with 'Result'.
    return msg_type[:-12] + "Result"


def get_result_class(action):
    """Gets the corresponding ROS action result message class.

    Args:
        action: ROS action name.

    Returns:
        Result message class. None if not found.
    """
    msg_type = get_result_type(action)

    # Verify message type was found.
    if not msg_type:
        return None

    return roslib.message.get_message_class(msg_type)


def get_action_list():
    """Gets list of registered ROS actions.

    Returns:
        List of ROS action names.
    """
    master = rosgraph.Master("/rosaction")

    # Get list of subscriptions from ROS master.
    _, subscriptions, _ = master.getSystemState()

    # Separate topic names from node names.
    registered_topics, _ = zip(*subscriptions)

    # All actions must have both '/goal' and '/cancel' topics.
    actions = [
        topic[:-5]  # Strip '/goal' from end of topic name.
        for topic in registered_topics
        if topic.endswith("/goal") and
        topic.replace("/goal", "/cancel") in registered_topics
    ]

    # Sort list for printing convenience.
    actions.sort()

    return actions


def create_goal_from_yaml(action, msg):
    """Constructs goal from YAML encoded string.

    Args:
        action: ROS action name.
        msg: YAML encoded message.

    Returns:
        ROS action goal instance.
    """
    # Initialize message.
    goal_cls = get_goal_class(action)
    goal_msg = goal_cls()

    # Load message dictionary from YAML encoded string.
    msg_dict = yaml.load(msg)

    # Fill message arguments.
    roslib.message.fill_message_args(goal_msg, [msg_dict])

    return goal_msg


def create_action_client(action):
    """Creates corresponding ROS action client.

    Args:
        action: ROS action name.

    Returns:
        Corresponding actionlib.SimpleActionClient instance.
        None if action server not found.
    """
    action_cls = get_action_class(action)
    client = actionlib.SimpleActionClient(action, action_cls)
    return client


def stringify_status(status):
    """Returns human-readable format of ROS action status enumeration.

    Args:
        status: GoalStatus enumeration.

    Returns:
        Human-readable string.
    """
    enum = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLING: "RECALLING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST"
    }
    return enum[status]
