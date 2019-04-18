from __future__ import print_function

import math

from roslibpy import Message
from roslibpy import Topic
from roslibpy.actionlib import Goal
from roslibpy.event_emitter import EventEmitterMixin

from compas_fab.backends.ros.messages.direct_ur import URGoal
from compas_fab.backends.ros.messages.direct_ur import URMovel
from compas_fab.backends.ros.messages.direct_ur import URPose
from compas_fab.backends.ros.messages.direct_ur import URPoseTrajectoryPoint

__all__ = ['DirectUrActionClient']


class DirectUrActionClient(EventEmitterMixin):
    """Direct UR Script action client to simulate an action interface
    on arbitrary URScript commands.
    """
    def __init__(self, ros, timeout=None,
                 omit_feedback=False, omit_status=False, omit_result=False):
        super(DirectUrActionClient, self).__init__()

        self.server_name = '/ur_driver/URScript'
        self.ros = ros
        self.timeout = timeout
        self.omit_feedback = omit_feedback
        self.omit_status = omit_status
        self.omit_result = omit_result
        self.goal = None

        self._received_status = False

        # Advertise the UR Script topic
        self._urscript_topic = Topic(ros, self.server_name, 'std_msgs/String')
        self._urscript_topic.advertise()

        # Create the topics associated with actionlib
        self.status_listener = Topic(ros, '/tool_velocity', 'geometry_msgs/TwistStamped')

        # Subscribe to the status topic
        if not self.omit_status:
            self.status_listener.subscribe(self._on_status_message)

        # If timeout specified, emit a 'timeout' event if the action server does not respond
        if self.timeout:
            self.ros.call_later(self.timeout / 1000., self._trigger_timeout)

        self._internal_state = 'idle'

    def _on_status_message(self, message):
        self._received_status = True

        self.goal.emit('status', message)

        twist = message['twist']
        total_velocity = math.fsum(list(twist['linear'].values()) + list(twist['angular'].values()))
        in_motion = total_velocity != 0.0

        if self._internal_state == 'idle':
            if in_motion:
                self._internal_state = 'executing_goal'
        elif self._internal_state == 'executing_goal':
            if not in_motion:
                self._internal_state = 'stopped'
                self.goal.emit('result', message)

    def _trigger_timeout(self):
        if not self._received_status:
            self.emit('timeout')

    def add_goal(self, goal):
        """Add a goal to this action client.

        Args:
            goal (:class:`.Goal`): Goal to add.
        """
        self.goal = goal

    def cancel(self):
        """Cancel all goals associated with this action client."""
        pass
        # self.cancel_topic.publish(Message())

    def dispose(self):
        """Unsubscribe and unadvertise all topics associated with this action client."""
        # And the UR Script topic
        self._urscript_topic.unadvertise()

        if not self.omit_status:
            self.status_listener.unsubscribe(self._on_status_message)

    def send_goal(self, goal, result_callback=None, timeout=None):
        """Send goal to the action server.

        Args:
            goal (:class:`URGoal`): The goal containing the script lines
            timeout (:obj:`int`): Timeout for the goal's result expressed in milliseconds.
            callback (:obj:`callable`): Function to be called when a result is received. It is a shorthand for hooking on the ``result`` event.
        """
        self._internal_state == 'idle'

        if result_callback:
            goal.on('result', result_callback)

        ur_script = goal.goal_message['goal']['script']
        message = Message({'data': ur_script})
        self._urscript_topic.publish(message)

        if timeout:
            self.ros.call_later(
                timeout / 1000., goal._trigger_timeout)


def direct_ur_movel(ros_client, callback, frames, acceleration=None, velocity=None, time=None, radius=None):

    action_client = DirectUrActionClient(ros_client, timeout=50000)

    script_lines = []
    for frame in frames:
        ptp = URPoseTrajectoryPoint(URPose.from_frame(frame), acceleration, velocity, time, radius)
        move = URMovel(ptp)
        script_lines.append(move)

    urgoal = URGoal(script_lines)

    goal = Goal(action_client, Message(urgoal.msg))
    action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
    # goal.on('feedback', lambda feedback: print(feedback))
    goal.on('result', callback)
    action_client.send_goal(goal)
