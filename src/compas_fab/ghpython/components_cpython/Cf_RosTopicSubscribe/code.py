# r: compas_fab>=1.0.2
"""
Subscribe to a ROS topic.

COMPAS FAB v1.0.2
"""

import time

import Grasshopper

from roslibpy import Topic
from scriptcontext import sticky as st

from compas_fab.backends.ros.messages import ROSmsg
from compas_fab.ghpython.components import create_id
from compas_fab.ghpython.components import message


class ROSTopicSubscribe(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, ros_client, topic_name: str, topic_type: str, interval: int, start: bool, stop: bool):
        if not topic_name:
            raise ValueError("Please specify the name of the topic")
        if not topic_type:
            raise ValueError("Please specify the type of the topic")

        if not hasattr(self, "message_count"):
            self.message_count = 0

        self.interval = interval or 25  # in milliseconds
        self.is_updating = False
        self.is_subscribed = False

        self.msg_key = create_id(ghenv.Component, "last_msg")  # noqa: F821
        key = create_id(ghenv.Component, "topic")  # noqa: F821

        last_msg = st.get(self.msg_key, None)
        topic = st.get(key, None)

        if ros_client and ros_client.is_connected:
            if start:
                self._unsubscribe(topic)

                topic = Topic(ros_client, topic_name, topic_type)
                topic.subscribe(self.topic_callback)
                time.sleep(0.2)

                st[key] = topic

            if stop:
                self._unsubscribe(topic)

        self.is_subscribed = topic and topic.is_subscribed

        if last_msg:
            last_msg = ROSmsg.parse(last_msg, topic_type)

        if self.is_subscribed:
            message(ghenv.Component, f"Subscribed, {self.message_count} messages")  # noqa: F821
        else:
            message(ghenv.Component, "Not subscribed")  # noqa: F821

        return (topic, last_msg, self.is_subscribed)

    def _unsubscribe(self, topic):
        if topic and topic.is_subscribed:
            topic.unsubscribe()
            time.sleep(0.2)

    def topic_callback(self, msg):
        self.message_count += 1
        st[self.msg_key] = msg

        if self.is_subscribed:
            ghdoc = ghenv.Component.OnPingDocument()  # noqa: F821 This is defined by Grasshopper
            if not self.is_updating and ghdoc.SolutionState != Grasshopper.Kernel.GH_ProcessStep.Process:
                self.is_updating = True
                ghdoc.ScheduleSolution(self.interval, Grasshopper.Kernel.GH_Document.GH_ScheduleDelegate(self.expire_callback))

    def expire_callback(self, ghdoc):
        if ghdoc.SolutionState != Grasshopper.Kernel.GH_ProcessStep.Process:
            ghenv.Component.ExpireSolution(False)  # noqa: F821 This is defined by Grasshopper
        self.is_updating = False
