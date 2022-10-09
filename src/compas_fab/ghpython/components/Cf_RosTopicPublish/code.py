"""
Publishes messages to a ROS topic

COMPAS FAB v0.26.0
"""
import time

from ghpythonlib.componentbase import executingcomponent as component
from roslibpy import Topic
from scriptcontext import sticky as st

from compas_fab.backends.ros.messages import ROSmsg
from compas_fab.ghpython.components import create_id


class ROSTopicPublish(component):
    def RunScript(self, ros_client, topic_name, topic_type, msg):
        if not topic_name:
            raise ValueError("Please specify the name of the topic")
        if not topic_type:
            raise ValueError("Please specify the type of the topic")

        key = create_id(self, "topic")

        topic = st.get(key, None)

        if ros_client and ros_client.is_connected:
            if not topic or topic.ros != ros_client:
                topic = Topic(ros_client, topic_name, topic_type, reconnect_on_close=False)
                topic.advertise()
                time.sleep(0.2)

                st[key] = topic

        self.is_advertised = topic and topic.is_advertised

        if msg:
            msg = ROSmsg.parse(msg, topic_type)
            topic.publish(msg.msg)
            self.Message = "Message published"
        else:
            if self.is_advertised:
                self.Message = "Topic advertised"

        return (topic, self.is_advertised)
