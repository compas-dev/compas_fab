import time

from roslibpy import Message
from roslibpy import Topic

from compas_fab.backends import RosClient

client = RosClient()
talker = Topic(client, '/chatter', 'std_msgs/String')


def start_talking():
    while client.is_connected:
        talker.publish(Message({'data': 'Hello RobArch World!'}))
        print('Sending message...')
        time.sleep(1)

    talker.unadvertise()


client.on_ready(start_talking)
client.run_forever()
