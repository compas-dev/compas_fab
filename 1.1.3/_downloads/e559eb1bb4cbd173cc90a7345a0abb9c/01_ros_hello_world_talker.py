import time

from roslibpy import Message
from roslibpy import Topic

from compas_fab.backends import RosClient

with RosClient() as client:
    talker = Topic(client, '/chatter', 'std_msgs/String')

    while client.is_connected:
        talker.publish(Message({'data': 'Hello World!'}))
        print('Sending message...')
        time.sleep(1)

    talker.unadvertise()
