import time

from roslibpy import Message
from roslibpy import Topic

from compas_fab.backends import RosClient

client = RosClient()
listener = Topic(client, '/chatter', 'std_msgs/String')

def receive_message(message):
    print('Heard talking: ' + message['data'])

listener.subscribe(receive_message)

client.run_forever()
