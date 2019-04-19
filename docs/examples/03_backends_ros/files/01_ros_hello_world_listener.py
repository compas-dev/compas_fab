import time

from roslibpy import Message
from roslibpy import Topic

from compas_fab.backends import RosClient

client = RosClient()
listener = Topic(client, '/chatter', 'std_msgs/String')


def start_listening():
    listener.subscribe(receive_message)

def receive_message(message):
    print('Heard talking: ' + message['data'])

client.on_ready(start_listening)
client.run_forever()
