from compas_fab.backends import RosClient

with RosClient() as client:
    print("Connected: ", client.is_connected)
