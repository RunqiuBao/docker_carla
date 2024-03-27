import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
print("available maps:\n{}".format(client.get_available_maps()))
