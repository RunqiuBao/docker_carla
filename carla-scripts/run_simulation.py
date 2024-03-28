import carla
import random
import os
import subprocess
try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        tsAll = [x.frame for x in data]
        assert all(x == tsAll[0] for x in tsAll)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)
        print('destroying sensors.')
        for sensor in self.sensors:
            sensor.destroy()

    def _retrieve_data(self, sensor_queue, timeout):
        data = sensor_queue.get(timeout=timeout)
        return data


def ConfigRgbCamera(cameraBlurPrint):
    cameraBlurPrint.set_attribute('image_size_x', '640')
    cameraBlurPrint.set_attribute('image_size_y', '360')
    # cameraBlurPrint.set_attribute('fov', '110')
    # Set the time in seconds between sensor captures
    # cameraBlurPrint.set_attribute('sensor_tick', '0.02')
    return cameraBlurPrint


def CreateDir(dir):
    os.makedirs(dir, exist_ok=True)
    return dir


def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    sensor_list = []

    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('*vehicle*')
    # Get the map's spawn points
    spawn_points = world.get_map().get_spawn_points()
    ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    ego_vehicle.set_autopilot(True)

    # populate the world with some vehicles and pedestrians
    process = subprocess.Popen(["python3", "generate_traffic.py", "-n", "100", "-w", "100"])

    # set the cars to autopilot
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.set_autopilot(True)
    # set all the traffic lights to green
    for traffic_light in world.get_actors().filter('*traffic_light*'):
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)

    # We create the camera through a blueprint that defines its properties
    camera_rgb_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.rgb')),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    camera_rgb_r = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.rgb')),
        carla.Transform(carla.Location(x=2.5, y=0.4, z=2)),
        attach_to=ego_vehicle)
    camera_insseg_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.instance_segmentation')),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    camera_depth_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.depth')),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    sensor_list.append(camera_rgb_l)
    sensor_list.append(camera_rgb_r)
    sensor_list.append(camera_insseg_l)
    sensor_list.append(camera_depth_l)

    outputPath= "/home/carla/dataOutput/"
    dataOutputPath_rgb_l = CreateDir(os.path.join(outputPath, 'cam_rgb_l'))
    dataOutputPath_rgb_r = CreateDir(os.path.join(outputPath, 'cam_rgb_r'))
    dataOutputPath_depth_l = CreateDir(os.path.join(outputPath, 'cam_depth_l'))
    dataOutputPath_insseg_l = CreateDir(os.path.join(outputPath, 'cam_insseg_l'))

    indexFrame = 0
    with CarlaSyncMode(world, camera_rgb_l, camera_rgb_r, camera_insseg_l, camera_depth_l, fps=60) as sync_mode:
        while True:
            camera_rgb_l, camera_rgb_r, camera_insseg_l, camera_depth_l = sync_mode.tick(timeout=2.0)
            camera_rgb_l.save_to_disk(os.path.join(dataOutputPath_rgb_l, '%06d.png' % indexFrame))
            camera_rgb_r.save_to_disk(os.path.join(dataOutputPath_rgb_r, '%06d.png' % indexFrame))            
            camera_insseg_l.save_to_disk(os.path.join(dataOutputPath_insseg_l, '%06d.png' % indexFrame), carla.ColorConverter.CityScapesPalette)
            camera_depth_l.save_to_disk(os.path.join(dataOutputPath_depth_l, '%06d.png' % indexFrame), carla.ColorConverter.Depth)
            indexFrame += 1
            print("generated frame %d" % indexFrame)


if __name__ == '__main__':
    main()