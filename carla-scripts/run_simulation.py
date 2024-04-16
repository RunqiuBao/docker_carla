import carla
import random
import os
import numpy
import cv2
import subprocess
import argparse
try:
    import queue
except ImportError:
    import Queue as queue


class CameraConfig:
    width = 640
    height = 360
    fov = 110
    kk = None

    def __init__(self):
        self.kk = self.ComputeKKFromFov(self.fov, self.width, self.height)

    @staticmethod
    def ComputeKKFromFov(fov, width, height):
        """
        fov is in degrees, the apex angle of view cone.
        """
        focalLength = width / (2 * numpy.tan(fov * numpy.pi / 360))
        cx = width / 2
        cy = height / 2
        return numpy.array([
            [focalLength, 0, cx],
            [0, focalLength, cy],
            [0, 0, 1]
        ])


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
        self.numSkipInBeginnings = kwargs.get('numSkipInBeginnings', 0)

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
        if self.numSkipInBeginnings > 0:
            for indexTick in range(self.numSkipInBeginnings):
                self.world.tick()
            self.numSkipInBeginnings = 0    
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        tsAll = [x.frame for x in data]
        print("tsAll: ", tsAll)
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


def ConfigRgbCamera(cameraBlurPrint, cameraConfig):
    cameraBlurPrint.set_attribute('image_size_x', str(cameraConfig.width))
    cameraBlurPrint.set_attribute('image_size_y', str(cameraConfig.height))
    cameraBlurPrint.set_attribute('fov', str(cameraConfig.fov))
    # Set the time in seconds between sensor captures
    # cameraBlurPrint.set_attribute('sensor_tick', '0.02')
    return cameraBlurPrint


def CreateDir(dir):
    os.makedirs(dir, exist_ok=True)
    return dir


def convert_bgra_to_array(image):
    """Convert a CARLA raw image to a BGRA numpy array."""
    if not isinstance(image, carla.Image):
        raise ValueError("Argument must be a carla.sensor.Image")
    array = numpy.frombuffer(image.raw_data, dtype=numpy.dtype("uint8"))
    array = numpy.reshape(array, (image.height, image.width, 4))
    return array


def convert_to_rgb_array(image):
    """Convert a CARLA raw image to a RGB numpy array."""
    array = convert_bgra_to_array(image)
    # Convert BGRA to RGB.
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array


def convert_depth_to_disparity_array(depthImage, camConfig, stereoBaseline):
    """
    Convert a Carla raw depth image to a disparity numpy array.
    """
    depthImage_np = convert_to_rgb_array(depthImage).astype(numpy.float32)
    depthImage_np = ((depthImage_np[:,:,0] + depthImage_np[:,:,1] * 256.0 + depthImage_np[:,:,2] * 256.0 * 256.0) / ((256.0 * 256.0 * 256.0) - 1))
    depthImage_np_meters = 1000 * depthImage_np
    disparity = stereoBaseline * camConfig.kk[0, 0] / depthImage_np_meters
    return disparity


def main(args):
    timeOutValue = 10.0

    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    client.set_timeout(timeOutValue)
    print("Available maps: \n{}".format(client.get_available_maps()))
    print("Using map: {}".format(args.worldMap))
    
    if args.worldMap is None:
        world = client.get_world()
    else:
        world = client.load_world(args.worldMap)
    sensor_list = []

    # config weather
    weatherNumber = random.randint(0, 1)
    world.set_weather(carla.WeatherParameters.CloudyNoon if weatherNumber == 0 else carla.WeatherParameters.ClearNoon)
    print("weather: \n{}".format(world.get_weather()))

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
    cameraConfig = CameraConfig()
    camera_rgb_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.rgb'), cameraConfig),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    camera_rgb_r = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.rgb'), cameraConfig),
        carla.Transform(carla.Location(x=2.5, y=0.4, z=2)),
        attach_to=ego_vehicle)
    camera_insseg_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.instance_segmentation'), cameraConfig),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    camera_depth_l = world.spawn_actor(
        ConfigRgbCamera(blueprint_library.find('sensor.camera.depth'), cameraConfig),
        carla.Transform(carla.Location(x=2.5, y=-0.4, z=2)),
        attach_to=ego_vehicle)
    sensor_list.append(camera_rgb_l)
    sensor_list.append(camera_rgb_r)
    sensor_list.append(camera_insseg_l)
    sensor_list.append(camera_depth_l)

    outputPath= "/home/carla/dataOutput/"
    dataOutputPath_rgb_l = CreateDir(os.path.join(outputPath, 'cam_rgb_l'))
    dataOutputPath_rgb_r = CreateDir(os.path.join(outputPath, 'cam_rgb_r'))
    dataOutputPath_depth_l = CreateDir(os.path.join(outputPath, 'disparity', 'event'))
    tsFilePath = os.path.join(outputPath, 'disparity', 'timestamps.txt')
    dataOutputPath_insseg_l = CreateDir(os.path.join(outputPath, 'cam_insseg_l'))

    indexFrame = 0
    rgbSensorFps = 1000
    slowDownSamplingRate = 50
    with CarlaSyncMode(world, camera_rgb_l, camera_rgb_r, camera_insseg_l, camera_depth_l, fps=rgbSensorFps, numSkipInBeginnings=5) as sync_mode:
        while True:
            camera_rgb_l, camera_rgb_r, camera_insseg_l, camera_depth_l = sync_mode.tick(timeout=timeOutValue)
            camera_rgb_l.save_to_disk(os.path.join(dataOutputPath_rgb_l, '%06d.png' % indexFrame))
            camera_rgb_r.save_to_disk(os.path.join(dataOutputPath_rgb_r, '%06d.png' % indexFrame))
            if indexFrame % slowDownSamplingRate == 0:
                camera_depth_l.convert(carla.ColorConverter.Depth)
                disparity = convert_depth_to_disparity_array(camera_depth_l, cameraConfig, stereoBaseline=0.8)
                cv2.imwrite(os.path.join(dataOutputPath_depth_l, '%06d.png' % (indexFrame // slowDownSamplingRate)), numpy.round(disparity).astype('uint8'))
                insseg_l = convert_to_rgb_array(camera_insseg_l)
                cv2.imwrite(os.path.join(dataOutputPath_insseg_l, '%06d.png' % (indexFrame // slowDownSamplingRate)), insseg_l)
                writeMode = 'a' if os.path.exists(tsFilePath) else 'w'
                with open(tsFilePath, writeMode) as file:
                    file.write(str(int(indexFrame * 10e6 / rgbSensorFps)) + "\n")  # Note: in microseconds
            indexFrame += 1
            print("generated frame %d" % indexFrame)
            if indexFrame >= 10000:
                break


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description="run simulation to generate data for training")
    argparser.add_argument(
        '--worldMap',
        metavar='m',
        default=None,
        help='Define the world map to use.')
    args = argparser.parse_args()

    main(args)