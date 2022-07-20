import glob
import os
import shutil
import sys
import numpy as np
import matplotlib.pyplot as plt
import queue
import datetime
import carla
import random


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    # to delete the rgb images folder
    location = "/home/lshi23/research_course_code"
    dir01 = "rgb_out"
    path01 = os.path.join(location, dir01)
    if os.path.exists(path01):
        shutil.rmtree(path01)

    # to delete the depth images folder
    dir02 = "depth_out"
    path02 = os.path.join(location, dir02)
    if os.path.exists(path02):
        shutil.rmtree(path02)
finally:
    pass


def process_img(data, rgb_queue, rgb_freq_message):
    rgb_queue.put(data)
    rgb_freq_message.put(1)


def process_dp(data, dp_queue, dp_freq_message):
    dp_queue.put(data)
    dp_freq_message.put(1)


def process_imu(data):
    angular_vel = data.gyroscope
    acceleration = data.accelerometer
    print(2 * "\n")
    print('angular_vel')
    print(angular_vel)
    print('acceleration')
    print(acceleration)


def process_gnss(data):
    lat = data.latitude
    long = data.longitude
    alt = data.altitude
    print(2 * "\n")
    print('position')
    print(lat)
    print(long)
    print(alt)


def process_lidar(data):
    distance = []
    # np.frombuffer is good while np.array bad, because the data here is a buffer
    i = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    i2 = np.reshape(i, (int(i.shape[0] / 4), 4))
    i3 = i2[:, :3]
    for i in range(i3.shape[0]):
        temp = pow(pow(i3[i][0], 2) + pow(i3[i][1], 2) + pow(i3[i][2], 2), 0.5)
        distance.append(temp)

    nearest_dist = min(distance)
    index = distance.index(nearest_dist)
    print(2 * "\n")
    print('lidar')
    print(i3)
    if nearest_dist < 3:
        print("Obstacle!")
        print(nearest_dist)
        print((index, len(distance)))


try:
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    client.load_world('Town05')
    client.start_recorder("/home/carla/recording01.log")
    IM_WIDTH = 640
    IM_HEIGHT = 480
    DP_IM_WIDTH = 800
    DP_IM_HEIGHT = 600

    # settings = world.get_settings()
    # settings.synchronous_mode = True  # Enables synchronous mode
    # world.apply_settings(settings)

    # It is important to note that the actors we create won't be destroyed
    # unless we call their "destroy" function. If we fail to call "destroy"
    # they will stay in the simulation even after we quit the Python script.
    # For that reason, we are storing all the actors we create so we can
    # destroy them afterwards.
    actor_list = []

    # Get the blueprint library and filter for the vehicle blueprints
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

    # spawn points for vehicles
    spawn_points = world.get_map().get_spawn_points()
    ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    actor_list.append(ego_vehicle)
    print('created ego_%s' % ego_vehicle.type_id)

    # set the spectator
    spectator = world.get_spectator()
    spectator.set_transform(
    carla.Transform(ego_vehicle.get_location() + carla.Location(z=25), carla.Rotation(pitch=-90)))

    for _ in range(0, 10):
        # This time we are using try_spawn_actor. If the spot is already
        # occupied by another object, the function will return None.
        npc = world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
        if npc is not None:
            actor_list.append(npc)
            npc.set_autopilot(True)
            # print('created %s' % npc.type_id)

    ego_vehicle.set_autopilot(True)
    # ego_vehicle.apply_control(carla.VehicleControl(throttle=0.1))

    # Create a transform to place the camera on top of the vehicle
    camera_transform = carla.Transform(carla.Location(x=0.5, z=2.5))
    lidar_transform = carla.Transform(carla.Location(x=1.0, z=1.8))

    # We create sensors through a blueprint that defines its properties
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    camera_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
    camera_bp.set_attribute("fov", "110")
    # camera_bp.set_attribute("sensor_tick", "0.0666")

    camera_dp = world.get_blueprint_library().find('sensor.camera.depth')
    camera_dp.set_attribute("image_size_x", f"{DP_IM_WIDTH}")
    camera_dp.set_attribute("image_size_y", f"{DP_IM_HEIGHT}")

    imu_bp = world.get_blueprint_library().find('sensor.other.imu')
    gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')

    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('dropoff_general_rate', '0.0')
    lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
    lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')

    # We spawn the camera and attach it to our ego vehicle
    camera01 = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
    camera02 = world.spawn_actor(camera_dp, camera_transform, attach_to=ego_vehicle)
    IMU = world.spawn_actor(imu_bp, camera_transform, attach_to=ego_vehicle)
    GNSS = world.spawn_actor(gnss_bp, camera_transform, attach_to=ego_vehicle)
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)

    actor_list.append(camera01)
    actor_list.append(camera02)
    actor_list.append(IMU)
    actor_list.append(GNSS)
    actor_list.append(lidar)

    # print('created %s' % camera01.type_id)
    # print('created %s' % camera02.type_id)
    # print('created %s' % IMU.type_id)
    # print('created %s' % GNSS.type_id)
    # print('created %s' % lidar.type_id)

    # The sensor data will be saved in thread-safe Queues
    rgb_image_queue = queue.Queue(maxsize=1)
    dp_image_queue = queue.Queue(maxsize=1)
    rgb_freq_message = queue.Queue()
    dp_freq_message = queue.Queue()
    count = 0
    freq_count = 0
    start_time = datetime.datetime.now()

    camera01.listen(lambda data: process_img(data, rgb_image_queue, rgb_freq_message))
    camera02.listen(lambda data: process_dp(data, dp_image_queue, dp_freq_message))
    fig, ax = plt.subplots()
    fig_dp, ax_dp = plt.subplots()
    array = np.random.randint(0, 100, size=(IM_HEIGHT, IM_WIDTH), dtype=np.uint8)
    dp_array = np.random.randint(0, 100, size=(DP_IM_HEIGHT, DP_IM_WIDTH), dtype=np.uint8)
    l = ax.imshow(array)
    ax.set_title('RGB')
    l_dp = ax_dp.imshow(dp_array, cmap='gray', interpolation='nearest', vmin=0, vmax=255)
    ax_dp.set_title('Depth')

    while world is not None:
        try:
            # Get the data once it's received.
            image_data = rgb_image_queue.get(True)
            dp_data = dp_image_queue.get(True)
        except queue.Empty:
            print("[Warning] Some sensor data has been missed")
            continue

        # Get the raw BGRA buffer and convert it to an array of RGB of shape (image_data.height, image_data.width, 3).
        im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
        im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
        im_array = im_array[:, :, :3][:, :, ::-1]
        l.set_data(im_array)

        dp_array = np.copy(np.frombuffer(dp_data.raw_data, dtype=np.dtype("uint8")))
        dp_array = np.reshape(dp_array, (dp_data.height, dp_data.width, 4))
        dp_array = dp_array.astype(np.float32) # since it's a float, so to make its range [0,1]
        # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
        normalized_depth = np.dot(dp_array[:, :, :3], [65536.0, 256.0, 1.0])
        normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)
        # Convert to logarithmic depth.
        logdepth = np.ones(normalized_depth.shape) + \
            (np.log(normalized_depth) / 5.70378)
        logdepth = np.clip(logdepth, 0.0, 1.0)
        # Expand to three colors.
        dp_array = np.repeat(logdepth[:, :, np.newaxis], 3, axis=2)
        l_dp.set_data(dp_array)

        plt.pause(0.001)
        count += 1
        if (datetime.datetime.now() - start_time).seconds == 1:
            start_time = datetime.datetime.now()
            frequency = count - freq_count
            freq_count = count
            print('Frequency of rgb_message is: %s' % rgb_freq_message.qsize())
            print('Frequency of dp_message is: %s' % dp_freq_message.qsize())
            print('Frequency of while loop is: %s' % frequency)
            rgb_freq_message = queue.Queue()
            with rgb_freq_message.mutex:
                rgb_freq_message.queue.clear()
            dp_freq_message = queue.Queue()
            with dp_freq_message.mutex:
                dp_freq_message.queue.clear()

        # camera01.listen(lambda image: image.save_to_disk('rgb_out/%06d.png' % image.frame))
        # camera02.listen(lambda data: data.save_to_disk("depth_out/%06d.png" % data.frame, carla.ColorConverter.LogarithmicDepth))
        # IMU.listen(lambda data: process_imu(data))
        # GNSS.listen(lambda data: process_gnss(data))
        # lidar.listen(lambda data: process_lidar(data))

        # time.sleep(10)

        # plt.show()

finally:
    print("Over")