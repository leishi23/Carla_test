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


def process_imu(data, imu_angular_vel_queue, imu_acceleration_queue):
    imu_angular_vel_queue.put(data.gyroscope)
    imu_acceleration_queue.put(data.accelerometer)


def process_lidar(data, lidar_queue):
    lidar_queue.put(data)


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
    lidar_transform = carla.Transform(carla.Location(x=0.5, z=1.5))

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
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)

    actor_list.append(camera01)
    actor_list.append(camera02)
    actor_list.append(IMU)
    actor_list.append(lidar)

    # print('created %s' % camera01.type_id)
    # print('created %s' % camera02.type_id)
    # print('created %s' % IMU.type_id)
    # print('created %s' % lidar.type_id)

    # The sensor data will be saved in thread-safe Queues
    rgb_image_queue = queue.Queue(maxsize=1)
    dp_image_queue = queue.Queue(maxsize=1)
    rgb_freq_message = queue.Queue()
    dp_freq_message = queue.Queue()
    imu_angular_vel_queue = queue.Queue(maxsize=1)
    imu_acceleration_queue = queue.Queue(maxsize=1)
    location_queue = queue.Queue(maxsize=1)
    vel_queue = queue.Queue(maxsize=1)
    lidar_queue = queue.Queue(maxsize=1)

    count = 0
    freq_count = 0
    start_time = datetime.datetime.now()

    # camera01.listen(lambda data: process_img(data, rgb_image_queue, rgb_freq_message))
    # camera02.listen(lambda data: process_dp(data, dp_image_queue, dp_freq_message))
    # IMU.listen(lambda data: process_imu(data, imu_angular_vel_queue, imu_acceleration_queue))
    lidar.listen(lambda data: process_lidar(data, lidar_queue))

    ############################### initial plot part #############################
    # RGB & Depth camera
    # fig, ax = plt.subplots()
    # fig_dp, ax_dp = plt.subplots()
    # array = np.random.randint(0, 100, size=(IM_HEIGHT, IM_WIDTH), dtype=np.uint8)
    # dp_array = np.random.randint(0, 100, size=(DP_IM_HEIGHT, DP_IM_WIDTH), dtype=np.uint8)
    # l = ax.imshow(array)
    # ax.set_title('RGB')
    # l_dp = ax_dp.imshow(dp_array, cmap='gray', interpolation='nearest', vmin=0, vmax=255)
    # ax_dp.set_title('Depth')

    # IMU acceleration and angular velocity
    # plt.ion()
    # fig_imu, ((ax_wx, ax_wy), (ax_wz, ax_ax), (ax_ay, ax_az)) = plt.subplots(nrows=3, ncols=2)
    # imu_time = 0
    # imu_x = list()
    # wx = list()
    # wy = list()
    # wz = list()
    # ax = list()
    # ay = list()
    # az = list()
    # ax_wx.set_title('angular x-velocity')
    # ax_wy.set_title('angular y-velocity')
    # ax_wz.set_title('angular z-velocity')
    # ax_ax.set_title('x-acceleration')
    # ax_ay.set_title('y-acceleration')
    # ax_az.set_title('z-acceleration')
    # fig_imu.tight_layout()

    # position & velocity
    # plt.ion()
    # fig_get, ((ax_px, ax_py, ax_pz), (ax_vx, ax_vy, ax_vz)) = plt.subplots(nrows=2, ncols=3)
    # get_time = 0
    # get_x = list()
    # px = list()
    # py = list()
    # pz = list()
    # vx = list()
    # vy = list()
    # vz = list()
    # ax_px.set_title('position-x', fontsize=10)
    # ax_py.set_title('position-y', fontsize=10)
    # ax_pz.set_title('position-z', fontsize=10)
    # ax_vx.set_title('x-velocity', fontsize=10)
    # ax_vy.set_title('y-velocity', fontsize=10)
    # ax_vz.set_title('z-velocity', fontsize=10)
    # fig_get.tight_layout()

    # update the plot data
    while world is not None:
        location_queue.put(ego_vehicle.get_location())
        vel_queue.put(ego_vehicle.get_velocity())
        try:
            # Get the data once it's received.
            # image_data = rgb_image_queue.get(True)
            # dp_data = dp_image_queue.get(True)
            # angular_vel_data = imu_angular_vel_queue.get(True)
            # acceleration_data = imu_acceleration_queue.get(True)
            location_data = location_queue.get(True)
            vel_data = vel_queue.get(True)
            lidar_data = lidar_queue.get(True)
        except queue.Empty:
            print("[Warning] Some sensor data has been missed")
            continue

        # # RGB:Get the raw BGRA buffer and convert it to an array of RGB as shape (image_data.height, image_data.width, 3).
        # im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
        # im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
        # im_array = im_array[:, :, :3][:, :, ::-1]
        # l.set_data(im_array)
        #
        # # Depth: use my own ColorConverter
        # dp_array = np.copy(np.frombuffer(dp_data.raw_data, dtype=np.dtype("uint8")))
        # dp_array = np.reshape(dp_array, (dp_data.height, dp_data.width, 4))
        # dp_array = dp_array.astype(np.float32) # since it's a float, so to make its range [0,1]
        # # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
        # normalized_depth = np.dot(dp_array[:, :, :3], [65536.0, 256.0, 1.0])
        # normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)
        # # Convert to logarithmic depth.
        # logdepth = np.ones(normalized_depth.shape) + \
        #     (np.log(normalized_depth) / 5.70378)
        # logdepth = np.clip(logdepth, 0.0, 1.0)
        # # Expand to three colors.
        # dp_array = np.repeat(logdepth[:, :, np.newaxis], 3, axis=2)
        # l_dp.set_data(dp_array)

        # IMU: angular vel & acceleration
        # imu_time += 1
        # angular_vel_x_array = np.array(angular_vel_data.x)
        # angular_vel_y_array = np.array(angular_vel_data.y)
        # angular_vel_z_array = np.array(angular_vel_data.z)
        # acceleration_x_array = np.array(acceleration_data.x)
        # acceleration_y_array = np.array(acceleration_data.y)
        # acceleration_z_array = np.array(acceleration_data.z)
        # imu_x.append(imu_time)
        # wx.append(angular_vel_x_array)
        # wy.append(angular_vel_y_array)
        # wz.append(angular_vel_z_array)
        # ax.append(acceleration_x_array)
        # ay.append(acceleration_y_array)
        # az.append(acceleration_z_array)
        # if imu_time > 6:
        #     ax_wx.set_xlim(imu_time-5, imu_time)
        #     ax_wy.set_xlim(imu_time-5, imu_time)
        #     ax_wz.set_xlim(imu_time-5, imu_time)
        #     ax_ax.set_xlim(imu_time-5, imu_time)
        #     ax_ay.set_xlim(imu_time-5, imu_time)
        #     ax_az.set_xlim(imu_time-5, imu_time)
        # ax_wx.scatter(imu_time, angular_vel_x_array)
        # ax_wy.scatter(imu_time, angular_vel_y_array)
        # ax_wz.scatter(imu_time, angular_vel_z_array)
        # ax_ax.scatter(imu_time, acceleration_x_array)
        # ax_ay.scatter(imu_time, acceleration_y_array)
        # ax_az.scatter(imu_time, acceleration_z_array)
        # # display(fig_imu)
        # # clear_output(wait=True)
        # plt.show()

        # position & velocity
        # get_time += 1
        # position_x_array = np.array(location_data.x)
        # position_y_array = np.array(location_data.y)
        # position_z_array = np.array(location_data.z)
        # vel_x_array = np.array(vel_data.x)
        # vel_y_array = np.array(vel_data.y)
        # vel_z_array = np.array(vel_data.z)
        # get_x.append(get_time)
        # px.append(position_x_array)
        # py.append(position_y_array)
        # pz.append(position_z_array)
        # vx.append(vel_x_array)
        # vy.append(vel_y_array)
        # vz.append(vel_z_array)
        # if get_time > 6:
        #     ax_px.set_xlim(get_time-5, get_time)
        #     ax_py.set_xlim(get_time-5, get_time)
        #     ax_pz.set_xlim(get_time-5, get_time)
        #     ax_vx.set_xlim(get_time-5, get_time)
        #     ax_vy.set_xlim(get_time-5, get_time)
        #     ax_vz.set_xlim(get_time-5, get_time)
        # ax_px.scatter(get_time, position_x_array)
        # ax_py.scatter(get_time, position_y_array)
        # ax_pz.scatter(get_time, position_z_array)
        # ax_vx.scatter(get_time, vel_x_array)
        # ax_vy.scatter(get_time, vel_y_array)
        # ax_vz.scatter(get_time, vel_z_array)
        # plt.show()

        # lidar
        distance = []
        p_cloud_size = len(lidar_data)
        p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
        p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))
        angle = lidar_data.horizontal_angle

        # Point cloud in lidar sensor space array of shape (p_cloud_size, 3).
        local_lidar_points = np.array(p_cloud[:, :3])

        for i in range(p_cloud_size):
            temp = pow(pow(local_lidar_points[i][0], 2) + pow(local_lidar_points[i][1], 2)
                       + pow(local_lidar_points[i][2], 2), 0.5)
            distance.append(temp)

        nearest_dist = min(distance)
        if nearest_dist < 3:
            print("Obstacle!")
            print(nearest_dist)
            print(angle)
            print("\n")

        plt.pause(0.1)
        # count += 1
        # if (datetime.datetime.now() - start_time).seconds == 1:
        #     start_time = datetime.datetime.now()
        #     frequency = count - freq_count
        #     freq_count = count
        #     print('Frequency of rgb_message is: %s' % rgb_freq_message.qsize())
        #     print('Frequency of dp_message is: %s' % dp_freq_message.qsize())
        #     print('Frequency of while loop is: %s' % frequency)
        #     rgb_freq_message = queue.Queue()
        #     with rgb_freq_message.mutex:
        #         rgb_freq_message.queue.clear()
        #     dp_freq_message = queue.Queue()
        #     with dp_freq_message.mutex:
        #         dp_freq_message.queue.clear()

        # set the spectator
        spectator = world.get_spectator()
        spectator.set_transform(
            carla.Transform(ego_vehicle.get_location() + carla.Location(z=25), carla.Rotation(pitch=-90)))

finally:
    print("Over")