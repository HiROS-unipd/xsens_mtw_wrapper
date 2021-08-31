# Hi-ROS Xsens MTw Wrapper


## Dependencies
* [Xsens MTw Awinda SDK](https://www.xsens.com/products/mtw-awinda)


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice

## Parameters
| Parameter                   | Description                                                  |
| --------------------------- | ------------------------------------------------------------ |
| `node_required`             | Set if the other ROS nodes on the PC should be killed when the driver is killed |
| `node_name`                 | Set the name of the ROS node                                 |
| `tf_prefix`                 | Set a prefix to avoid conflicts in the ROS transforms        |
| `number_of_mtws`            | Set the number of MTw trackers that will be connected to the master |
| `desired_update_rate`       | Set the desired update rate                                  |
| `desired_radio_channel`     | Set the desired radio channel                                |
| `fixed_latency`             | Set a fixed communication latency (in seconds)               |
| `reset_initial_orientation` | Set if the initial orientation should be reset (IMU-like behavior) or not (MIMU behavior) |
| `enable_custom_labeling`    | Set if custom labels should be assigned to the trackers. The custom labels can be defined in file [sensor\_labels.yaml](https://github.com/HiROS-unipd/xsens_mtw_wrapper/blob/master/config/sensor_labels.yaml) |
| `enable_external_sync`      | Set if the Awinda station's synchronization ports are being used |
| `publish_only_recording`    | Set if messages should be published only while in <em>Recording</em> state |
| `synchronize`               | Set if data among different trackers should be synchronized  |
| `sync_policy`               | Set the synchronization policy (`fillPartialFrames` or `skipPartialFrames`) |
| `publish_mimu_array`        | Set if a single topic containing all the sensor readings should be published or if a series of topics for each sensor should be published |
| `publish_imu`               | Set if the IMU data (accelerometer, gyroscope, and orientation as quaternion) should be published |
| `publish_mag`               | Set if the magnetometer data should be published             |
| `publish_euler`             | Set if the orientation as Euler angles (roll, pitch, yaw) should be published |
| `publish_free_acceleration` | Set if the free acceleration should be published             |
| `publish_pressure`          | Set if the pressure should be published                      |
| `publish_tf`                | Set if the orientation as ROS transform should be published  |

## Published topics
- **/mimu/data**: array of MIMU messages (where each MIMU message contains a *sensor_msgs/Imu* and a *sensor_msgs/MagneticField*)
- **/imu/data**: orientation, angular velocity, linear acceleration
- **/imu/mag**: magnetic field
- **/imu/euler**: orientation expressed as Euler angles (roll, pitch, yaw)
- **/filter/free_acceleration**: free acceleration
- **/pressure**: pressure
- **/tf**: orientation of each IMU expressed as tf

## Usage
```
roslaunch hiros_xsens_mtw_wrapper custom_configuration_example.launch
```

## Perform online reset of IMU(s) orientation
To reset the orientation of IMUs separately by label <imu_0, imu_1>:
```
rosservice call /xsens_mtw/reset_orientation "sensors: ['imu_0','imu_1']"
```
or by ID:
```
rosservice call /xsens_mtw/reset_orientation "sensors: ['00B44BFF','00B44C00']"
```

To reset the orientation of all IMUs:
```
rosservice call /xsens_mtw/reset_orientation "sensors: []"
```

## Start/stop recording
To start recording:
```
rosservice call /xsens_mtw/start_recording "{}"
```

To stop recording:
```
rosservice call /xsens_mtw/stop_recording "{}"
```

## Using this driver

When using this driver, please cite the following paper:

```latex
Guidolin, Mattia, et al. "A ROS driver for Xsens wireless inertial measurement unit systems." 2021 22nd IEEE International Conference on Industrial Technology (ICIT). Vol. 1. IEEE, 2021.
```

Bib citation Source:

```bibtex
@inproceedings{guidolin2021ros,
  title={A ROS driver for Xsens wireless inertial measurement unit systems},
  author={Guidolin, Mattia and Menegatti, Emanuele and Reggiani, Monica and Tagliapietra, Luca},
  booktitle={2021 22nd IEEE International Conference on Industrial Technology (ICIT)},
  volume={1},
  pages={677--683},
  year={2021},
  organization={IEEE}
}
```

