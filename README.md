# Hi-ROS Xsens MTw Wrapper


## Dependencies
* [Xsens MTw Awinda SDK](https://www.xsens.com/products/mtw-awinda)


## Launch files
**hiros\_xsens\_mtw\_wrapper\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice

## Parameters
| Parameter                   | Description                                                                                                                                                                                                     |
|-----------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `xsens_mtw_node_required`   | Set if the other ROS nodes on the PC should be killed when the driver is killed                                                                                                                                 |
| `node_name`                 | Set the name of the ROS node                                                                                                                                                                                    |
| `tf_prefix`                 | Set a prefix to avoid conflicts in the ROS transforms                                                                                                                                                           |
| `desired_update_rate`       | Set the desired update rate                                                                                                                                                                                     |
| `desired_radio_channel`     | Set the desired radio channel                                                                                                                                                                                   |
| `reset_initial_orientation` | Set if the initial orientation should be reset (IMU-like behavior) or not (MIMU behavior)                                                                                                                       |
| `enable_custom_labeling`    | Set if custom labels should be assigned to the trackers. The custom labels can be defined in file [sensor\_labels.yaml](https://github.com/HiROS-unipd/xsens_mtw_wrapper/blob/master/config/sensor_labels.yaml) |
| `synchronize`               | Set if data should be synchronized                                                                                                                                                                              |
| `sync_policy`               | Set the synchronization policy (`fillPartialFrames` or `skipPartialFrames`)                                                                                                                                     |
| `publish_mimu_array`        | Set if a single topic containing all the sensor readings should be published or if a series of topics for each sensor should be published                                                                       |
| `publish_imu`               | Set if the IMU data (accelerometer, magnetometer, and orientation as quaternion) should be published                                                                                                            |
| `publish_mag`               | Set if the magnetometer data should be published                                                                                                                                                                |
| `publish_euler`             | Set if the orientation as Euler angles (roll, pitch, yaw) should be published                                                                                                                                   |
| `publish_free_acceleration` | Set if the free acceleration should be published                                                                                                                                                                |
| `publish_pressure`          | Set if the pressure should be published                                                                                                                                                                         |
| `publish_tf`                | Set if the orientation as ROS transform should be published                                                                                                                                                     |


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

## Using this driver



When using this driver, please cite the following paper:

```latex
A ROS Driver for Xsens Wireless Inertial Measurement Unit Systems.
M. Guidolin, E. Menegatti, M. Reggiani, L. Tagliapietra
In proceedings of 2021 IEEE International Conference on Industrial Technology (ICIT), 2021, in press.
```

Bib citation Source:

```bibtex
@inproceedings{guidolin2021ros,
  title={A {ROS} Driver for Xsens Wireless Inertial Measurement Unit Systems},
  author={Guidolin, Mattia and Menegatti, Emanuele and Reggiani, Monica and Tagliapietra, Luca},
  booktitle={2021 IEEE International Conference on Industrial Technology (ICIT)},
  year={2021, in press},
  organization={IEEE}
}
```

