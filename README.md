# hiros_xsens_mtw_wrapper


## Dependencies
* [Xsens MTw Awinda SDK](https://www.xsens.com/products/mtw-awinda)


## Launch files
**hiros\_xsens\_mtw\_wrapper\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


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

```
A ROS Driver for Xsens Wireless Inertial Measurement Unit Systems.
M. Guidolin, E. Menegatti, M. Reggiani, L. Tagliapietra
In proceedings of 2021 IEEE International Conference on Industrial Technology (ICIT), 2021, to be published.
```

Bib citation source:
```
@inproceedings{guidolin2021ros,
  title={A {ROS} Driver for Xsens Wireless Inertial Measurement Unit Systems},
  author={Guidolin, Mattia and Menegatti, Emanuele and Reggiani, Monica and Tagliapietra, Luca},
  booktitle={2021 IEEE International Conference on Industrial Technology (ICIT)},
  year={2021, to be published},
  organization={IEEE}
}
```
