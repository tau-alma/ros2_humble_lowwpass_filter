# lowpass_filter

A configurable ROS 2 (Humble) C++ package for lowpass-filtering sensor and
state topics. Filters are IIR (bilinear-transformed analog design), support
1st or 2nd order, optional differentiation (for estimating velocity/
acceleration from a position-like signal), and optional cutoff prewarping.

Any number of filter nodes can be launched at once, each independently
configured via a YAML file — no code changes needed to add a new sensor,
change a cutoff frequency, or filter a different topic.

## Supported topic types

| `topic_type`          | Message type                        | Filter keys |
|------------------------|--------------------------------------|-------------|
| `joint_state`           | `sensor_msgs/msg/JointState`         | `raw_encoder` mode: `ang`, `vel`, `acc` (fixed) — `full_state` mode: `position`, `velocity`, `effort` (fixed) |
| `odometry`               | `nav_msgs/msg/Odometry`              | any subset of `vx`, `vy`, `vz`, `wx`, `wy`, `wz` |
| `float64`                 | `std_msgs/msg/Float64`               | `value` (fixed) |
| `float64_multi_array`      | `std_msgs/msg/Float64MultiArray`     | one key per array element, named `"0"`, `"1"`, `"2"`, ... |
| `fluid_pressure`           | `sensor_msgs/msg/FluidPressure`      | `fluid_pressure` (fixed) — `variance` is passed through unfiltered |
| `magnetic_field`           | `sensor_msgs/msg/MagneticField`      | any subset of `x`, `y`, `z` — `magnetic_field_covariance` is passed through unfiltered | 
|`imu`  | `sensor_msgs/msg/Imu`     |	any subset of `ax`, `ay`, `az` (linear acceleration), `wx`, `wy`, `wz` (angular velocity) — orientation and all covariances are passed through unfiltered |

Each filter node subscribes to `<topic_name>` and publishes the filtered
result to `<topic_name>_filt`, preserving message fields it isn't
configured to filter.

### JointState modes

- **`raw_encoder`** — the incoming message only carries a raw sensor
  reading in `position[0]`. The node converts it to an angle via a
  `sensor` calibration (`center_link`, `boom`, `bucket`, or `none`), then
  cascades the filtered angle through a derivator filter for velocity and
  a second derivator filter (fed by the filtered velocity) for
  acceleration.
- **`full_state`** — the incoming message already has `position`,
  `velocity`, and `effort` populated. Each is filtered independently, with
  no cascading.

## Configuration

Filter nodes are defined in a YAML file. Each top-level key is one node
instance. See [`configs/config_template.yaml`](configs/config_template.yaml)
for a fully worked example of every topic type.

```yaml
pressure_sensor:
  node_name: pressure_filter
  topic_name: /pressure
  topic_type: float64
  filters:
    value:
      freq: 100.0       # sample rate (Hz), required
      cutoff: 8.0        # cutoff frequency (Hz), required
      zeta: 0.7071       # damping ratio, order-2 only (default 1/sqrt(2))
      order: 1           # 1 or 2 (default 1)
      derivator: false   # differentiate instead of just lowpass (default false)
      prewarp: false      # prewarp the cutoff before the bilinear transform (default false)
```

`node_name` and `topic_name` are required for every entry. `sensor` and
`mode` are only read when `topic_type: joint_state`.

## Building

```bash
cd ros_ws
colcon build --packages-select lowpass_filter
source install/setup.bash
```

Dependencies (see `package.xml`): `rclcpp`, `sensor_msgs`, `nav_msgs`,
`std_msgs`, `yaml-cpp`, plus `ament_index_python`, `launch`, `launch_ros`
for the launch file.

## Running

Via the launch file (recommended — resolves config paths through the
package share directory automatically):

```bash
ros2 launch lowpass_filter lowpass_filter.launch.py
# or, with a specific config file (looked up inside this package's configs/):
ros2 launch lowpass_filter lowpass_filter.launch.py config:=my_config.yaml
```

To use a config file that lives outside this package (e.g. in a
downstream project that includes this package as a submodule), pass a
full path as the `config` launch argument instead — see
[Using this package from another project](#using-this-package-from-another-project).

Directly via the executable (mainly useful for debugging):

```bash
ros2 run lowpass_filter main /full/path/to/config.yaml
```

## Using this package from another project

This package is designed to be added as a git submodule and reused
as-is, with project-specific configuration kept in the *downstream*
project rather than modified here.

```bash
cd <downstream_project>/src
git submodule add <this-repo-url> lowpass_filter
```

In the downstream project, keep your own config YAML in your own package
(e.g. `your_package/config/your_config.yaml`), and include this
package's launch file from your own launch file, passing a full,
resolved path:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('your_package'), 'config', 'your_config.yaml'
    )

    lowpass_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lowpass_filter'), 'launch', 'lowpass_filter.launch.py')
        ]),
        launch_arguments={'config': config_path}.items()
    )

    return LaunchDescription([lowpass_filter_launch])
```

This keeps the submodule itself generic and reusable — bug fixes and new
topic-type support land here once and are picked up by every project
using it, without any project-specific configuration living inside this
repo.

