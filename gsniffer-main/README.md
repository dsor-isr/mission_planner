# gSniffer

ROS wrapper for a gamma radiation CZT microspectrometer, also known as gamma sniffer.

[Install MCA_COMM library](https://github.com/ramones-eu/mca_comm)

## Setup workspace

Create a workspace:
```bash
mkdir -p ~/gsniffer_ws/src
cd ~/gsniffer_ws/src
```

If you are not planning to develop code, you can download and extract the latest releases from
[here](https://github.com/ramones-eu/mca_comm/releases) and [here](https://github.com/ramones-eu/gsniffer/releases),
e.g.:
```bash
wget https://github.com/ramones-eu/mca_comm/archive/refs/tags/1.2.0.tar.gz
wget https://github.com/ramones-eu/gsniffer/archive/refs/tags/0.0.1.tar.gz
tar xzf mca_comm-1.2.0.tar.gz
tar xzf gsniffer-0.0.1.tar.gz
```

:warning: If `wget` fails, you would have to download it from a browser after login into github.

If you are planning to develop code, then checkout the repository with the code:
```bash
git clone git@github.com:ramones-eu/mca_comm.git
git clone git@github.com:ramones-eu/gsniffer.git
```

:information_source: In order to use `git clone` you need to generate and add a new SSH key following the instructions
[here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
if you have not done it before.

Follow the instructions in https://github.com/ramones-eu/mca_comm/blob/main/README.md in order to install the MCA COMM
library dependencies and setup the system requirements:
* https://github.com/ramones-eu/mca_comm/blob/main/README.md#install-ftd2xx-library
* https://github.com/ramones-eu/mca_comm/blob/main/README.md#deploy-udev-rules

## Install dependencies

When we added support to save `.spe` files, we added a dependency on Boost asio, so we have to install it with:
```bash
sudo apt install -y libboost-dev
```

## Build workspace

```bash
cd ~/gsniffer_ws
catkin build
```

## Test device out

:information_source: The `save_folder` parameter is set to `"/media/data/gsniffer"` in the `config/gsniffer.yaml` file.
The output `.spe` files would be saved in that folder every time a new spectrum sample is acquired.

:warning: Make sure you have write permissions into that folder.

### Run `gsniffer` launch

Run the `gsniffer` launch file:
```bash
source devel/setup.bash
roslaunch gsniffer gsniffer.launch
```

This will run the `gsniffer` node with a good default configuration in `config/gsniffer.yaml`.

### Run `gsniffer` node

The recommended way to run the `gsniffer` node is the launch file, because it also loads a good default configuration
and you only need one instruction. That being said, you can also run the node directly.

Run a `roscore`:
```bash
roscore
```

From another termainal, run the `gsniffer` ROS node:
```bash
source devel/setup.bash
rosrun gsniffer gsniffer
```

### Echo the `gsniffer` topics

From another terminal we can see the messages published:
```bash
$ source devel/setup.bash
$ rostopic echo -n1 /gsniffer/state
header: 
  seq: 1
  stamp: 
    secs: 1682802178
    nsecs: 129513279
  frame_id: ''
device_type: 5
hardware_version: 5120
firmware_version: 8705
serial_number: 3160
preset: 0
preset_time: 100
threshold: 0.5
lld: 0
uld: 1023
coarse_gain: 10
fine_gain: 11252.0
cps: 239
detector_bias: 1500
pzc: 1874
---
$ rostopic echo -n1 /gsniffer/system_data
header:
  seq: 0
  stamp:
    secs: 1674588328
    nsecs: 791931457
  frame_id: ''
state: 1
---
$ rostopic echo -n1 /gsniffer/power
header:
  seq: 0
  stamp:
    sec: 1663777965
    nanosec: 890454102
  frame_id: ''
high_voltage: 1536.0
---
$ rostopic echo -n1 /gsniffer/calibration
slope: 1.0
offset: 0.0
square: 0.0
---
$ rostopic echo --noarr -n1 /gsniffer/spectrum
header: 
  seq: 4
  stamp: 
    secs: 1682802292
    nsecs: 742696990
  frame_id: ''
first_channel: 0
last_channel: 1023
channels: "<array type: uint32, length: 1024>"
life_time: 
  secs: 14
  nsecs:         0
real_time: 
  secs: 14
  nsecs:         0
flat_top_time: 
  secs: 0
  nsecs:      1200
high_voltage: 1541.0
counts: 1554
peak_detector_counts: 0
dead_time: 
  secs: 0
  nsecs:  15000000
busy_time: 
  secs: 0
  nsecs:         0
detector_temperature: -256.0
mca_temperature: 32.0
---
```
:point_right: For the `/spectrum` you might have to wait up to `15s` with the default configuration because the sample
period is `15s`.

### Setup the acquisition mode

The acquisition mode can be configured with the `acquisition_mode` parameter. It can take the following values:
* `"none"` : The sensor doesn't stop the acquisition automatically. This is the **default** mode.
* `"live"` : The sensor stops the acquisition automatically when the live time has reached the acquisition time provided with the `acquisition_time` parameter.
* `"real"` : The sensor stops the acquisition automatically when the real time has reached the acquisition time provided with the `acquisition_time` parameter.

:information_source: With the default configuration the acquisition is stopped before sampling. This is what makes more sense for the default acquisition mode `"none"`, because it doesn't stop acquiring data automatically. The other acquisition modes stop acquiring automatically after the
acquisition time. For that reason it's recommended to ensure the sample period is larger than the acquisition time. This behaviour can be changed so the acquisition isn't stopped before sampling by setting the `stop_acquisition_before_sampling` parameter to `false`.

:information_source: After sampling, another acquisition is always started regardless of the acquisition mode. The ROS driver checks if no acquisition is running, and starts a new acquisition in that case.

This allows to acquire and sample data with many different strategies, which are explained in detail below.

#### Sample spectrum continuously

This is the **default** sample and acquisition strategy. It samples the spectrum continously, that is, it re-starts the acquisition right after taking a sample.

The actual acquisition time isn't accurate with this strategy because the acquisition is stopped from the ROS driver, as opposed to the sensor firmware. If you need the acquisition time to be accurate you need another strategy where the acquisition mode isn't `"none"` and the acquisition time is provided. Such a strategy is described later.

You don't need to provide any configuration, because it's the default, but for illustration purposes, its configuration would be as follows:
```yaml
acquisition_mode: none
stop_acquisition_before_sampling: true
sample_period: 15.0
```

Note that you could use any sample period you want.

This strategy does the following:
```c++
startAcquisition();     // start acquisition
sample_timer_.start();  // start timer with sample period

void sample()  // the sample timer callback
{
  stopAcquisition();   // stop acquisition
  querySpectrum();     // query the spectrum to take a sample
  startAcquisition();  // start acquisition
}
```

### Sample cummulative spectrum

This strategy doesn't stop and re-start the acquisition when a new sample is taken. This means the spectrum has the cummulative counts since the very beginning when the first and only acquisition was started.

This is configured as follows:
```yaml
acquisition_mode: none
stop_acquisition_before_sampling: false
sample_period: 15.0
```

Note that you could use any sample period you want.

This strategy does the following:
```c++
startAcquisition();     // start acquisition
sample_timer_.start();  // start timer with sample period

void sample()  // the sample timer callback
{
  querySpectrum();     // query the spectrum to take a sample
}
```

:warning: If we run this for too long or the spectrum counts increment very fast, it could happen that the counts saturate. However, each channel in the spectrum is represented by a `uint32` type, so it can count up to `2^32 = 4294967296` and it's very unlikely we'd reach that value in practice.

To reduce the risk of saturating the spectrum counts, we could use a `"live"` or `"real"` acquisition mode with a large acquisition time. The configuration would be as follows:
```yaml
acquisition_mode: live
acquisition_time: 3600.0
stop_acquisition_before_sampling: false
sample_period: 15.0
```

:warning: When we reach the acquisition time, the acquisition will stopped automatically, so the sample taken right after will likely have a period of time where the sensor was idle. If that's not acceptable, you shouldn't use this configuration.

:information_source: The acquisition is re-started if the sensor isn't acquiring, so after the acquisition stops automatically after the acquisition time, it'll be re-started. This means the spectrum counts are zeroed out, so the spectrum is only cummulative for the given acquisition time.

### Sample with accurate live or real acquisition time

If the acquisition time must be accurate, we can use the `"live"` or `"real"` acquisition modes. In the example below we use the `"live"` mode, but the `"real"` can be used instead without any additional changes.

This is configured as follows:
```yaml
acquisition_mode: live
acquisition_time: 100.0
sample_period: 101.0
```

:information_source: It's recommeded to use a sample period relatively larger than the acquisition time to avoid sampling while the sensor is still acquiring. For that reason in the example configuration we use `100s` for the acquisition time and `101s` for the sample period. This means there is a `1s` idle period. If that's not acceptable, you shouldn't use this strategy.

This strategy makes sense when we don't need samples very frequently, e.g. only once per hour, but we want a significantly smaller acquisition time, e.g. `100s`, like in the following example configuration:
```yaml
acquisition_mode: live
acquisition_time: 100.0
sample_period: 3600.0
```

However, it's actually possible to use a sample period lower than the acquisition time, but then it could happen that:
* Some samples will have an actual acquisition time lower than the given one, and it won't be accurate.
* Some samples will have been taken after the acquisition time was reached, so the acquisition was automatically stopped and there will be an idle period.

Consider this example:
```yaml
acquisition_mode: live
acquisition_time: 100.0
sample_period: 90.0
```

In this case the following will happen:
* The first sample will be taken at approximately `90s` with the sensor still acquiring, so the spectrum will have an actual acquisition time of approximately `90s`.
* The second sample will be taken approximately `90s` after the first one, that is, `180s` since the beginning. That means that after `100s` since the beginning the acquisition time was reached and the sensor stopped acquiring automatically, so the spectrum will have an actual acquisition time of approximately `10s`, and the other `80s` the sensor was idle.

We can modify this strategy a bit if we stop the acquisition before sampling. In that case we could try using the same sample period as the acquisition time, as in the following configuration:
```yaml
acquisition_mode: live
stop_acquisition_before_sampling: true
acquisition_time: 100.0
sample_period: 100.0
```

In this case we'd have something similar to the strategy where we sample continuously using the acquisition mode `"none"`. The main difference is that the acquisition time will be more accurate for most of the samples, but there's a small chance that sometimes the sensor is still acquiring when we're going to take a sample. In those case, the acquisition is stopped and the only side effect is that the actual acquisition time would be slightly lower than the given acquisition time. That's very unlikely though. This is a good alternative to the continous sampling strategy with the acquisition mode `"none"`, if you need the acquisition time to be accurate and it's acceptable that sometimes is smaller, or there are small idle periods since the sensor stopped acquiring and we take the sample.

### Parameters

MCA initialization parameters:
* **timeout** (default: `1.0`s): MCA initialization timeout in seconds
* **attempts** (default: `8`): MCA initialization attempts
* **baud_rate** (default: `0`): MCA initialization baud rate; `0` means the highest possible baud rate is set

Acquisition parameters:
* **sample_period** (default: `15.0`s): Sample period in seconds
* **acquisition_mode** (default: `"none"`): Acquisition mode: {`"none"`, `"live"`, `"real"`}
* **acquisition_time** (default: `10.0`s): Acquisition time in seconds; only applies to live and real acquisition modes
* **stop_acquisition_before_sampling** (default: `true`): Whether to stop the acquisition before sampling or not
* **high_voltage** (default: `1500`V): MCA high voltage in volts
* **first_channel** (default: `0`): First channel to sample from the spectrum
* **last_channel** (default: `-1`): Last channel to sample from the spectrum. If lower than `0`, it is set to the number of channels available in the MCA minus `1`, i.e. the last channel of all the zero-indexed channels available
* **channels** (default: `1024`): Number of channels for the spetrum. It must be a power of `2` from `128` to `16284`, and not higher than the maximum number of channels reported by the senosr at runtime
* **lld** (default: `0`): Lower channel. It must be lower than the upper channel **uld**
* **uld** (default: `1023`): Upper channel. It must be smaller than the maximum **uld** reported by the sensor at runtime
* **coarse_gain** (default: `10`): Coarse gain. It must be `2`, `5`, `10`, `20`, `50`, `100`, `200`, `500` or `1000`
* **fine_gain** (default: `11252`): Fine gain. It must be `>= 5000` and `<=` the max fine gain reported by the sensor at runtime. For MCA-166 the product of the coarse and fine gains mut be `<= 10000000`
* **shaping_time** (default: `-1`us): Shaping time in microseconds (us). If negative, it is not set
* **save_folder** (default: `""`): Save folder to save .spe files for each acquisition sample. If empty, no .spe file is saved

Power parameters:
* **power_publish_period** (default: `1.0`s): Power state publish period in seconds

State parameters:
* **state_publish_period** (default: `1.0`s): Sate publish period in seconds

System data parameters:
* **system_data_publish_period** (default: `1.0`s): System data publish period in seconds

Calibration parameters:
* **calibration_slope** (default: `1.0`): Calibration slope
* **calibration_offset** (default: `0.0`): Calibration offset
* **calibration_square** (default: `0.0`): Calibration square

The calibration is not used, but it's published on a latched topic so it can be recorded the with the rest of the data and used later using the corresponding calibration formula:
* Linear calibration: `channel = (energy - offset) / slope`
* Quadratic calibration: `channel = (sqrt(slope * slope - 4 * square * (offset - energy)) - slope) / (2 * square)`

### Device shutdown

We can `shutdown` the node with `Ctrl+C` from the terminal we run it.

:information_source: There's no need to wait for the high voltage published in `/power`, or queried by other means, to
reach `0V` before unplugging the device.
