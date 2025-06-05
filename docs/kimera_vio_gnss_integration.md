# Kimera-VIO with GNSS Integration

This fork extends [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) by incorporating GNSS support into the stereo visual-inertial pipeline. This implementation adds loosely coupled GNSS integration via custom GTSAM factors. The GNSS data is treated as pose factors (position only).

Due to lack of access to raw GNSS measurements in the [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset, ground truth data with added synthetic noise was used to simulate GNSS input.

> Developed as part of a Bachelor's thesis at Saint Petersburg State University.

## Architecture

### Added Components
Main modules:
* `GNSSVIODataProvider`: loads GNSS data from dataset
* `GnssStereoDataProviderModule`: synchronizes different measurements at one time
* `GnssStereoVisionImuFrontend`: passes GNSS data to backend
* `GnssVioBackend`: performs graph optimization taking into account GNSS factors

Additional components:
* `GnssParams`: holds GNSS-related parameters (added to `VioParams`) The `BackendParams` were also modified
* `GnssFactor`: custom GTSAM factor applying a positional constraint to a pose
* `ThreadSafeGnssBuffer`: threadsafe queue for GNSS measurements
* Unit tests for new modules

### Backend Integration

GNSS factors are injected into the factor graph using `BeforeOptimizeHook()` in the `RegularVioBackend`. No changes to iSAM2 optimizer needed. Robustness to GNSS noise is achieved via configurable loss functions in GNSS factors: `L2`, `Huber`, or `Tukey`.

## Dataset Format

Simulated GNSS data should be located at `gnss0/data.csv`:

```csv
#timestamp;x;y;z
1403636585023555584;0.2;1.5;0.0
...
```

Sensor transformation (from GNSS coordinates to body coordinates) should be defined in `gnss0/sensor.yaml`.

## Build and Run (Windows via WSL + Docker)

1. Clone the [repository](https://github.com/kleo-53/Kimera-VIO), switch to the `dev` branch
2. In `scripts/gnss/kimera_vio_docker.bash` update the volume paths for the dataset and the code directory.
3. Open WSL, go to the folder with the project
4. Execute commands:
```bash
docker build -t kimera_vio -f scripts/gnss/Dockerfile .
./scripts/gnss/kimera_vio_docker.bash
```
5. Inside the docker container in the `Kimera-VIO/` folder, recreate the `build/` folder and build the library there:
```bash
rm -rf /root/Kimera-VIO/build/*
mkdir -p ~/Kimera-VIO/build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(($(nproc)-1))
```
6. Run the script, providing the path to the dataset within the container: 
```bash
../scripts/gnssVIOEuroc.bash -p /Euroc/MH_01_easy -log
```

Use Jupyter notebooks from the [evaluation repository](https://github.com/MIT-SPARK/Kimera-VIO-Evaluation) to visualize results and compare error metrics.

## Results

At optimal parameters, the implementation with GNSS produces more accurate trajectories than VIO without GNSS. When GNSS data is noisy or sparse, the performance gracefully degrades and remains on par with standard VIO.
The parameters that produced the best results can be found in the `Params\EurocGnss` folder.

## Links

* [Pull Request #258](https://github.com/MIT-SPARK/Kimera-VIO/pull/258)
* [Kimera-VIO Repository](https://github.com/MIT-SPARK/Kimera-VIO)

---

For research use. Feedback and collaboration welcome!
