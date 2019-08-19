Tips for development
----------------------
- To make the pipeline deterministic:
    - Disable TBB in GTSAM (go to build folder in gtsam, use cmake-gui to unset ```GTSAM_WITH_TBB```).
    - Change ```ransac_randomize``` flag in ```params/trackerParameters.yaml``` to 0, to disable ransac randomization.

> Note: these changes are not sufficient to make the output repeatable between different machines.

> Note: remember that we are using ```-march=native``` compiler flag, which will be a problem if we ever want to distribute binaries of this code.
