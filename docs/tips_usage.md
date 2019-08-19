Tips for usage
----------------------
- The 3D Visualization window implements the following keyboard shortcuts (you need to have the window in focus: click on it):
    - Press `t`: toggle freezing visualization.
    - Press `v`: prints pose of the current viewpoint of the 3D visualization window.
    - Press `w`: prints to the terminal the size of the 3D visualization window.

     > These last two shortcuts are useful if you want to programmatically set the initial viewpoint and size of the screen when launching the 3D visualization window (this is done at the constructor of the 3DVisualizer class).

    - Do not press `q` unless you want to terminate the pipeline in an abrupt way (see #74).
    - Press `s`: to get a screenshot of the 3D visualization window.
    - Press `0`, `1`, or `2`: to toggle the 3D mesh representation (only visible if the gflag `visualize_mesh` is set to true).
    - Press `a`: to toggle ambient light for the 3D mesh (gflag `visualize_mesh` has to be set to true).
    - Press `l`: to toggle lighting for the 3D mesh (gflag `visualize_mesh` has to be set to true).

- The 3D Visualization allows to load an initial `.ply` file.
This is useful for example for the Euroc dataset `V1_01_*` where we are given a point-cloud of the scene. Note that the `.ply` file must have the following properties in the header, and its corresponding entries:
  - A vertex element: `element vertex 3199068` (actual number depends on how many vertices has the ply file).
  - A face element: `element face 0`(actual number depends on how many faces has the ply file)
  - Properties may vary, as for now, any given `.ply` file will be displayed as a point-cloud, not a mesh (due to some weirdness in opencv).
  - For example, the ground-truth point-cloud provided in `V1_01_easy` dataset must be modified to have the following header:
      ```
      ply
      format ascii 1.0
      element vertex 3199068
      property float x
      property float y
      property float z
      property float intensity
      property uchar diffuse_red
      property uchar diffuse_green
      property uchar diffuse_blue
      element face 0
      property list uint8 int32 vertex_indices
      end_header
      ```
