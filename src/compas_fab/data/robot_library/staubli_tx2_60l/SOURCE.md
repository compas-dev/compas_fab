# Stäubli TX2-60L model assets

The TX2-60L URDF and meshes come from the ROS-Industrial repositories:

- [`ros-industrial/staubli_experimental`](https://github.com/ros-industrial/staubli_experimental),
  branch `kinetic-devel`, commit `256a71acea2161b64e12af1a6b925a7f26695007`.
- [`ros-industrial/staubli`](https://github.com/ros-industrial/staubli),
  branch `melodic-devel`, commit `367d92ec7432867d9ba87c3399d49a940c4afc67`.

The URDF was flattened from
`staubli_tx2_60_support/urdf/tx2_60l.xacro`. Links 5 and 6 are intentionally
referenced from the shared `staubli_tx60_support` mesh package, as in the
upstream model. The assets are covered by the adjacent Apache-2.0 `LICENSE`.
