# Carla_dataset_generator

An automatic dataset generator for 2D/3D detection and segment mission based on Carla Simulator.

## Result

![output.gif](output.gif)

## Requirement

```
pip install  requirements.txt
```

## Architecture

carla  
├─carla
│  │  requirements.txt
│  │  scene_layout.py
│  │  
│  ├─agents
│  │  │  __init__.py
│  │  │  
│  │  ├─navigation
│  │  │  │  basic_agent.py
│  │  │  │  behavior_agent.py
│  │  │  │  behavior_types.py
│  │  │  │  controller.py
│  │  │  │  global_route_planner.py
│  │  │  │  local_planner.py
│  │  │  │  __init__.py
│  │  │          
│  │  ├─tools
│  │  │  │  misc.py
│  │  │  │  __init__.py
│          
├─examples
│  │  automatic_control.py
│  │  client_bounding_boxes.py
│  │  dynamic_weather.py
│  │  generate_traffic.py
│  │  lidar_to_camera.py
│  │  manual_control.py
│  │  manual_control_carsim.py
│  │  manual_control_chrono.py
│  │  manual_control_steeringwheel.py
│  │  no_rendering_mode.py
│  │  open3d_lidar.py
│  │  requirements.txt
│  │  sensor_synchronization.py
│  │  show_recorder_actors_blocked.py
│  │  show_recorder_collisions.py
│  │  show_recorder_file_info.py
│  │  start_recording.py
│  │  start_replaying.py
│  │  synchronous_mode.py
│  │  tutorial.py
│  │  tutorial_ego.py
│  │  tutorial_replay.py
│  │  vehicle_gallery.py
│  │  vehicle_physics.py
│  │  visualize_multiple_sensors.py
│          
├─Usr
│  │  automatic_control.py
│  │  config.py
│  │  depth_cam_utils.py
│  │  findBoundingRect.py
│  │  generateSegLabel.py
│  │  jpg2mp4.py
│  │  kittiDelete.py
│  │  makeImageSets.py
│  │  myDelete.py
│  │  myRecordTest.py
│  │  mySynchronize.py
│  │  newSynchronize.py
│  │  open3d_test.py
│  │  parseLLtxt.py
│  │  ply2bev.py
│  │  ply2bin.py
│  │  pos2kitti.py
│  │  position2bev.py
│          
└─util
    │  check_collisions_substepping.py
    │  check_lidar_bb.py
    │  check_raycast_sensors_determinism.py
    │  config.py
    │  environment.py
    │  lane_explorer.py
    │  performance_benchmark.py
    │  raycast_sensor_testing.py
    │  requirements.txt
    │  test_connection.py
    │  vehicle_physics_tester.py
    │  


## Contact

If you think this work is useful, please give me a star!  
If you find any errors or have any suggestions, please contact me (**Email:** `sekirorong@gmail.com`).  
Thank you!

## Reference

1. [GitHub - carla-simulator/carla: Open-source simulator for autonomous driving research.](https://github.com/carla-simulator/carla)

2. KITTI: http://www.cvlibs.net/datasets/kitti/raw_data.php
