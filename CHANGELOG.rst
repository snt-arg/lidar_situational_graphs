^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package s_graphs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [Feat] Update walldata msg
* [Feat] Change graph_manager_msgs->reasoning_msgs
* Contributors: Hriday Bavle

2.0.0 (2022-10-19)
------------------
* Switch to random color in plane visualization
* Increase floor node height
* Bug fix room analyzer throwing nan
* Merge branch 'feature/s_graphs_2.0' of github.com:snt-arg/s_graphs into feature/s_graphs_2.0
* Add more noise to odom in s_graphs.yaml
* Merge pull request `#12 <https://github.com/snt-arg/s_graphs/issues/12>`_ from snt-arg/refactor/docker_image
  Just modified the command to remove the source files.
* Imporoved the command to remove source files
* Add vertex neighbours param in yaml
* Add rainbow color function for paiting planes
* Improve visualzation of the S-Graphs
* Merge pull request `#10 <https://github.com/snt-arg/s_graphs/issues/10>`_ from snt-arg/feature/keyframe_mapper
  Separate keyframe mapper functionality
* Add sub_room variable
* Add vertex neighbours as rosparam
* Avoid drawing sub-rooms
* Separate keyframe mapper functionality
* Merge branch 'feature/s_graphs_2.0' of github.com:snt-arg/s_graphs into feature/s_graphs_2.0
* Update rviz file
* Update - Adding commands to remove source files
* Fix typo in floor edges
* Change default room edge color to black
* Add map_extracted_planes func to plane mapper class
* Merge pull request `#9 <https://github.com/snt-arg/s_graphs/issues/9>`_ from snt-arg/feature/Visualizer
* Add graph visualizer to Cmakelists
* Add graph visualizer changes to s_graphs_nodelet
* Merge conflicst
* Refactor
* Merge pull request `#8 <https://github.com/snt-arg/s_graphs/issues/8>`_ from snt-arg/feature/docker_image
  Update - updating the dockerfile to the newer version
* Update - updating the dockerfile to the newer version
* Merge pull request `#7 <https://github.com/snt-arg/s_graphs/issues/7>`_ from snt-arg/revert-6-feature/Visualizer
  Revert "Verify separation of visualization "
* Revert "Verify separation of visualization "
* Merge pull request `#6 <https://github.com/snt-arg/s_graphs/issues/6>`_ from snt-arg/feature/Visualizer
  Verify separation of visualization
* Seperate visualization functionality
* Fix - Fixed command to launch the rosbag
* Reduce loop closure radius in config file
* Improve cout info in infinite room mapper
* Add cluster center as fixed node in the graph
* Comment unwanted couts in all apps
* Comment unwanted couts in room_analyzer
* Remove voxblox mesh from rviz file
* Comment room creation from pre-existing corridors
* Minor adjustments to the code, not harmfuk
* Probable bug fix in room/corridor multi-edge
* Merge branch 'feature/s_graphs_2.0' of github.com:snt-arg/s_graphs into feature/s_graphs_2.0
* Add room-plane multi-edges
* Update - Changing git url from ssh to https
* Update - Changing git URLs from ssh to https
  This change is to work with docker
* Add new corridor-planes multi-edge
* Merge branch 'feature/s_graphs_2.0' of github.com:snt-arg/s_graphs into feature/s_graphs_2.0
* Revert back to implementation of distance only room-plane edge
* Update README.md
* Remove overlapped corridors and reduce size of room edges
* Merge pull request `#4 <https://github.com/snt-arg/s_graphs/issues/4>`_ from snt-arg/update/documentation
  Fix `#2 <https://github.com/snt-arg/s_graphs/issues/2>`_ & `#3 <https://github.com/snt-arg/s_graphs/issues/3>`_ Addition of the .rosinstall file + updated the readme file + creating of the script setup.sh
* Small Change in Readme install instructions
* Update README.md
* Fix - Fixed some typos
* Fix - Corrected name of the folder ws from s-graphs_ws to s_graphs_ws
* Refactoring - Changed the name S_Graphs to S-Graphs and fixed some inconsistencies
* Merge branch 'update/documentation' of github.com:snt-arg/s_graphs into update/documentation
* Refactoring - Removed empty first line
* Fix - Fixed the maintainer email + updated the version to be 2.0.0
* Update - Added compile_commands.json to be ignored
* Update README.md
* Bug fix - Added the arg -r to the rosdep install command + added line to install catkin-tools
* Update - Removed the section requirements
* Add floor segmentor in one launch file
* Comment couts
* Improve config s_graphs
* Update - Removed the unusefull comments
* Fix - Fixed the identation of the file
* Merge branch 'update/documentation' of github.com:snt-arg/s_graphs into update/documentation
* Creation - Created a script that install all dependencies automatically
* Creation - Created the .rosinstall file and added the different dependencies
* Adding - Installation procedure section + Example section
  Added the manual installation procedure. This is to be completed once the .rosinstall file is created.
  Added the example section, where it shows how to run a S_graphs on a real data set and a simulated dataset.
* Add install targets in Cmakelists
* Remove unnecessary launch files
* Improve floor-room edge error
* Comment prior edge on corridor pose
* Merge pull request `#1 <https://github.com/snt-arg/s_graphs/issues/1>`_ from snt-arg/bug/launch_file_correction
  Bug/launch file correction
* Bug fix in infinite room prior edge
* Update - adding the ignore for .vscode folder
* Fix Removig suffix /slam from s_graphs.launch
* Add sim time true in s_graphs.launch
* Remove rviz from gitingore
* Change s_graphs.launch for running wo stugalux_ws
* Add s_graphs.launch and config files
* Add floor/room edges but not a good implementation
* Add floor node to the graph, remaining egdes between rooms/floors
* Add initial structure for floor node
* Remove mutex in odom pose publisher
* Add x and y priors for x and y corridor poses respectively
* Check plane points for matching both finite and infinite rooms
* Add corridor measurement as 2d vector
* Remove edges from marker array
* Add sign check in room plane edge
* Edge case fix in corridor plane error calculation
* Add corridor error to zero at start
* Fix small error in corridor information matrix
* Bug fix corridor plane edge
* Add room/corridor plane edges only when new planes are detected
* Default edge colors to black
* Add plane information in yaml
* Bug fix subcluster calculation
* Spell fix keyframewindowsize
* Add keyframe window in config
* Add TODOs
* Compute corridor pose from vector
* Revert back to working commet of 60167fd and subdividing clusters for getting corridors centers
* Improve room measurement to vector format
* Increase point matching threshold for room detection
* Bug fix in Finite and Inifinite room measurement
* Remove check for s_graph marker for publishing map cloud
* Remove text from rooms/corridors
* Perform plane segmentation before adding keyframe relative pose edges
* Add dot product check to mapped planes for floor node calculation
* Bug fix room pose calculations
* Feature: change the implementation of room_pose computation, compatible with any room orientations
* Change from walltimer to ros timer
* Fix bug in mapperutils functions
* Add option to downsample free space pointcloud
* Comment merge duplicate planes
* Format scan matching code
* Remove couts and small bug fixes also uncomment mergeduplicate planes and comment flush all mapped planes
* Increase map interval and comment cout
* Change room/corridor colors
* Improve visualization of overlapped corridors
* Add floor node in markerarray
* Add floor analyzer and publish floor position
* Improve corridor viz
* Improve further the visualization of the s-graph
* Improve visualization of the s-graphs
* Published refined skeleton graph
* Remove plane segmentation nodelet, not required now
* Add max point neighbour check for matching two planes and removing duplicate planes only after several occurances of the dupl plane
* Add param to extract planar surfaces
* Add plane analyzer in room segmentor
* Add plane analyzer files
* Bug fix xy plane alignment
* Intent to fix bug for xy plane alignment but not working
* Bound the room detections using plane point check
* Add room xy plane check and add entire cloud cluster to nearest neighbour check
* Increase cluster threshold back in plane segmentation
* Fix problem for finding planes oriented outwards
* Factor neighbours correctly using floor plan nodelelet
* Publish all mapped planes and all room data
* Add function perform_room_segmentation inside room_analyzer.cpp
* Room detector working over weird shapes
* Improve coloring of x-y planes
* Bug fix in plane-point matching function
* Add version of max neighbour check in room detection node
* Improve associate corridor functions
* Remove extra code in s_graph_nodelet
* Add neighbour mapper
* Move code from s_graph_nodelet to room mappers
* Add second lookup room function in finite room mapper
* Clean code which was moved to planemapper class
* Add plane mapper class
* Move more functions to plane utils
* Add finite and infinite room mapper clases
* Rename to infinite and finite room mapper
* Remove corridor mapping related functions from s_graph_nodelet
* Add a new cpp for room mapper
* Increase matching threshold for plane-freespace points
* Improve the coloring of the planes
* Add function overloading for map cloud generator
* Enable publish map points when subsribing to marker arrays as well
* Big commit: Improve code structure of room sementation nodelet, creating a new room analyzer
* Adjust threshold in plane point matching
* Bug fix in point plane dist calculator
* Add lifetime for markerarrays
* Publish map planes before optimixzation
* Further improve plane finding for each free space cluster
* Add version 1 of replacing room planes with mapped corridors and vice and versa. Not working version
* Add small hacks to improve the mapped plane publishing
* Publishing room centers after receiving mapped plane measurements
* Add neighbour edges between neighbouring rooms and corridors
* Improve the implementation of searching and visualizing room neighbours
* Clean code using clang formatting
* Fix bug in factoring X corridor and removed pre-room neighbour check
* Fix bug in room neighbourg viz but still buggy
* Visualizing neighbours in the in the s_graph_nodelet
* Check and publish neighbours of each room, time to connect this b**ches
* Reduce the plane-point matching threshold in room segmentation
* Add corridor node into the graph obtained from room segmentation node
* Check only the last 10 keyframes for mapped planes publishing
* Add detected rooms from room segmentation node to the graph
* Fix Y corridor pose calculation bug
* Add points in closest plane check
* Add check for centroid center
* Subscribe to different subgraphs for getting room candidate
* Add diagonal check to seperate different corridors
* First version of axis clustering for corridors
* Add code for fitting line segment
* Add corridor segmentation
* Improve room segmentation and the plane finding procedure
* Add option for publishing 3D points with the map planes
* Improve code getting room clusters
* Publish only rooms which are supported by planar surfaces
* Add map planes publisher
* Subscribe to room data msg
* Change name room msg name
* Visualize possible room node poses in room segmentation
* Add room segmentation msg
* Comment topological layer callback thread
* Add room segmentation nodelet
* Comment lookup rooms in topological callback
* Add lookup rooms in topological layer thread
* Add x,y and hort plane ids to each keyframe
* Add seperate callback for detecting and adding topological constraints
* Add param for plane points min distance
* Feature: Add edge between room node to detected mapped plane after removing dupl plane
* Feature: Add edge between corridor and detected mapped plane after removal of dupl plane
* Add corridor min seg dist as a ros param
* Bug Fix: Fix egde se3 plane line visualization
* Bug Fix: Finally fixed the code crash in create_marker_array function
* Bug Fix: Node crash during removal of vert planes
* not workin version (tmp commit)
* Feature: Merging planes for room nodes as well (potentially buggy implementation)
* Feature: merging duplicate plane nodes detected by corridors
* Feature: merging duplicate y planes detected by corridor nodes (implementation untested)
* Feature: Seperate corridors based on different walls
* Pose-Plane edges in black color
* Fix corridor mapping visualization
* Fix plane2keyframe edge
* Comment couts
* Improvement: edge creation of corridors and rooms (probably buggy implementation)
* Feature: basic version of wall detection instead of planar surfaces
* Possible fix: mapping planes in all orientations
* Merge branch 'main' of github.com:snt-arg/s_graphs into main
* Rename hdlGraphslamnodelet to sgraphNodeley
* Update README.md
* Resize image in readme
* Update readme
* Rename launch files
* Add export targets for proper proper ros message build
* First Commit: Rename from HDL_SLAM to s_graphs
* Add parallel plane constaint for newly associated planes of corridors and rooms
* Add option constant covariance
* Feature: Improved visualalization of the room edges and robot pose-plane edges
* Feature: Add line connections between room/corridor nodes and semantic planes
* Feature add entire point visualization of semantic map
* Bug fix in corridor parallel plane constraint
* Add max room width check
* Add color variable in yaml file
* Fix bug in room-plane, corr-plane edge measurement eq
* Improve debugging visualization
* Add ROS_DEBUG_NAMED instead of cout
* Fix bug in point to plane param
* Add params for room width diff
* Seperate functions for corridor and room lookup
* Add switch cases for organizing the better the plane matching
* Improve room check condition
* Add point diff to check consistency of corridors and rooms instead of length diff
* Initial version of pose and path publishers
* Fix edgeplane visualization
* Improve params for corridor factor
* Update config file for plane factor related params
* Fix bug in corridor measurement function
* Documenting a bit a code
* Add proper corridor factor
* Reduce the plane extraction distance thres
* Fix wrong push in room_vec pose
* Improve plane_d correction
* Add parallel and perpendicular only between planes of rooms
* Change corridor vertex to single number and changed room node implementation
* Improve corridor pose but yet to fix the bugs
* Change corridor meas from vector to double
* change the sign of the corridor pose
* Update params for plane matching
* Include only parallel planes
* Improve clustering in plane segmentation
* Improve sorting and refining of corridors and rooms
* Add eq clustering
* Increase cov of plane meas
* Improve bugs in corridor and room factor
* Improve room pose calculation, TODO: fix corridor pose calc
* Add config for enabling and disbaling room and corridor factor
* Improve room factoring logic and decreased the plane detection thres
* Add room squareness check
* Paint edges in white
* Add diff plane filters and several prints for debug in mapping
* Add Z axis in corridor vertex
* Add corridor pre-pose and final-pose
* Add seperate corridor vertex
* Seperated X and Y corridor edges
* Connect room node with keyframe node
* Add Room Vertex
* improve plane_seg_launch
* Clean the code and improve implementation of planepoints in map frame
* Reorganize the code and add thresholds as ros params
* Add rosparams for most of the threshold values
* Fix plotting of X corridor
* Add params for plane filtering
* Improve ground plane segmentation
* Add Room plane visualization
* Improve edge drawing for planes
* Improve naming corridors and rooms
* Add struct for planedata
* Reorganize corridor factor function
* Improve drawing of parallel planes
* Draw corridor and room nodes
* Fix bugs with room factor
* Fix bug in width measurement of corridots
* Add version 1 of room factor
* Improve implementation of corridor factor
* Fix bug in corridor matching and add basic structure for room node
* Add X corridor
* Add id for corridor
* Version 1 of Y corridor factor
* Add enum for plane class
* Add corridor first edge
* Add skeletal structure for corridor_plane edge
* Add edge plane edges
* Ploting segmented planes
* Add perpendicular plane constraint and comment drawing of parallel planes
* Visualize parallel plane factors
* Fix bug in plotting parallel planes
* Clean code for point to plane
* Improve parallel constraint between planes
* Add plane parallelity check in struct
* Fix bug in parallel plane edge
* Add parallel plane constraints
* Compare maha distance in robot frame
* Fix error with ploting the point-plane edge
* Remove map frame plane fix
* Comment plane segmentation
* Publish planes in closest point form
* Different colors for different plane edges
* Add horizontal plane
* Data association using mahalonobis distance
* Computing marginals
* Comment the CP plane form
* Improve point to plane
* Add 3D plane in thes struct for vert planes
* Add launcher for slam backend only
* Try and catch in map2odom transform
* Improve data association of planes
* Change back to map to odom transform as identity
* Add first version of point_plane factors for x and y vert planes
* Remove redundant variable from calc in point_to_plane factor
* Improve point to plane factor and add Y-axis plane
* Complete math for point to plane factor
* Add skeletal for point to plane factor
* Add proper implementation of pointcloud segmented in local (body) frame
* Revert back to segmenting cloud in map frame
* Receive the segmented cloud in local body frame
* Fixed bug in plane mapping
* Further improve logic for x_vert plane mapping
* Add better logic for associating x_plane
* Add vert plane seg in a function
* Add custom pointcloud vector message
* Remove minus from dist estimate
* Add the x-plane constraint
* Add planes struct and add vert plane with data association (math still not working)
* Add vertical plane x to the graph
* Improve implementation plane segmentation normal publisher
* Publish pointcloud with its normals
* Complete logic for keyframe to vert plane edge
* Change implementation of seg cloud subsriber
* Add sub in hdl for plane pointcloud
* Add map frame instead of base_link for plane seg
* Add filtered points instead of velodyne points
* Clean code and adhere to proper code terminology
* Version 1 working in simulation wo crashes
* Remove unncesessary files
* Add point removal
* Struggle to get proper pointcloud segmentation
* publishing the largest plane with green color
* Add plane segmentation nodelet in nodelet_plugin.xml
* Add plane segmentation
* Add subscriber for pointcloud in plane seg
* Add plane_segmentor_nodelete in cmake
* Fix clang-format
* Initial commit for plane segmentro
* Merge pull request `#1 <https://github.com/snt-arg/s_graphs/issues/1>`_ from hridaybavle/fix-callback-not-being-called
  Add ros::spinOnce() to make sure callbacks are being called when insi…
* Add ros::spinOnce() to make sure callbacks are being called when inside while loop
* Add initial odom2map transform listener
* added param for publishing tf for odom and base_link
* Merge pull request `#190 <https://github.com/snt-arg/s_graphs/issues/190>`_ from koide3/fix
  fix dependency issue
* fix dependency issue
* Merge pull request `#185 <https://github.com/snt-arg/s_graphs/issues/185>`_ from ksuszka/master
  Fixed formatting UTM origin coordinates in second save function
* Merge pull request `#187 <https://github.com/snt-arg/s_graphs/issues/187>`_ from koide3/fix_ci
  fix CI error
* fix CI error
* Fixed coordinates formatting in save_map_service function
* Merge pull request `#183 <https://github.com/snt-arg/s_graphs/issues/183>`_ from koide3/devel
  Devel
* refactor dockerfiles
* fix typo and format issue
* Merge pull request `#165 <https://github.com/snt-arg/s_graphs/issues/165>`_ from koide3/pub_status
  Pub status
* Merge pull request `#162 <https://github.com/snt-arg/s_graphs/issues/162>`_ from koide3/vgicp_cuda
  vgicp_cuda
* fix inlier_fraction calculation bug
* fix for melodic
* initial guess based on robot odometry
* rename to matching_error
* add ScanMatchingStatus.msg
* add vgicp_cuda
* Merge branch 'master' of github.com:koide3/hdl_graph_slam
* add launch file for KITTI00
* Update howtouse.md
* Update howtouse.md
* Merge branch 'master' of github.com:koide3/hdl_graph_slam
* add docker howtouse.md
* Merge pull request `#158 <https://github.com/snt-arg/s_graphs/issues/158>`_ from jitrc/devel
  Publishing aligned point cloud if subscribed, fixed use of map_cloud_resolution
* Merge pull request `#160 <https://github.com/snt-arg/s_graphs/issues/160>`_ from koide3/ndt_params
  fix ndt param name bug
* fix ndt param name bug
* Merge pull request `#157 <https://github.com/snt-arg/s_graphs/issues/157>`_ from koide3/nan_angle
  fix delta angle evaluation bug
* fix delta angle evaluation bug
* pass map_cloud_resolution to map_cloud_generator
* publish aligned points in odom frame
* allow generating unfiltered point cloud
* Update README.md
* Merge pull request `#152 <https://github.com/snt-arg/s_graphs/issues/152>`_ from robustify/crash_on_loop_closure
  Normalize orientations in loop closure candidate keyframes
* Normalize orientations in loop closure candidate keyframes
* Merge pull request `#151 <https://github.com/snt-arg/s_graphs/issues/151>`_ from robustify/rospy_setup
  Use rospy and setup.py to manage shebangs for Python 2 and Python 3
* Use rospy and setup.py to manage shebangs for Python 2 and Python 3
  Following guidance found here: http://wiki.ros.org/UsingPython3/SourceCodeChanges#Changing_shebangs
* Merge pull request `#150 <https://github.com/snt-arg/s_graphs/issues/150>`_ from koide3/refactor
  refactoring
* refactoring
* Merge pull request `#149 <https://github.com/snt-arg/s_graphs/issues/149>`_ from koide3/fast_gicp
  Add fast_gicp
* add fast_gicp
* Merge pull request `#148 <https://github.com/snt-arg/s_graphs/issues/148>`_ from koide3/noetic
  Update for Focal Fossa & ROS Noetic
* update for noetic
* Merge pull request `#146 <https://github.com/snt-arg/s_graphs/issues/146>`_ from krisklau/clang-format
  entire repo: clang-format.
* entire repo: clang-format.
  Processed with the command:
  find . -type f \( -name "*.cpp" -o -name "*.hpp" \) -execdir clang-format-6.0 -i {} \;
* add license identifiers
* Merge branch 'master' of https://github.com/koide3/hdl_graph_slam
* add transformation_epsilon
* Merge pull request `#128 <https://github.com/snt-arg/s_graphs/issues/128>`_ from tim-fan/master
  Approximate time sync for odom/scan input to HdlGraphSlamNodelet
* Use approximate time sync for odom/scan input to HdlGraphSlamNodelet
* Merge pull request `#124 <https://github.com/snt-arg/s_graphs/issues/124>`_ from koide3/devel
  Fix resource consuming problem
* disable deskewing by default
* fix consuming resource after data flow is stopped
* fix a loading bug
* Merge pull request `#102 <https://github.com/snt-arg/s_graphs/issues/102>`_ from naoki-mizuno/radius-outlier-removal
  Fix RadiusOutlierRemoval not applied
* Fix RadiusOutlierRemoval not applied
* preliminary implementation of IMU-based frontend
* Create LICENSE
* Merge pull request `#96 <https://github.com/snt-arg/s_graphs/issues/96>`_ from koide3/devel
  Update of the first node anchor mechanism
* build check with clang & lld
* make first node anchor information matrix configurable
* Merge pull request `#93 <https://github.com/snt-arg/s_graphs/issues/93>`_ from koide3/devel
  fix a bug in odometry information matrix calculation
* fix empty marker bug
* fix a bug in odometry information matrix calculation
* Merge pull request `#91 <https://github.com/snt-arg/s_graphs/issues/91>`_ from Tutorgaming/patch-1
  Fix ros-kinetic-pcl-ros typo inside readme :)
* Fix ros-kinetic-pcl-ros typo inside readme :)
  On the installation guide inside the readme
  the package name was misspell
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update hdl_graph_slam_501.launch
* Update hdl_graph_slam_400.launch
* Update hdl_graph_slam.launch
* Merge pull request `#81 <https://github.com/snt-arg/s_graphs/issues/81>`_ from koide3/devel
  Add normal orientation-aware plane edge
* update identity plane edge for kinetic
* fix g2o vector error
* add normal orientation-aware plane edge
* Merge pull request `#79 <https://github.com/snt-arg/s_graphs/issues/79>`_ from koide3/devel
  Configurable scan matching parameters & orientation constraint bug fix
* fix orientation constraint bug
* expose scan matching parameters in hdl_graph_slam.launch
* make scan matching parameters configurable
* fix orientation constraint bug & make solver configurable
* add plane edges
* Merge branch 'master' of https://github.com/koide3/hdl_graph_slam
* add plane prior
* Update hdl_graph_slam_nodelet.cpp
  Fix typo
* Merge pull request `#72 <https://github.com/snt-arg/s_graphs/issues/72>`_ from koide3/devel
  Add functions & edges for interactive SLAM
* Merge branch 'master' of https://github.com/koide3/hdl_graph_slam into devel
* update README
* add perpendicular plane edge
* merge
* update for interactive_map_correction
* Merge pull request `#69 <https://github.com/snt-arg/s_graphs/issues/69>`_ from ktk1501/issue-66/road_flipped_problem
  road_flipped_problem fix by adding minus to Z in g2o optimizer
* road_flipped_problem fix by adding minus to Z in g2o optimizer
* Merge pull request `#67 <https://github.com/snt-arg/s_graphs/issues/67>`_ from ll7/patch-1
  pcl-ros
* pcl-ros
* Merge pull request `#50 <https://github.com/snt-arg/s_graphs/issues/50>`_ from jihoonl/patch-1
  Add libg2o as build depend
* add plane edges
* update for interactive map correction
* Merge branch 'devel' of https://github.com/koide3/hdl_graph_slam into devel
* fix typo
* add license
* Update README.md
* Update README.md
* Update README.md
* Add libg2o as build depend
* update .travis.yml
* Merge pull request `#43 <https://github.com/snt-arg/s_graphs/issues/43>`_ from koide3/devel
  add codacy and refactoring
* update README
* fix format issues
* add codacy and refactoring
* Merge pull request `#41 <https://github.com/snt-arg/s_graphs/issues/41>`_ from koide3/arch-melodic
  Support melodic & build test
* remove unnecessary launch files
* update README.md
* update dockerfiles and .travis.yml
* add build test
* update for melodic
* add start_time option to bag_player.py
* add fitness_score_max_range
* fix a reference error on clang
* Update README.md
* Merge branch 'master' of https://github.com/koide3/hdl_graph_slam into devel
* update so map can be saved without visualization
* Merge pull request `#34 <https://github.com/snt-arg/s_graphs/issues/34>`_ from koide3/devel
  New constraints
* update README
* update launch files
* update so that the package can find ros libg2o
* update README.md
* some comments
* add new constraints, robust kernels, optimization params
* Update README.md
* update README and Dockerfile
* update README.md
* update scan_matching_odometry so that it retrieves base_frame_id from messages and add missing params to launch files
* update README.md and add Dockerfile
* update README.md
* add dependency on ndt_omp to package.xml
* Update README.md
* update README.md
* add SaveMap.srv and add an outdoor mapping example
* update README.md
* update README.md
* modify README.md
* initial commit
* Contributors: Eduardo Schmidt, Hriday Bavle, Jihoon Lee, Jit Ray Chowdhury, Kenji Koide, Kristian Klausen, Krzysztof Suszka, Micho Radovnikovich, Muhammad Shaheer, Naoki Mizuno, Pedro, Pedro Soares, PedroS, PedroS235, Tim, hriday, iTUTOR, k.koide, kenji koide, koide, koide3, ktk1501, ll7, mbzirc, shaheer34mts
