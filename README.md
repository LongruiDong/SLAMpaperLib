<p align="center">
 <p align="center">✔ This repo collects some links with papers which I recently starred related on SLAM , sensor fusion ,etc.</p>
</p>

<!--
✔ This repo collects some links with papers which I recently starred related on SLAM , sensor fusion ,etc.
-->

paperswithcode.com: [sensor-fusion](https://paperswithcode.com/task/sensor-fusion)、
[visual-odometry](https://paperswithcode.com/task/visual-odometry/)、
[visual-localization](https://paperswithcode.com/task/visual-localization/)

# sensor fusion

## camera+imu

### filtering-based




《Iterated extended Kalman filter based visual-inertial odometry using direct photometric feedback》 [Rovio](https://github.com/ethz-asl/rovio)
> a fully robocentric and
direct visual-inertial odometry framework;  the fully robocentric formulation of a visual-inertial odometry, as well as
in the tight integration of the photometric error

《Maplab: An Open Framework for Research in Visual-Inertial Mapping and Localization》 [maplab](https://github.com/ethz-asl/maplab)
> an open visual-inertial mapping framework for processing and manipulating multi-session maps, a collection of multisession mapping tools that include map merging, visual-inertial batch optimization, and loop closure.  Maplab
includes ROVIOLI (ROVIO with Localization Integration)

《Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight》[S-MSCKF](https://github.com/KumarRobotics/msckf_vio)
> stereo version of MSCKF

《Robocentric visual–inertial odometry》[R-VIO](https://github.com/rpng/R-VIO)
>  robocentric sliding-window filtering-based VIO; MSCKF-based



《OpenVINS : A Research Platform for Visual-Inertial Estimation》[code](https://github.com/rpng/open_vins)

> 基于Sliding window visual-inertial MSCKF 的VIO开源框架；
文中总结了以往的开源VIO

《An Efficient Schmidt-EKF for 3D Visual-Inertial SLAM》 未开源 CVPR2019
> tightly coupled;adapt the Schmidt Kalman filter formulation to selectively include informative features in the state vector, achieve linear computational complexity in terms of map size.

### optimization-based
《 Keyframe-based visual–inertial odometry using nonlinear optimization》 [OKVIS](https://github.com/ethz-asl/okvis)

> an excellent keyframe-based VI-SLAM
system; that combined the IMU and reprojection error terms into a cost function to optimize the system

《Visual-Inertial Monocular SLAM with Map Reuse》 [VIORB（非官方开源）](https://github.com/jingpang/LearnVIORB)

> a monocular tightly coupled VI-SLAM based on ORB-SLAM and contains an ORB
sparse front-end, graph optimization back-end, loop closure, and relocation.

《Kimera: an Open-Source Library for Real-Time
Metric-Semantic Localization and Mapping》 [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)

> MAP问题，factor-graph 优化

《Closed-form preintegration methods for graph-based visual–inertial navigation》[code](https://github.com/rpng/cpi)
> use CPI(closed-form preintegration), an indirect (feature-based), tightly coupled, sliding-window optimization based VIO;  loosely coupled direct image alignment VIO



《Visual-Inertial Mapping With Non-Linear Factor Recovery》 [code](https://gitlab.com/VladyslavUsenko/basalt)
> VIO is formulated as fixed-lag smoothing
which optimizes a set of active recent frames in a sliding window and keeps past information in marginalization priors

《A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors》[VINS-fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
> an optimization-based multi-sensor state estimator;whose frond-end uses the KLT optical flow to track the Harris corner, while the back-end uses a sliding window for nonlinear optimization. The entire system includes measurement processing, estimation initialization, local bundle adjustment without relocalization, loop closure, and global pose optimization.

《ICE-BA: Incremental, Consistent and Efficient Bundle Adjustment for Visual-Inertial SLAM》 CVPR2018  [code](https://github.com/baidu/ICE-BA)
> the first BA based VI-SLAM solver, sliding window based bundle adjustment

## lidar+vision

[lidar+camera博客](https://mp.weixin.qq.com/s/d65KK2-G0gVjId34GL2r3w)

《Multi-view 3D Object Detection Network for Autonomous Driving》 CVPR2017
[code](https://github.com/bostondiditeam/MV3D)

> 提到不同的融合方式：early, late ,deep fusion


《LATTE: Accelerating LiDAR Point Cloud Annotation via Sensor Fusion, One-Click Annotation, and Tracking》 [code](https://github.com/bernwang/latte)

> 投影法来传播label进行融合，也涉及到去除地面 ，对点云聚类;
3D point cloud is projected onto the image which is then segmented by Mask R-CNN. The 3D points that are projected onto the masks are highlighted, and the segmented image is displayed.

《A Multisensor Data Fusion Approach for Simultaneous Localization and Mapping》https://github.com/ZhekaiJin/the-Cooper-Mapper/tree/master/L_SLAM （未全部实现）
> 再LOAM基础上使用stereo图像来进行place recogmition 从而re-localization



《Vision-Enhanced Lidar Odometry and Mapping》硕士论文2016 --紧耦合，多相机+lidar
> 后端： tightly couples sparse visual odometry and lidar scanmatching in a single optimization problem. 

> 前端：A new feature tracking pipeline to reject outlier;
A new method is proposed to associate 2D keypoints detected in camera images with 3D po- sition in the lidar point cloud.

> the algorithm is capable of functioning when either the lidar or the camera is blinded;on KITTI, around 1% translation error.

###### Zhang ji papers:


V-LOAM《Visual-lidar odometry and mapping: Low-drift, robust, and fast》ICRA2015（未开源） --紧耦合 无loop closure 无后端
> 在之前基础上，developed a general framework for combining visual odometry (VO) and LiDAR odometry which uses highfrequency visual odometry to estimate the overall ego-motion
while lower-rate LiDAR odometry, which matches scans to
the map and refines the VO estimates. uses images to compute motion prior for scan matching.

> DEMO+LOAM refine,论文版本在KITTI test当时第一 0.75%

《Real-time depth enhanced monocular odometry》IROS2014 (未开源) ++也是DEMO(simple version papaer)++
> associated depth information from LiDAR to visual camera
features, resulting in what can be considered as a RGBD
system with augmented LiDAR depth

DEMO《A real-time method for depth enhanced visual odometry》Auton. Robots2017（未开源） --BA tightly coupled feature-based, no loop colsure, no keyframe 机制
> extend RGB-D visual odometry to large scale, open environments where depth often cannot be sufficiently acquired,uses all three types of features in determining motion(depth avaliable, depth from SFM, no depth), the first optimization based visual odometry method combining both features with and without depth in solving for motion
> no existing method in the literature has associated a lidar to amonocular camera for visual odometry estimation.当时仅次于v-loam，排KITTI第4

###### Z end
《LIMO: Lidar-Monocular Visual Odometry》IROS2018 [LIMO](https://github.com/johannes-graeter/limo) --feature based visual module,tight,optimization(BA), use semantic,没有 loop，ICP closure,ICP步骤，未和其他同类算法详细对比结果
> LiDAR was leveraged for
augmenting depth to visual features by fitting local planes,
which was shown to perform well in autonomous driving
scenarios

> proposing a depth extraction algorithm from LIDAR measurements for camera feature tracks and estimating motion by robustified keyframe based Bundle Adjustment.

> the second best LIDAR-Camera method published and the best performing method that does not use ICP based LIDAR-SLAM as refinement， 次于V-LOAM

《Direct Visual SLAM using Sparse Depth for Camera-LiDAR System》 ICRA2018（未开源） --optimization, tightly coupled   **还是++DVL-SLAM++**
> used the depth from LiDAR in a direct visual
SLAM method, where photometric errors were minimized in
an iterative way

《DVL-SLAM: sparse depth enhanced direct visual-LiDAR SLAM》Auton. Robots2020（[DVL-SLAM](https://github.com/irapkaist/dvl_slam)将开源） --optimization tightly coupled,没有ICP步骤，和DEMO比较结果
> first to combines the sparse depth measurement of LiDAR with the intensity of image and utilizes the direct method

《Lidar-Monocular Visual Odometry using Point and Line Features》2020（未开源） --optimization(BA), tightly coupled 
> structure information: point and line features;fuses the point and line features as landmarks.extract the depth of the points and lines from the lidar data,depth prior is also formulated as prior factors in the point-line bundle adjustment.

> the first efficient lidar-monocular odometry approach using the point and line features together in a purely geometric way，达到KITTI上的SOTA

《Visual-LiDAR SLAM with loop
closure》 2018 硕士论文 （未开源） -紧耦合，但没有联合两类约束 -闭环，关键帧

> 在LOAM之上，增加pose graph optimization和place recognition.位置识别部分使用3种方法来识别位置，基于点云匹配、视觉词典BoW、SegMatch；
讨论了pose graph 受到太多闭环的一定的负面影响

《ViLiVO: Virtual LiDAR-Visual Odometry for an Autonomous Vehicle with a Multi-Camera System》IROS2019（未开源） -关键是也涉及联合优化(BA)

-------
《A Joint Optimization Approach of LiDAR-Camera Fusion for Accurate Dense 3D Reconstructions》 IROS2019 （未开源）

《LiDAR Enhanced Structure-from-Motion》2019 - 和上文一个作者

--------

《A Tight Coupling of Vision-Lidar Measurements for an Effective Odometry》IV2019（未开源）-点云部分基于LOAM（改进voxe map），添加闭环并用pose graph optimization优化点云地图，但文中没有介绍该部分 ;视觉部分基于ORBSLAM2，关键帧局部BA 
> proposed
to use both visual and LiDAR measurements by running in parallel SLAM for each modality and coupling the data. This was done by using both modalities’ residuals during the optimization phase

> builds two maps in different modalities: a lidar voxel map and a visual map with map-points and 2) use them together when to solve the residuals for an odometry

> track部分各自估计frame2fame位姿，然后和各自地图匹配联合两种误差优化当前帧位姿，更新地图（两种地图分开更新），最后使用闭环和位姿图优化（框架图和论文中都没有介绍）

《A Simultaneous Localization and Mapping (SLAM) Framework for 2.5D Map Building Based on Low-Cost LiDAR and Vision Fusion》 Appl. Sci. 2019 -紧耦合，代价函数含scan和image匹配的约束，视觉闭环和关键帧，2.5D地图重定位

> the most tight fusion currently available was proposed in [78], where a graph optimization was performed, using a specific cost function considering both laser and feature constraints.

> track部分各自估计frame2fame位姿,同时检测闭环并保存闭环约束；和2.5D地图匹配构造pose约束ei（ei是联合代价函数，包含闭环约束）;最后pose graph 一起全局优化这些poses；更新地图

《PAVO: A Parallax based Bi-Monocular VO Approach For Autonomous Navigation In Various Environments》DISP2019

《A Review of Visual-LiDAR Fusion based Simultaneous Localization and Mapping》 Sensors2020
> 总结了当下的 visual-lidar slam 的类型，优缺点，提出将来紧耦合的可能的框架，上面三篇文章都被总结进来了。

> 该框架可以说融合了两篇的优点，前面tacking部分相似，得到各自frame to frame 的位姿；之后先对当前帧位姿与现有地图两种landmark匹配(这里没说是否是联合优化代价函数)，进行refine，并更新地图；判断关键帧并检测闭环，最后选择global/local 优化


## lidar+imu

#### loosely coupled
《Loam: Lidar odometry and mapping in real-time.” in Robotics: Science and Systems, vol. 2, 2014.》 纯lidar

《Low-drift and real-time LiDAR odometry and mapping》Autonomous Robots2017 使用了imu

《A Line/Plane Feature-based Lidar Inertial Odometry and Mapping》2019（未开源）
> loam的改进版，imu松耦合，点云投影为range image 在计算法线SRI，增添pose graph 和回环检测  

《INS/GPS/LiDAR integrated navigation system for urban and indoor environments using hybrid scan matching algorithm》2015（未开源）--EKF
> a hybrid lidar odometry algorithm was proposed which
combines the feature-based scan matching method with the ICP-based scan matching method, and
the lidar odometry result was then fused with the IMU in an EKF framework

《LiDAR and Inertial Fusion for Pose Estimation by Non-linear Optimization》2017（未开源）--optimization
> a bunch of IMU
measurements were considered as a relative constraint using the pre-integration theory, and the results
of lidar odometry were fused with IMU measurements in a factor graph framework

《Robust localization and localizability estimation with a rotating laser scanner》ICRA2017（未开源）--Kalman filter

《Efficient Continuous-Time SLAM for 3D Lidar-Based Online Mapping》ICRA2018（未开源） 
> incorporate information from other sensors, such as an inertial measurement unit (IMU) or wheel
odometry, to account for motion of the sensor during acquisition. Furthermore, these motion estimates are used as prior
for the registration

#### Tightly coupled methods
《Tight coupling of laser scanner and inertial measurements for a fully autonomous relative navigation solution》2007（未开源） --Kalman filter

《Long-range gps-denied aerial inertial navigation with lidar localization》IROS2016 (未开源)--error state Kalman filter

《Elastic lidar fusion: Dense map-centric continuous-time slam》 ICRA2018（未开源）--optimization

《LIPS: LiDAR-Inertial 3D Plane SLAM》[LiDAR-Inertial 3D Plane Simulator(LIPS)](https://github.com/rpng/lips) IROS2018 --graph optimizatin
> fuse the inertial pre-integration measurements and plane constraints
from a lidar

《IMU-Aided High-Frequency Lidar Odometry for Autonomous Driving》2019（未开源）--EKF
> aid lidar
odometry in the pre-processing level and the intermediate level output level

《Tightly Coupled 3D Lidar Inertial Odometry and Mapping》（ICRA 2019）[LIO-mapping](https://github.com/hyye/lio-mapping) --graph optimization
> the first open-source implementation for tightly coupled lidar and IMU fusion available to the community.

> 自称SOTA 和LOAM，loam+imu（loosely coupled）比较  是在非公开数据集benchmark测试的

《R-LINS: A Robocentric Lidar-Inertial State Estimator for Robust and Efficient Navigation》2019 （未开源）
> iterated ESKF（iterated error-state Kalman filter） with robocentric formulation

#### 待分类

《MC2SLAM: Real-Time Inertial Lidar Odometryusing Two-Scan Motion Compensation》GCPR2018（未开源） --仅次于LOAM的lidar-inertial方法，总排名第5，旋转误差优于IMLS-SLAM

《IN2LAMA: INertial Lidar Localisation And MApping》ICRA2019

《IN2LAAMA: INertial Lidar Localisation Autocalibration And MApping》2019





《LiDAR Inertial Odometry Aided Robust LiDAR Localization System in Changing City Scenes》ICRA2020



## 多传感器SLAM框架

融合方法：pose graph optmization

《A Modular Optimization Framework for
Localization and Mapping》[mola](https://github.com/MOLAorg/mola)
> MOLA在单个系统中结合了多传感器功能和大型地图管理，同时可以由用户完全自定义。包含了各种数据集各传感器数据的读取等常用的模块


《SelectFusion: A Generic Framework to Selectively Learn Multisensory Fusion》

《Stereo Visual Inertial LiDAR Simultaneous Localization and Mapping》 （未开源）

《Laser-visual-inertial odometry and mapping with high robustness and low drift》（未开源）

《A robust and modular multi-sensor fusion approach applied to mav navigation》IROS2013（未开源） --loosely coupled EKF

《Complementary perception for handheld slam》ICRA2018 （未开源） --tightly coupled  optimization

《Laser–visual–inertial odometry and mapping with high robustness and low drift》JFR2018（未开源） --VIO loosely coupled with lidar
> a sequential multi-layer processing pipeline and consists
of three main components: IMU prediction, visual-inertial
odometry, and scan matching refinement. 

《LIC-Fusion: LiDAR-Inertial-Camera Odometry》IROS2019（未开源） --a tightly-coupled multi-sensor fusion  MSCKF
> 实验结果和VIO(msckf)，LOAM相比较，并非在benchmark实验

> a fast, tightly-coupled, singlethread, LiDAR-inertial-camera (LIC) odometry algorithm
with online spatial and temporal multi-sensor calibration within the computationally-efficient multi-state constraint kalman filter (MSCKF) framework

《HeteroFusion: Dense Scene Reconstruction Integrating Multi-sensors》TVCG2019（未开源）

《A robust and modular multi-sensor fusion approach applied to MAV navigation》IROS2013--Extended Kalman Filter (EKF)  [ethzasl_msf](https://github.com/ethz-asl/ethzasl_msf)


# semantic/Object SLAM

[《DSP-SLAM: Object Oriented SLAM with Deep Shape Priors》](https://arxiv.org/abs/2108.09481)3DV 2021 [code](https://github.com/JingwenWang95/DSP-SLAM) 
> 将基于深度学习的shape prior 结合到ORB-SLAM2的前端中，在线得到物体mesh，可以在原稀疏特征点m基础上增加得到含有object model的地图

# deep learning + Geometric SLAM

[《DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras》](https://arxiv.org/abs/2108.10869) NeurIPS 2021 [code](https://github.com/princeton-vl/DROID-SLAM)
> 在RAFT基础上增加基于几何方法的DBA layer，端到端，效果优异


# Contribute

Please feel free to pull requests to add links.

# License

[![CC0](http://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg)](https://creativecommons.org/publicdomain/zero/1.0/)
