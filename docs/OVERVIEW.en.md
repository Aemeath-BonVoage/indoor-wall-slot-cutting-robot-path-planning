# Technical Overview

[README](../README.en.md) · [report abstract](report/abstract_en.md) · [figure gallery and glossary](FIGURES.md)

This page summarises the project for readers who do not read Chinese: for each part, the problem, the method, the key result and what was learned. Every number comes from the experiment report cited next to it. The reports are in Chinese; their figures carry English labels and are collected in the [figure gallery](FIGURES.md), which ends with an English–Chinese glossary.

## The task

Electrical conduits in residential walls are laid in slots cut into the wall. The robot is a mecanum omnidirectional base with a Cartesian 4-DoF arm, a 2D LiDAR and a front camera, simulated in ROS 2 Humble and Gazebo Classic 11. The arm covers only about 12 cm along the wall, so a slot is cut from a sequence of base poses called stations. The tool tip sits about 0.69 m ahead of the base centre, so a heading error of 0.5° moves it by about 6 mm ([multi-station execution](experiments/slot_execution/README.md)). The base must therefore be placed to millimetre accuracy, slot routes must satisfy construction slotting rules while keeping execution short, and the simulator has to be trusted before its numbers are.

Gazebo walls are rigid and cannot model material removal, so cutting is a dry cut: the tool tip follows the planned path at a fixed gap in front of the wall. Gazebo ground truth is used for evaluation only.

## 1. Fine alignment in front of the wall

**Problem.** The first version (colour-blob marker, position-based visual servoing with the LiDAR) stopped at about 2 mm. An offline comparison of PBVS, IBVS and a 2.5D hybrid over about 12,500 trials showed that this residual came from the stop criterion, not from perception, and ruled out pure IBVS: with a 5 cm marker at 0.55 m the Jacobian condition number is 314 ([visual-servoing comparison](experiments/ibvs_vs_pbvs/README.md)). A blob also has no identity, so partial occlusion shifts its centroid silently.

**Method.** Detection, estimation, control and the arrival test are separated, following Nav2 `opennav_docking`. An AprilTag is detected with edge-fitted sub-pixel corners and IPPE PnP. An EKF over the wall-relative state (distance, along-wall position, heading) predicts with exact SE(2) odometry increments, compensates measurement latency and gates updates with a χ² test; the LiDAR wall fit supplies distance and heading, the marker the along-wall position. The control law is a square-root-shaped saturated proportional law: with V = ½|e|² it is globally asymptotically stable for the integrator plant, and a discrete-time pole analysis with sampling, velocity-loop lag and a one-step delay gives a stability limit of k·T ≈ 0.6, about three times the value used at the actual 100 ms period. Arrival requires small error, a base at rest and a small estimate covariance for 10 consecutive cycles. Static errors are removed by a taught zero offset ([design and analysis](experiments/fine_alignment/README.md)).

**Result.** In paired Gazebo trials (n = 8 per condition) the position error at arrival fell from 4.16 ± 0.94 mm to 0.53 ± 0.21 mm at the same duration, and stayed at 0.5–0.8 mm under odometry drift, strong light, cool light and intermittent occlusion. Under cool light the colour-blob pipeline lost the marker in all 8 trials; under static occlusion it converged with an 8.6 mm error and no warning, whereas the new pipeline did not move and reported failure. A 1° camera mounting error added about 10 mm of along-wall error to both pipelines, and the taught zero offset reduced it to 1.04 mm. No trial touched the wall ([robustness experiments](experiments/alignment_robustness/README.md)).

**Learned.** Accuracy was set by when the robot stopped. Keeping the blob detector and replacing only estimation, control law and arrival test already gave 0.49 mm in nominal conditions; the detector matters under lighting changes and occlusion, where a marker with a checkable identity fails safely.

## 2. Slot planning under slotting rules

**Problem.** A route must avoid obstacles and obey construction rules: no horizontal slots in load-bearing walls, horizontal runs of at most 50 cm in partition walls, a conduit bending radius of at least six pipe diameters, and clearances around openings, beams and columns. The first-version planner, a grid A\* in Python, could only inflate obstacles.

**Method.** A C++ A\* takes whole straight segments as edges, so limits on segment length and bend spacing are checked when an edge is generated. The state (x, y, last direction) is enough to keep optimality, because earlier segment lengths no longer matter after a turn.

**Result.** On 140 random walls the planner returned no violating route, while the first-version planner violated the rules on 106. On 6 load-bearing walls it reported that no compliant route exists, matching an independent geometric check. Its cost agreed with an independent cell-by-cell Dijkstra on 400 random grids with random rule sets ([slot planning](experiments/cpp_planner/README.md)).

**Learned.** Rules about whole segments can be hard constraints without enlarging the state if the search steps by segments, and "no compliant route" is a more useful answer than a detour that breaks a rule. The 100 % violation rate of the first-version planner on partition walls partly reflects how the tasks were generated.

## 3. Task-level joint planning with lower-bound certificates

**Problem.** Choosing routes first and stations afterwards ignores that two routes of equal length and turn count can need different numbers of stations and arm points.

**Method.** Execution time is predicted by a model calibrated on Gazebo runs: time per station = settling time + strafe distance / strafe speed + arm points × time per point. Stations are partitioned by dynamic programming and visited in a sweep order that is provably shortest on a line. Joint planning makes the route search aware of the station windows, so that corners land on window boundaries, and keeps the rule-based routes as a candidate, so the result cannot be worse. A lower bound is computed for every wall; a plan whose predicted time equals it is proven optimal for that time model. This replaced a planned MILP comparison, for which no solver was available, and applies to every wall instead of only small ones ([joint planning](experiments/task_planning/README.md)).

**Result.** DP partition and sweep order alone cut a two-slot task in Gazebo from 283 s (16 stations) to 250 s (11 stations), −11.8 % ([multi-station execution](experiments/slot_execution/README.md)). On 400 random residential walls, joint planning was never slower than rule-based routes, saved a further 3.5 % of predicted time on partition walls (95 % CI [3.0, 3.9] %, n = 156) and was proven optimal on 92 % of the paired walls; planning time rose from a median of 25 ms to 0.40 s. Executed in Gazebo with paired slip seeds (2 + 2 runs), the example task took 228 s with the joint plan against 246 s with rule-based routes: a measured saving of 7.3 % against a predicted 6.6 %, with the same accuracy ([plan validation](experiments/plan_validation/README.md)).

**Learned.** The saving comes almost entirely from fewer arm points; slot length and turn count barely change. A time model holds only for the execution path it was calibrated on: after the path changed, the previous preset underestimated every run by 18–21 s. The recalibrated preset has a per-station RMSE of 0.22 s on 44 stations and predicts the total station time of 5 held-out runs within 0.7 %.

## 4. Multi-station execution and along-wall accuracy

**Problem.** Every station inherits the wall origin fixed by fine alignment, and between stations the base strafes along a flat wall, which gives a 2D LiDAR distance and heading but no along-wall information.

**Method.** At each station the robot strafes in the wall frame, holds the base with a position loop, measures the residual pose error, corrects the arm points and cuts. The wall origin is the marker position estimated by the EKF, not the pose where the base happened to park. For realistic odometry three remedies were compared: a multi-marker map of AprilTags with known along-wall positions, re-alignment at the reference marker after a set travel distance, and online estimation of the strafe-scale correction as a fourth EKF state.

**Result.** With ground-truth odometry, the constant lateral offset of each run fell from +1.26 / +0.65 mm to +0.24 / −0.18 mm (+0.86 mm in the ablation without the origin fold-in), with a maximum tool-tip path error of 0.93 mm ([multi-station execution](experiments/slot_execution/README.md)). With the encoder error model, a single marker and odometry-only strafing, the arrival error grew in proportion to the distance from the marker, up to 24.7 mm at a trend of −2.8 %, matching the model's 3 % lateral slip. Markers every 0.6 m gave 2.8 mm and every 1.2 m gave 11.9 mm, because the spacing must stay within the camera's visible window of about 0.83 m. Re-alignment did not help against an error of fixed direction (17.5 mm, and 48 s longer). The scale estimate converged to −2.7 to −3.0 % (σ ≈ 0.3 %). The default combines a 0.6 m map with scale estimation: maximum along-wall error 1.5 mm (mean 0.7 mm), 95th-percentile tool-tip path error 0.23 mm, total time 295 s against 272 s. A ±3 mm error in the recorded marker positions passes into the result almost one to one ([along-wall accuracy](experiments/along_wall_accuracy/README.md)).

**Learned.** The earlier sub-millimetre multi-station accuracy rested on odometry that equalled ground truth. Along-wall position is observable only while a marker is in view, and the error between markers was a systematic scale error, so the remedy was to estimate that scale, not to re-align more often.

## 5. System layer

**Problem.** The task flow was one sequential program of about 1400 lines, navigation used NavFn + DWB, `/odom` was Gazebo ground truth, and the base bypassed ros2_control.

**Method.** The steps became 6 skill actions sequenced by a BehaviorTree.CPP executor structured like the Nav2 `bt_navigator`, with retries and re-alignment as recovery branches. Nav2 uses the Smac 2D planner and an omnidirectional MPPI controller. The base runs under ros2_control with a custom mecanum controller and a simulated hardware interface whose encoders follow an error model (per-wheel scale errors, random slip, 3 % lateral and 2 % yaw slip); a `robot_localization` EKF fuses wheel odometry with a simulated gyro.

**Result.** Behaviour-tree execution of the 11-station plan matched the sequential program: maximum tool-tip path error 1.36 mm, 232.5 s against 233.2–233.5 s. Over 3 laps of 6 goals, Smac 2D + MPPI reached 18 / 18 goals against 15 / 18 for NavFn + DWB, with a minimum clearance of 183 mm against 5 mm and 9.7 % against 45.5 % of one core in the controller. Fusing the gyro reduced odometry drift over a 129 m route from 1.4–1.6 % to 0.14–0.26 % ([system-layer modernisation](experiments/system_modernization/README.md)).

**Learned.** Local loops and navigation need different odometry. The fused output under-reported slow motion (over 8 s at −0.7 mm/s, ground truth moved −5.5 mm and the EKF output −0.2 mm), which made the station hold drift; the skills now close their loops on raw wheel odometry.

## 6. Simulation fidelity

**Problem.** In a controlled test the base turned at only 18–37 % of the commanded yaw rate while translation was correct.

**Method and result.** Several hypotheses were tested and rejected, and one data set was discarded after it turned out the robot had touched a wall. Two interacting causes remained: position-interface arm joints that are moved to their targets instantly every cycle, and a planar-move plugin that assigns one velocity to every link. With force-controlled joints and a custom force-based base plugin the ratio is 0.99–1.01 in every arm pose; this is simulation configuration v2 ([project evolution, stage 4](EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真)). Later, part of the base rotation in Gazebo was found to appear only in the pose and not in link velocities, so a velocity loop, the encoder model and Gazebo's IMU all missed it. Deriving velocity from pose differences cut the EKF heading error after a 60 s test sequence from 4.5–7.1° to 0.26° ([system-layer modernisation](experiments/system_modernization/README.md)).

**Learned.** The simulator can itself be the source of error: when a metric looks wrong, first check under controlled conditions that the simulated robot moves as commanded. Contaminated data are discarded, rejected hypotheses are recorded, and after a change of simulation configuration earlier data are labelled and dependent models recalibrated.

## Limitations

All results come from simulation, with idealised rendering, an encoder error model that is assumed and not calibrated on hardware, a force-driven base whose wheels do not carry the load, and no cutting forces. Samples are small: 8 trials per alignment condition, 1–3 runs per system-layer configuration and 2 runs per along-wall or plan-validation configuration. The gains over 400 walls are model predictions; one wall and one task were executed in Gazebo.
