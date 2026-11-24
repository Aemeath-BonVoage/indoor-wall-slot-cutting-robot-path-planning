# Technical Overview

[README](../README.en.md) · [report abstract](report/abstract_en.md) · [figure gallery and glossary](FIGURES.md)

This page summarises the project for readers who do not read Chinese: for each part, the problem, the method, the key result and what was learned. Every number comes from the experiment report cited next to it. The reports are in Chinese; their figures carry English labels and are collected in the [figure gallery](FIGURES.md), which ends with an English–Chinese glossary.

## The task

Electrical conduits in residential walls are laid in slots cut into the wall. The robot is a mecanum omnidirectional base with a Cartesian 4-DoF arm, a 2D LiDAR and a front camera, simulated in ROS 2 Humble and Gazebo Classic 11. The arm covers only about 12 cm along the wall, so a slot is cut from a sequence of base poses called stations. The tool tip sits about 0.69 m ahead of the base centre, so a heading error of 0.5° moves it by about 6 mm ([multi-station execution](experiments/slot_execution/README.md)). The base must therefore be placed to millimetre accuracy, slot routes must satisfy construction slotting rules while keeping execution short, and the simulator has to be trusted before its numbers are.

Gazebo walls are rigid and cannot model material removal, so cutting is a dry cut: the tool tip follows the planned path at a fixed gap in front of the wall. Gazebo ground truth is used for evaluation only. Times are station time (the sum over stations of strafing, settling, residual measurement and cutting, excluding navigation and the first alignment) unless called total task time, which includes navigation and alignment.

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

**Result.** DP partition and sweep order alone cut a two-slot task in Gazebo from 283 s (16 stations) to 250 s (11 stations), −11.8 % ([multi-station execution](experiments/slot_execution/README.md)). On 400 random residential walls, joint planning was never slower than rule-based routes, saved a further 3.5 % of predicted time on partition walls (95 % CI [3.0, 3.9] %, n = 156, point-to-point time model `align_v3`) and was proven optimal on 92 % of the paired walls; planning time rose from a median of 25 ms to 0.40 s. Executed in Gazebo with paired slip seeds (2 + 2 runs), the example task took 228 s with the joint plan against 246 s with rule-based routes: a measured saving of 7.3 % against a predicted 6.6 %, with the same accuracy ([plan validation](experiments/plan_validation/README.md)).

**Learned.** The saving comes almost entirely from fewer arm points; slot length and turn count barely change. A time model holds only for the execution path it was calibrated on: after the path changed, the previous preset underestimated every run by 18–21 s. The recalibrated preset has a per-station RMSE of 0.22 s on 44 stations and predicts the total station time of 5 held-out runs within 0.7 %.

## 4. Multi-station execution and along-wall accuracy

**Problem.** Every station inherits the wall origin fixed by fine alignment, and between stations the base strafes along a flat wall, which gives a 2D LiDAR distance and heading but no along-wall information.

**Method.** At each station the robot strafes in the wall frame, holds the base with a position loop, measures the residual pose error, corrects the arm points and cuts. The wall origin is the marker position estimated by the EKF, not the pose where the base happened to park. For realistic odometry three remedies were compared: a multi-marker map of AprilTags with known along-wall positions, re-alignment at the reference marker after a set travel distance, and online estimation of the strafe-scale correction as a fourth EKF state.

**Result.** With ground-truth odometry, the constant lateral offset of each run fell from +1.26 / +0.65 mm to +0.24 / −0.18 mm (+0.86 mm in the ablation without the origin fold-in), with a maximum tool-tip path error of 0.93 mm ([multi-station execution](experiments/slot_execution/README.md)). With the encoder error model, a single marker and odometry-only strafing, the arrival error grew in proportion to the distance from the marker, up to 24.7 mm at a trend of −2.8 %, matching the model's 3 % lateral slip. Markers every 0.6 m gave 2.8 mm and every 1.2 m gave 11.9 mm, because the spacing must stay within the camera's visible window of about 0.83 m. Re-alignment did not help against an error of fixed direction (17.5 mm, and 48 s longer). The scale estimate converged to −2.7 to −3.0 % (σ ≈ 0.3 %). The default combines a 0.6 m map with scale estimation: maximum along-wall error 1.5 mm (mean 0.7 mm), 95th-percentile tool-tip path error 0.23 mm, total task time 295 s against 272 s. A ±3 mm error in the recorded marker positions passes into the result almost one to one ([along-wall accuracy](experiments/along_wall_accuracy/README.md)).

**Learned.** The earlier sub-millimetre multi-station accuracy rested on odometry that equalled ground truth. Along-wall position is observable only while a marker is in view, and the error between markers was a systematic scale error, so the remedy was to estimate that scale, not to re-align more often.

## 5. System layer

**Problem.** The task flow was one sequential program of about 1400 lines, navigation used NavFn + DWB, `/odom` was Gazebo ground truth, and the base bypassed ros2_control.

**Method.** The steps became 6 skill actions sequenced by a BehaviorTree.CPP executor structured like the Nav2 `bt_navigator`, with retries and re-alignment as recovery branches. Nav2 uses the Smac 2D planner and an omnidirectional MPPI controller. The base runs under ros2_control with a custom mecanum controller and a simulated hardware interface whose encoders follow an error model (per-wheel scale errors, random slip, 3 % lateral and 2 % yaw slip); a `robot_localization` EKF fuses wheel odometry with a simulated gyro.

**Result.** Behaviour-tree execution of the 11-station plan matched the sequential program: maximum tool-tip path error 1.36 mm, 232.5 s against 233.2–233.5 s. Over 3 laps of 6 goals, Smac 2D + MPPI reached 18 / 18 goals against 15 / 18 for NavFn + DWB, with a minimum clearance of 183 mm against 5 mm and 9.7 % against 45.5 % of one core in the controller. Fusing the gyro reduced odometry drift over a 129 m route from 1.4–1.6 % to 0.14–0.26 % ([system-layer modernisation](experiments/system_modernization/README.md)).

**Learned.** Local loops and navigation need different odometry. The fused output under-reported slow motion (over 8 s at −0.7 mm/s, ground truth moved −5.5 mm and the EKF output −0.2 mm), which made the station hold drift; the skills now close their loops on raw wheel odometry. The cause was found later (section 10).

## 6. Simulation fidelity

**Problem.** In a controlled test the base turned at only 18–37 % of the commanded yaw rate while translation was correct.

**Method and result.** Several hypotheses were tested and rejected, and one data set was discarded after it turned out the robot had touched a wall. Two interacting causes remained: position-interface arm joints that are moved to their targets instantly every cycle, and a planar-move plugin that assigns one velocity to every link. With force-controlled joints and a custom force-based base plugin the ratio is 0.99–1.01 in every arm pose; this is simulation configuration v2 ([project evolution, stage 4](EVOLUTION.md#4-仿真保真度底盘漂移与旋转失真)). Later, part of the base rotation in Gazebo was found to appear only in the pose and not in link velocities, so a velocity loop, the encoder model and Gazebo's IMU all missed it. Deriving velocity from pose differences cut the EKF heading error after a 60 s test sequence from 4.5–7.1° to 0.26° ([system-layer modernisation](experiments/system_modernization/README.md)).

**Learned.** The simulator can itself be the source of error: when a metric looks wrong, first check under controlled conditions that the simulated robot moves as commanded. Contaminated data are discarded, rejected hypotheses are recorded, and after a change of simulation configuration earlier data are labelled and dependent models recalibrated.

## 7. Arm trajectories and cutting force

**Problem.** Each arm point took a fixed 2 s regardless of distance, so three quarters of the arm time was spent on non-cutting moves; and the simulation applied no cutting force, so the station hold had only been tested against the arm's own reaction.

**Method.** The arm points of a station are sent as one trajectory to the `joint_trajectory_controller`. Every straight segment is a rest-to-rest S-curve; cutting and plunge segments run at the feed rate and the other moves at per-axis rapid limits, with a short dwell after the plunge. The time model gained a term proportional to cut length. Separately, a cutting-force model based on specific cutting energy was converted into a wrench on the base, the friction the platform needs was computed, and proportional, PI, PID and disturbance-observer hold laws were compared on the sampled loop and in an offline simulation ([continuous arm trajectories](experiments/arm_trajectory/README.md), [cutting force and hold](experiments/cutting_force/README.md)).

**Result.** In Gazebo the 11 stations took 160 s instead of 228 s (−30 %) with the same mean tool-tip path error; the new time-model preset predicted a held-out plan within 1.0 s. Because an extra segment now costs 2.0 s instead of 8.1 s, the gain of joint planning over rule-based routes shrank from 3.5 % to 1.2 % on partition walls. Offline, vertical cuts load the base about ten times less than horizontal cuts; a horizontal cut at full spindle power needs a friction coefficient of about 0.49. The proportional hold yields 4.7 mm at the tool tip per newton of lateral force, and a disturbance observer with feedforward keeps the peak deviation for a horizontal cut in brick at 5 mm/s to 1.6 mm.

**Learned.** A planning gain depends on the cost structure of execution: speeding up the arm removed most of what joint planning had been saving. The feed rate cannot be set from a dry-cut simulation: the default 0.05 m/s is above what the force model allows in brick at the tool's 820 W (about 16 mm/s); section 8 derives the limit. The force values are mostly assumptions.

## 8. Cutting force in Gazebo and a force-limited feed

**Problem.** The hold laws and the force plugin had only been analysed offline, and the default feed of 0.05 m/s had no basis in the force model.

**Method.** Constant wrenches were first applied to the base in Gazebo to check the plant model, then a whole wall (brick, 11 stations, 20 cuts) was executed through the behaviour tree with the cutting-force plugin active, for the proportional hold, the disturbance observer, and the observer with 70 % feedforward. The feed limit was derived from three constraints (spindle power, wheel–floor friction, the yaw moment the hold can cancel), the time model was recalibrated at that feed, and the 400-wall planning comparison was repeated ([report](experiments/cutting_force/README.md)).

**Result.** Steady-state deviations matched the model within 1.3 % and step peaks within 6 % in 12 of 14 tests, after two defects were fixed: the hold loop was triggered in bursts by a 10 Hz simulation clock, and the plugin measured feed relative to the base instead of the wall. Under cutting force the proportional hold lets the tool tip move 81–85 mm and covers 40 % of the slot length; the observer holds 8.3–9.0 mm and 5.0–6.2 mm with feedforward (1.2 mm RMS, n = 4), about twice the offline prediction. In brick the feed is limited to 5.5 mm/s by friction (μ = 0.3, safety factor 1.5), below the 9.1 mm/s allowed by 67 % of spindle power. At that feed the example wall takes 978.5 s instead of 160 s, an arm point costs 5.1 s instead of 0.5 s, and joint planning saves 22.6 s (2.3 %) on partition walls instead of 2.3 s (1.2 %).

**Learned.** An offline model that treats the cutting force as a given function of time misses its coupling with base motion, which is where the factor of two comes from. The platform, not the tool, limits the feed. Once the feed is force-limited, execution time is governed by cut length and the number of plunges, so what planning can still reduce is the number of arm points, not the number of stations. No default was changed; the observer's comparison on the dry-cut task followed in section 10.

## 9. From one wall to a room

**Problem.** The joint planner returned "no plan" on 106 of 400 random walls, and the time model covered a single wall, without navigation between walls.

**Method.** Each infeasible wall is diagnosed (smallest set of slots to remove, minimal conflicting set, which single rule relaxation would make it feasible), and alternatives that keep every slotting rule are ranked by how much they change the installation: another feed point, routing through the floor or ceiling, moving a box. For a room, each wall gets its plan and two sweep directions; the order and directions are solved exactly by dynamic programming over subsets with a transit-time model ([report](experiments/room_planning/README.md)).

**Result.** The conflict is always one pair of slots. 102 of the 106 walls get a compliant plan: 70 by changing a feed point and 32 by routing one to three slots through the floor. On 200 random rooms the optimal order is 1.4 % faster than a fixed order and 0.4 % faster than nearest neighbour; work on the walls is 81 % of predicted time and the first alignment at each wall costs more than navigation. These are model predictions; the installer cost rates are assumptions.

**Learned.** Most "infeasible" walls came from an unreasonable task assignment (two crossing feeds), not from the rules. Visiting order matters little inside one room; the fixed overhead per wall is the better target.

## 10. Execution-layer issues and marker-map estimation

**Problem.** Four unexplained behaviours remained in the run logs (a stall before navigation arrival, a ~2.7 s pause while cutting, rare Nav2 start-up failures, an EKF that under-reported slow motion), and along-wall accuracy still depended on markers being placed to within a millimetre.

**Method.** Each issue was first searched for in existing logs and bags, then reproduced or instrumented with a minimal experiment, and fixed with a before/after comparison when the cause was in this repository ([report](experiments/execution_issues/README.md)). Separately, marker positions were added to the wall EKF state (EKF-SLAM along the wall) and evaluated offline with a measurement simulator whose noise model was fitted to recorded Gazebo data and checked against 10 existing configurations ([report](experiments/tag_map_estimation/README.md)).

**Result.** The stall came from a 20 ms goal-acknowledgement timeout measured in simulation time that advances in 0.1 s steps; the EKF bias came from publishing an instantaneous twist that was inconsistent with the integrated pose. Both are fixed. The pause is a late action response whose delay follows the DDS heartbeat period; why the sample is lost is unknown. The start-up failure did not recur in 30 cold starts. On the dry-cut task the disturbance-observer hold costs 0.3 % in time and lowers the maximum path error in every pair. Offline, online map estimation keeps the maximum along-wall error near 1.1–1.2 mm for placement errors up to 5 mm, and the residual bias without scale estimation is explained by strafe-scale error leaking through heading.

**Learned.** A simulation clock published at 10 Hz interacts with anything that measures short timeouts in simulation time; three separate defects in this project trace back to it. The stall fix rests on log evidence: the stall itself was too rare (about 1 %) to reproduce in a controlled comparison. The marker-map estimator has not been run in Gazebo.

## Limitations

All results come from simulation, with idealised rendering, an encoder error model that is assumed and not calibrated on hardware, a force-driven base whose wheels do not carry the load, and a cutting-force model whose values are mostly assumptions (the default execution path is still a dry cut). Samples are small: 8 trials per alignment condition, 1–3 runs per system-layer configuration and 2 runs per along-wall or plan-validation configuration. The gains over 400 walls are model predictions; one wall and one task were executed in Gazebo, and room-level plans have not been executed.
