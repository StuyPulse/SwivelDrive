# SwivelDrive

attempt at new kinematics for swerve that enable straight line spin maneuvers without corrections

## Notes

Approach: 
 * use a `Pose2d` to represent the position of the swerve 
 * use a `Translation2d[]` to represent the velocity vector of each module
 * calculate the position of a module by adding its offset, rotated by the pose heading, to the pose translation
 * calculate its position after the `ChassisSpeed` is applied to the module (e.g. apply the rotation and translation to the point)
 * calculate a velocity vector from initial module position and the calculated final position
 * calculate an acceleration vector from the provided velocity for that module and the calculated velocity vector
 * store calculated velocity vector and acceleration in a `SwivelModuleState`
 * return `SwivelModuleState[]`
 
Similar Alternative Approach:
 * calculate the final pose FIRST by applying a `Twist2d`, generated from a `ChassisSpeed * period`
 * calculate the modules relative to the start pose and relative to the final pose
 * calculate the velocity and acceleration vectors in the same way as above the initial and final module positions
 
Problems:
 * this covers inverse kinematics? but how to go forward
 * what is the proper way to do odometry with this method?
 
Better method:
 * direct formula that covers acceleration
 * easier to do forward kinematics because algebra can be applied to the formula
 * [Notes_5_CurvilinearMotion.pdf](https://github.com/StuyPulse/SwivelDrive/files/9464070/Notes_5_CurvilinearMotion.pdf)
