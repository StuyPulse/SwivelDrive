package com.stuypulse.robot.kinematics;

import com.stuypulse.robot.subsystems.SwivelModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.NumericalIntegration;

public class SwivelDriveOdometry {
    
    // private double x;
    // private double y;

    public SwivelDriveOdometry(SwivelDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    private SwivelDriveKinematics kinematics;


    private Pose2d pose = new Pose2d();

    public Pose2d update(Rotation2d gyroAngle, SwivelModuleState[] states, double period) {
        var offsets = kinematics.getModuleOffsets();
        var translation = pose.getTranslation();
        var rotation = pose.getRotation();

        var position = new Translation2d();
        for (int i = 0; i < states.length; ++i) {
            var state = states[i];
            var vel = state.velocity.plus(state.acceleration.times(period));
            var pos = translation.plus(offsets[i].rotateBy(rotation))
                    .plus(vel.times(period));
            
            position = position.plus(pos);
        }

        return pose = new Pose2d(position.div(states.length), gyroAngle);
    }

    public Pose2d getPose() {
        return pose;
    }
}
