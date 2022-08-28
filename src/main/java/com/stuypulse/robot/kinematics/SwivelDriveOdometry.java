package com.stuypulse.robot.kinematics;

import com.stuypulse.robot.subsystems.SwivelModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwivelDriveOdometry {
    
    // private double x;
    // private double y;

    public SwivelDriveOdometry() {

    }

    private Translation2d ;

    public Pose2d update(Rotation2d gyroAngle, SwivelModuleState[] states, double period) {

        return new Pose2d(new Translation2d(), gyroAngle);
    }
}
