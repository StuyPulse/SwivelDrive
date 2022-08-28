package com.stuypulse.robot.kinematics;

import com.stuypulse.robot.subsystems.SwivelModule;

import edu.wpi.first.math.SimpleMatrixUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;

public class SwivelDriveKinematics {

    private final Translation2d[] moduleOffsets;

    public SwivelDriveKinematics(Translation2d[] moduleOffsets) {
        this.moduleOffsets = moduleOffsets;
    }

    public Translation2d[] getModuleOffsets() {
        return moduleOffsets;
    }

    public SwivelModuleState[] toSwivelModuleStates(Pose2d chassis, Translation2d[] moduleSpeeds, ChassisSpeeds chassisSpeeds, double period) {
        if (moduleOffsets.length != moduleSpeeds.length) {
            throw new IllegalArgumentException("not good");
        }
        
        var pos = chassis.getTranslation();
        var heading = chassis.getRotation();

        SwivelModuleState[] states = new SwivelModuleState[moduleOffsets.length];
        for (int i = 0; i < moduleOffsets.length; ++i) {
            Translation2d moduleLocation = pos.plus(moduleOffsets[i].rotateBy(heading));
            
            double rad = chassisSpeeds.omegaRadiansPerSecond * period;
            double dx = chassisSpeeds.vxMetersPerSecond * period;
            double dy = chassisSpeeds.vyMetersPerSecond * period;

            Translation2d nextModuleLocation = 
                moduleLocation.minus(pos).rotateBy(new Rotation2d(rad))
                    .plus(new Translation2d(dx, dy))
                    .plus(pos);

            var velocity = nextModuleLocation.minus(moduleLocation).div(period);
            var acceleration = velocity.minus(moduleSpeeds[i]).div(period);

            states[i] = new SwivelModuleState(velocity, acceleration);
        }

        return states;
    }
}
