package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.kinematics.SwivelDriveKinematics;
import com.stuypulse.robot.kinematics.SwivelDriveOdometry;
import com.stuypulse.robot.kinematics.SwivelModuleState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivelDrive extends SubsystemBase {
    private SwivelDriveKinematics kinematics = new SwivelDriveKinematics(new Translation2d[] {
        new Translation2d(+1, -1),
        new Translation2d(+1, +1),
        new Translation2d(-1, +1),
        new Translation2d(-1, -1)
    });

    private SwivelDriveOdometry odometry = new SwivelDriveOdometry(kinematics);

    private SwivelModule[] modules = new SwivelModule[] {
        new SwivelModule(),
        new SwivelModule(),
        new SwivelModule(),
        new SwivelModule()
    };

    private ChassisSpeeds speeds = new ChassisSpeeds();

    private double heading = 0.0;

    private Field2d field = new Field2d();

    public SwivelDrive() {
        SmartDashboard.putData("Field",field);
    }

    public SwivelModuleState[] getModuleStates() {
        SwivelModuleState[] states = new SwivelModuleState[modules.length];
        for (int i = 0; i < states.length; ++i) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public Translation2d[] getModuleSpeeds() {
        Translation2d[] states = new Translation2d[modules.length];
        for (int i = 0; i < states.length; ++i) {
            states[i] = modules[i].getState().velocity;
        }
        return states;
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public Rotation2d getHeading() {
        return new Rotation2d(-heading);
    }
    
    private double x = 0.0, y = 0.0;

    public void periodic() {

        // SwivelModuleState[] states = kinematics.toSwivelModuleStates(odometry.getPose(), getModuleSpeeds(), speeds, 0.02);

        // for (int i = 0; i < states.length; ++i) {
        //     modules[i].setTargetState(states[i]);
        // }

        // odometry.update(getHeading(), getModuleStates(), 0.02);

        x += speeds.vxMetersPerSecond * 0.02;
        y += speeds.vyMetersPerSecond * 0.02;
        heading += speeds.omegaRadiansPerSecond * 0.02;

        field.setRobotPose(new Pose2d(x, y, new Rotation2d(heading)));
    }

}
