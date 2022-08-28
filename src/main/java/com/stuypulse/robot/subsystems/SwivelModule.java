package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.kinematics.SwivelModuleState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivelModule extends SubsystemBase {
    private SwivelModuleState targetState = new SwivelModuleState(new Translation2d(), new Translation2d());

    public void setTargetState(SwivelModuleState state) {
        this.targetState = state;
    }

    public SwivelModuleState getState() {
        return targetState; 
    }
}
