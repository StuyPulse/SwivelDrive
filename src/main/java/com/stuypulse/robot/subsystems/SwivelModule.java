package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.kinematics.SwivelModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivelModule extends SubsystemBase {
    private SwivelModuleState targetState;

    public void setTargetState(SwivelModuleState state) {
        this.targetState = state;
    }
}
