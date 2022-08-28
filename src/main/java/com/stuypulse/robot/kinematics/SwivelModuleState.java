package com.stuypulse.robot.kinematics;

import edu.wpi.first.math.geometry.Translation2d;

public class SwivelModuleState {
    public final Translation2d velocity; // m/s
    public final Translation2d acceleration; // m/s^2

    public SwivelModuleState(Translation2d velocity, Translation2d acceleration) {
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}
