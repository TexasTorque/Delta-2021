package org.texastorque.subsystems;

import org.texastorque.inputs.State.RobotState;

public abstract class Subsystem {
    public void initTeleop() {
    };

    public void initAuto() {
    };

    public void runTeleop(RobotState state) {
    };

    public void runAuto(RobotState state) {
    };

    protected void output() {
    };

    protected void updateFeedback() {
    };

    public void disable() {
    };

    public void smartDashboard() {
    };
}
