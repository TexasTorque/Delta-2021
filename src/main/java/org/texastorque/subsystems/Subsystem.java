package org.texastorque.subsystems;

import org.texastorque.torquelib.component.TorqueSubsystem;
import org.texastorque.inputs.State.RobotState;

public abstract class Subsystem implements TorqueSubsystem {
    public void run(RobotState state){};
}
