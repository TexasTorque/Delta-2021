package org.texastorque.inputs;

public class State {
    public static volatile State instance;

    /**
     * States the robot can be in
     */
    public enum RobotState {
        AUTO, TELEOP, VISION, SHOOTING, MAGLOAD;
    }

    /**
     * The robot's current state
     */
    private RobotState robotState = RobotState.TELEOP;

    /**
     * @return The current robot state
     */
    public RobotState getRobotState() {
        return robotState;
    }

    /**
     * Set the curren state
     * @param state
     */
    public synchronized void setRobotState(RobotState state ){
        this.robotState = state;
    }

    /**
     * Get the State instance
     * @return State
     */
    public static synchronized State getInstance() {
        if(instance == null) {
            instance = new State();
        }
        return instance;
    }
}
