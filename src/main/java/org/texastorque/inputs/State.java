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
     * States of the rotary
     */
    public enum RotaryState {
        UP(0), PRIME(1), DOWN(2);

        private final int id;

        RotaryState(int id) {
            this.id = id;
        }

        public int getValue() {
            return id;
        }
    }

    /**
     * Climber state
     */
    public enum ClimberState {
        RETRACT(-1), NEUTRAL(0), EXTEND(1);

        private final int id;

        ClimberState(int id) {
            this.id = id;
        }

        public int getValue() {
            return id;
        }
    }

    /**
     * Climber side
     */
    public enum ClimberSide {
        LEFT, NEUTRAL, RIGHT;
    }

    /**
     * Hood setpoints
     */
    public enum HoodSetpoint {
        NEUTRAL(0), LAYUP(4), LIMELIGHT(50), LONGSHOT(57), TRENCH(61);

        private final int id;

        HoodSetpoint(int id) {
            this.id = id;
        }

        public int getValue() {
            return id;
        }
    }

    /**
     * Flywheel speeds
     */
    public enum FlywheelSpeed {
        NEUTRAL(0), LIMELIGHT(3900), LAYUP(4250), TRENCH(6000), LONGSHOT(6500);

        private final int id;

        FlywheelSpeed(int id) {
            this.id = id;
        }

        public int getValue() {
            return id;
        }
    }

    /**
     * AutoMag
     */
    public enum AutoMagState {
        EMPTY, ONE_PAST_SECOND, MOVING_TWO_UP, UPPER_FULL, FULL
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
     * Set the current state
     * 
     * @param state
     */
    public synchronized void setRobotState(RobotState state) {
        this.robotState = state;
    }

    /**
     * Get the State instance
     * 
     * @return State
     */
    public static synchronized State getInstance() {
        if (instance == null) {
            instance = new State();
        }
        return instance;
    }
}
