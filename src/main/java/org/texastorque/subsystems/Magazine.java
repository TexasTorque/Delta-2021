package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.AutoMagState;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueVictor;

public class Magazine extends Subsystem {
    private static volatile Magazine instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // Speeds
    private double velocityHigh;
    private double velocityLow;

    private final double robotMultiplier = -1;

    // Values
    private boolean preShootStarted = false;
    private double startTime;

    // Motors
    private TorqueVictor beltHigh = new TorqueVictor(Ports.BELT_HIGH);
    private TorqueVictor beltLow = new TorqueVictor(Ports.BELT_LOW);

    /**
     * AutoMag reset ball count
     * 
     */
    @Override
    public void initTeleop() {
        feedback.getMagazineFeedback().resetAutomag();
    }

    public void runTeleop(RobotState state) {
        updateFeedback();

        if ((state == RobotState.TELEOP || state == RobotState.VISION) && !input.getMagazineInput().shootingNow()) {
            preShootStarted = false;

            if (input.getMagazineInput().getAutoMag()) {
                runAutomag();
            } else {
                velocityLow = input.getMagazineInput().getVelocityLow(); // ball in lower mag
                velocityHigh = input.getMagazineInput().getVelocityHigh(); // ball in upper mag
            }
        } else if ((state == RobotState.TELEOP || state == RobotState.VISION)
                && input.getMagazineInput().shootingNow()) { // check mode AND if operator wants to pre shoot
            runShootingNow();
        }
        output(); // sets motors (gate and mag) with selected seeds
    }

    public void runAutomag() {
        AutoMagState state = feedback.getMagazineFeedback().getState();
        switch (state) {
            case EMPTY:
                // run upper and lower mag
                velocityLow = input.getMagazineInput().getSetSpeedLow();
                velocityHigh = input.getMagazineInput().getSetSpeedHigh();
                break;
            case ONE_PAST_SECOND:
                // if ball clears second sensor stop upper magazine and wait for another ball.
                // We want to move two balls into the upper mag together
                velocityLow = input.getMagazineInput().getSetSpeedLow();
                velocityHigh = 0;
                break;
            case MOVING_TWO_UP:
                // if ball reaches second sensor, move both balls to the top
                velocityLow = input.getMagazineInput().getSetSpeedLow();
                velocityHigh = input.getMagazineInput().getSetSpeedHigh();
                break;
            case UPPER_FULL:
                // Two balls are loaded into upper, only run lower now
                velocityLow = input.getMagazineInput().getSetSpeedLow();
                velocityHigh = 0;
                break;
            case FULL:
                // We're done here ⎛⎝(•ⱅ•)⎠⎞
                velocityLow = 0;
                velocityHigh = 0;
                break;
        }
    }

    public void runShootingNow() {
        feedback.getMagazineFeedback().resetAutomag(); // set auto mag to empty
        if (!preShootStarted) { // if the pre shoot has not already been started
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp(); // get time
            preShootStarted = true; // indicated start of preshoot
        } else if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime < .15) { // check if pre shoot started
                                                                                       // less than .15 seconds ago
            // Run the gate and high mag together first
            velocityHigh = 1;
            velocityLow = 0;
        } else {
            velocityHigh = 1;
            velocityLow = -.9;
        }
    }

    public void runAuto(RobotState state) {
        updateFeedback();

        if (!input.getMagazineInput().shootingNow()) { // Normal operation
            preShootStarted = false;
            velocityLow = input.getMagazineInput().getVelocityLow();
            velocityHigh = input.getMagazineInput().getVelocityHigh();
        } else {
            runShootingNow();
        }

        output();
    }

    protected void output() { // sets motors (gate and mag) with selected seeds [executed in runTeleop]
        beltHigh.set(velocityHigh);
        beltLow.set(velocityLow * robotMultiplier);
    }

    protected void updateFeedback() {
    }

    /**
     * Get the Magazine instance
     * 
     * @return Magazine
     */
    public static synchronized Magazine getInstance() {
        if (instance == null) {
            instance = new Magazine();
        }
        return instance;
    }
}