package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;

public class Magazine extends Subsystem {
    private static volatile Magazine instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // Speeds
    private double velocityHigh;
    private double velocityLow;
    private double velocityGate; // -1 if open, 0 if closed

    private final double robotMultiplier = -1;

    // Values
    private boolean preShootStarted = false;
    private double startTime;
    private int lowMag;
    private int highMag;

    // Motors
    private TorqueSparkMax beltHigh = new TorqueSparkMax(Ports.BELT_HIGH);
    private TorqueSparkMax beltLow = new TorqueSparkMax(Ports.BELT_LOW);
    private TorqueSparkMax beltGate = new TorqueSparkMax(Ports.BELT_GATE);

    /**
     * AutoMag reset ball count
     * 
     */
    @Override
    public void initTeleop() {
        feedback.getMagazineFeedback().resetCount();
    }

    public void runTeleop(RobotState state) {
        updateFeedback();

        if ((state == RobotState.TELEOP || state == RobotState.VISION) && !input.getMagazineInput().shootingNow()) {
            preShootStarted = false;
            lowMag = input.getMagazineInput().getMagLow(); // Low mag
            highMag = input.getMagazineInput().getMagHigh(); // High mag

            velocityLow = input.getMagazineInput().getVelocityLow(); // ball in lower mag
            velocityHigh = input.getMagazineInput().getVelocityHigh(); // ball in upper mag
            velocityGate = input.getMagazineInput().getVelocityGate(); // what speed should the gate be at

            if (velocityGate != 0) {
                feedback.getMagazineFeedback().resetCount(); // if gate is open, reset the ball count
            }
        } else if ((state == RobotState.TELEOP || state == RobotState.VISION)
                && input.getMagazineInput().shootingNow()) { // check mode AND if operator wants to pre shoot
            runShootingNow();
        }
        output(); // sets motors (gate and mag) with selected seeds
    }

    public void runShootingNow() {
        feedback.getMagazineFeedback().resetCount(); // reset ball count
        if (!preShootStarted) { // if the pre shoot has not already been started
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp(); // get time
            preShootStarted = true; // indicated start of preshoot
        } else if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime < .15) { // check if pre shoot started
                                                                                       // less than .15 seconds ago
            // Run the gate and high mag together first
            velocityGate = -1;
            velocityHigh = 1;
            velocityLow = 0;
        } else {
            velocityGate = -1; // gate open
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
            velocityGate = input.getMagazineInput().getVelocityGate();
        } else {
            runShootingNow();
        }

        output();
    }

    protected void output() { // sets motors (gate and mag) with selected seeds [executed in runTeleop]
        beltHigh.set(velocityHigh * robotMultiplier);
        beltLow.set(velocityLow);
        beltGate.set(velocityGate * robotMultiplier);
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