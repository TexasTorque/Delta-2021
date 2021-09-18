package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Input;
import org.texastorque.torquelib.component.TorqueVictor;

public class Magazine extends Subsystem {
    private static volatile Magazine instance;

    // Cached instances
    private Input input = Input.getInstance();

    // Speeds
    private double velocityHigh;
    private double velocityLow;

    // private final double lowerVeloMultiplier = -1; // Lower *1/2
    // ^ Formerly robotMultiplier
    // Values
    private boolean preShootStarted = false;
    private double startTime;

    // Motors
    private TorqueVictor beltHigh = new TorqueVictor(Ports.BELT_HIGH);
    private TorqueVictor beltLow = new TorqueVictor(Ports.BELT_LOW);

    @Override
    public void runTeleop() {
        updateFeedback();

        if (!input.getMagazineInput().shootingNow()) {
            preShootStarted = false;

            velocityLow = input.getMagazineInput().getVelocityLow(); // ball in lower mag
            velocityHigh = input.getMagazineInput().getVelocityHigh(); // ball in upper mag

        } else if (input.getMagazineInput().shootingNow()) { // check mode AND if operator wants to pre shoot
            runShootingNow();
        }
        output(); // sets motors (gate and mag) with selected seeds
    }

    public void runShootingNow() {
        if (!preShootStarted) { // if the pre shoot has not already been started
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp(); // get time
            preShootStarted = true; // indicated start of preshoot
        } else if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime < .45) { // check if pre shoot started
                                                                                       // less than .15 seconds ago
            // Run the gate and high mag together first
            velocityHigh = 1;
            velocityLow = 0;
        } else {
            velocityHigh = 1;
            velocityLow = .9;
        }
    }

    protected void output() { // sets motors (gate and mag) with selected seeds [executed in runTeleop]
        beltHigh.set(velocityHigh);
        beltLow.set(velocityLow);
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