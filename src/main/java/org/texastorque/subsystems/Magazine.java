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
    private double velocityGate; // -1 if open,  0 if closed

    private final double setSpeedLow = -.5;
    private final double setSpeedHigh = .7;
    private final double setSpeedGate = 0;
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
    public void initTeleop(){
        feedback.getMagazineFeedback().resetCount();
    }

    public void runTeleop(RobotState state){
        updateFeedback();
        
        if((state == RobotState.TELEOP || state == RobotState.VISION) && !input.getMagazineInput().needPreShoot()) {
            preShootStarted = false; 
            lowMag = input.getMagazineInput().getMagLow(); // Low mag speed
            highMag = input.getMagazineInput().getMagHigh(); // High mag speed
            velocityGate = input.getMagazineInput().getVelocityGate(); // Current gate state
            
            velocityLow = 0; // Set speed to 0 as default
            velocityHigh = 0; // - -

            // Will have to change for three!!!!
            if(input.getMagazineInput().getAutoMag()) { // if auto mag enabled (by operator)
                velocityHigh = setSpeedHigh; // set speed high to constant
                velocityLow = setSpeedLow; // set speed low to constant
                velocityGate = setSpeedGate;
                
                if(feedback.getMagazineFeedback().getMagHigh() && feedback.getMagazineFeedback().getMagLow() && feedback.getMagazineFeedback().getCount() == 3) { // ball in mag high ; ball in mag low ; ball count is 3
                    beltHigh.set(0); // set belt high speed to 0
                    velocityHigh = 0;
                    beltLow.set(-0.3); // set belt low speed to -.3
                    if(feedback.getMagazineFeedback().getCount() == 3 && feedback.getMagazineFeedback().getMagLow()) { // 3 balls in mag ; ball in low mag
                        beltLow.set(0); // set belt low speed to 0
                    }
                }
            } else {
                velocityHigh = input.getMagazineInput().getMagHigh(); // ball in upper mag
                velocityLow = input.getMagazineInput().getMagLow(); // ball in lower mag
                velocityGate = input.getMagazineInput().getVelocityGate(); // what speed should the gate be at
            }

            if(velocityGate != 0 ) {
                feedback.getMagazineFeedback().resetCount(); // if gate is open, reset the ball count
            }
        } else if((state == RobotState.TELEOP || state == RobotState.VISION) && input.getMagazineInput().needPreShoot()) { // check mode AND if operator wants to pre shoot
            feedback.getMagazineFeedback().resetCount(); // reset ball count
            if(!preShootStarted) { // if the pre shoot has not already been started
                startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp(); // get time
                preShootStarted = true; // indicated start of preshoot
            } else if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime < .25) { // check if pre shoot started less than .25 seconds ago
                velocityGate = -1; // gate open
                velocityHigh = 0; // stop movement of velocity high
                velocityLow = 0; // stop movement of velocity low
            } else {
                velocityGate = -1; // gate open
                velocityHigh = 1;
                velocityLow = -1;
            }
        }
        output(); // sets motors (gate and mag) with selected seeds
    }

    public void runAuto(RobotState state){
        velocityLow = input.getMagazineInput().getMagHigh();
        velocityHigh = input.getMagazineInput().getMagLow();
        velocityGate = input.getMagazineInput().getVelocityGate();
    }
    
    protected void output(){ // sets motors (gate and mag) with selected seeds [executed in runTeleop]
        beltHigh.set(velocityGate * robotMultiplier);
        beltLow.set(velocityLow);
        beltGate.set(velocityHigh * robotMultiplier);
    }

    protected void updateFeedback(){}

    public void disable(){}

    /**
     * Get the Magazine instance
     * @return Magazine
     */
    public static synchronized Magazine getInstance() {
        if(instance == null) {
            instance = new Magazine();
        }
        return instance;
    }
}
