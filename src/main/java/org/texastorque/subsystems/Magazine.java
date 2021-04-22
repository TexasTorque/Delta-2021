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
    private double speedHigh;
    private double speedLow;
    private double speedGate;

    private final double setSpeedLow = -.5;
    private final double setSpeedHigh = .7;
    private final double setSpeedGate = 0;

    // Values
    private boolean preShootStarted = false;
    private int lowMag;
    private int highMag;

    // Motors
    private TorqueSparkMax beltHigh = new TorqueSparkMax(Ports.BELT_HIGH);
    private TorqueSparkMax beltLow = new TorqueSparkMax(Ports.BELT_LOW);
    private TorqueSparkMax beltGate = new TorqueSparkMax(Ports.BELT_GATE);


    /**
     * Reset the ball count
     */
    @Override
    public void initTeleop(){
        feedback.getMagazineFeedback().resetCount();
    }

    public void runTeleop(RobotState state){
        updateFeedback();
        
        if((state == RobotState.TELEOP || state == RobotState.VISION) && !input.getMagazineInput().needPreShoot()) {
            preShootStarted = false;
            lowMag = input.getMagazineInput().getMagLow();
            highMag = input.getMagazineInput().getMagHigh();
            speedGate = input.getMagazineInput().getVelocityGate();

            speedLow = 0;
            speedHigh = 0;

            if(input.getMagazineInput().getAutoMag()) {
                speedHigh = setSpeedHigh;
                speedLow = setSpeedLow;
                speedGate = setSpeedGate;
                
                // if((feedback.getMagazineFeedback().getMagHigh()))
            } else {
                speedHigh = input.getMagazineInput().getMagHigh();
                speedLow = input.getMagazineInput().getMagLow();
                speedGate = input.getMagazineInput().getVelocityGate();
            }
        }
    }

    public void runAuto(RobotState state){

    }
    
    protected void output(){

    }

    protected void updateFeedback(){
        
    }

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
