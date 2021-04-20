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
    private double speedLow = -.5;
    private double speedHigh = .7;
    private double speedGate = 0;

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
    };

    public void runTeleop(RobotState state){
        updateFeedback();
        
        if((state == RobotState.TELEOP || state == RobotState.VISION)) {
            
        }
    };

    public void runAuto(RobotState state){

    };
    
    protected void output(){

    };

    protected void updateFeedback(){
        
    };

    public void disable(){};

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
