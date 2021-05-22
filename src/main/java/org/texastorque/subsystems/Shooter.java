package org.texastorque.subsystems;

import com.revrobotics.ControlType;

import org.texastorque.constants.Constants;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.HoodSetpoint;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.component.TorqueTalon;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.util.KPID;

import edu.wpi.first.wpilibj.PIDOutput;

public class Shooter extends Subsystem {
    private static volatile Shooter instance;
    
    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // PIDs
    private ScheduledPID shooterPID;
    private KPID hoodKPID = new KPID(0.1, 0, 0, 0, -1, 1);
    private KPID neoKPID = new KPID(0, 0, 0, 0, 0, 1); 

    // Variables
    private double flywheelSpeed;
    private double pidOuput = 0;

    // Motors
    private TorqueTalon flywheel = new TorqueTalon(Ports.FLYWHEEL_LEAD);
    private TorqueSparkMax hood = new TorqueSparkMax(Ports.SHOOTER_HOOD);

    private Shooter() {
        flywheel.configurePID(neoKPID); // add PID to flywheel
        flywheel.addFollower(Ports.FLYWHEEL_FOLLOW);
        flywheel.invertFollower();

        hood.configurePID(hoodKPID); // add PID to hood
        hood.tareEncoder();


        shooterPID = new ScheduledPID.Builder(0, -1, 1, 1)
            .setPGains(0.028)
            .setIGains(0.0008)
            .setDGains(0)
            .setFGains(0.00385)
            .build();
    }

    public void runTeleop(RobotState state){
        flywheelSpeed = input.getShooterInput().getFlywheelSpeed();
        shooterPID.changeSetpoint(flywheelSpeed * Constants.RPM_CONVERSION_SPARKMAX); // change target speed of flywheel to requested speed
        pidOuput = shooterPID.calculate(feedback.getShooterFeedback().getShooterVelocity());
        output();
    };
    
    public void runAuto(RobotState state){
        output();
    };
    
    protected void output(){
        hood.set(input.getShooterInput().getHoodSetpoint().getValue(), ControlType.kPosition); // set hood
        if (input.getShooterInput().getPercentOutputType()) { // if percent output type, 
            flywheel.set(input.getShooterInput().getFlywheelPercent()); // just set flywheel speed to percent (-1 to 1)
        } else {
            if(pidOuput > 0) {
                flywheel.set(0); // if the PID is sending junk, (>0), ignore
            }
            else flywheel.set(-pidOuput); // set to opposite of pid output
        }
    };

    protected void updateFeedback(){
        feedback.getShooterFeedback().setHoodPosition(hood.getPosition());
        feedback.getShooterFeedback().setShooterVelocity(flywheel.getVelocity());
    };

    /**
     * Get the Shooter instance
     * @return Shooter
     */
    public static synchronized Shooter getInstance() {
        if(instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
}
