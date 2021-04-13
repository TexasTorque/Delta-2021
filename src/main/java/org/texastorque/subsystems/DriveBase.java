package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends Subsystem {
    public static volatile DriveBase instance;
    
    // Cached instances
    private Input input;

    // Create the SparkMax motors
    private TorqueSparkMax DBLeft = new TorqueSparkMax(Ports.DB_LEFT_1);
    private TorqueSparkMax DBRight = new TorqueSparkMax(Ports.DB_RIGHT_1);

    // Variables to hold the current speed
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double speedMult = .55;

    // PIDs
    private ScheduledPID linePid = new ScheduledPID.Builder(0, -1, 1, 1)
        .setPGains(.01)
        .setIGains(.005)
        .setDGains(.000005)
        .build();
    private LowPassFilter lowPassFilter = new LowPassFilter(.5);

    /**
     * Instantiate a new DriveBase
     */
    private DriveBase() {
        input = Input.getInstance();
        DBLeft.addFollower(Ports.DB_LEFT_2);
        DBRight.addFollower(Ports.DB_RIGHT_2);
    }

    /**
     * Reset the encoders when auto is initialized
     */
    @Override
    public void initAuto() {
        resetEncoders();
    }

    /**
     * Reset the speeds when teleop is initialized
     */
    @Override
    public void initTeleop() {
        leftSpeed = 0;
        rightSpeed = 0;
    }
    
    /**
     * Move during telepo
     */
    @Override
    public void runTeleop(RobotState state) {
        double left = input.getDriveBaseInput().getLeftSpeed();
        double right = input.getDriveBaseInput().getRightSpeed();
        
        leftSpeed = ((left * left) * (left < 0 ? -1 : 1)) * speedMult;
        rightSpeed = ((right * right) * (right < 0 ? -1 : 1)) * speedMult;
    
        DBLeft.set(leftSpeed);
        DBRight.set(rightSpeed);
        smartDashboard();
    }
    
    /**
     * Set the zero values for the encoder to the current values
     */
    public void resetEncoders() {
        DBLeft.tareEncoder();
        DBRight.tareEncoder();
    }

    /**
     * @return The left drive distance
     */
    public double getLeftDistance() {
        return -(DBLeft.getPosition());
    }

    /**
     * @return The right drive distance
     */
    public double getRightDistance() {
        return -(DBRight.getPosition());
    }
    
    /**
     * Update values in SmartDashboard
     */
    @Override
    public void smartDashboard() {
        // Speeds
        SmartDashboard.putNumber("[DB]leftSpeed", leftSpeed);
        SmartDashboard.putNumber("[DB]rightSpeed", rightSpeed); 
        // Distance
        SmartDashboard.putNumber("[DB]leftDistance", getLeftDistance()); 
        SmartDashboard.putNumber("[DB]rightDistance", getRightDistance()); 
    }

    /**
     * Get the DriveBase instance
     * @return DriveBase
     */
    public static synchronized DriveBase getInstance() {
        if(instance == null) {
            instance = new DriveBase();
        }
        return instance;
    }
}
