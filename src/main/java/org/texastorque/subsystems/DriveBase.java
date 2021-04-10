package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends Subsystem {
    public static volatile DriveBase instance;

    // Create the SparkMax motors
    private TorqueSparkMax DBLeft = new TorqueSparkMax(Ports.DB_LEFT_1);
    private TorqueSparkMax DBRight = new TorqueSparkMax(Ports.DB_RIGHT_1);

    // Variables to hold the current speed
    private double leftSpeed = 0;
    private double rightSpeed = 0;

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
        DBLeft.addFollower(Ports.DB_LEFT_2);
        DBRight.addFollower(Ports.DB_RIGHT_2);
    }

    /**
     * Reset the encoders when auto is initialized
     */
    @Override
    public void autoInit() {
        resetEncoders();
    }

    /**
     * Reset the speeds when teleop is initialized
     */
    @Override
    public void teleopInit() {
        leftSpeed = 0;
        rightSpeed = 0;
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
        SmartDashboard.putNumber("leftSpeed", leftSpeed);
        SmartDashboard.putNumber("rightSpeed", rightSpeed); 
        // Distance
        SmartDashboard.putNumber("leftDistance", getLeftDistance()); 
        SmartDashboard.putNumber("rightDistance", getRightDistance()); 
    }


    // Unusued methods
    @Override
    public void disabledInit() {}
    @Override
    public void disabledContinuous() {}
    @Override
    public void autoContinuous() {}
    @Override
    public void teleopContinuous() {}

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
