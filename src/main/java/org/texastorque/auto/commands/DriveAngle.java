package org.texastorque.auto.commands;

import java.sql.DriverManager;

import org.texastorque.auto.Command;
import org.texastorque.inputs.Feedback;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAngle extends Command{ 

    private double targetAngle;
    private double radius;
    
    private double leftDistance;
    private double rightDistance;

    private boolean direction;

    private double speedBase;
    
    private double diameter = 27.5/12;

    private double leftRadius;
    private double rightRadius;

    private double leftSpeed;
    private double rightSpeed;


    /**
     * turns robot in a curve of designted radius and direction
     * @param delay
     * @param targetAngle
     * @param radius
     * @param right
     */
    public DriveAngle(double delay, double targetAngle, double radius, boolean right) {
        super(delay);
        this.targetAngle = targetAngle;
        this.radius = radius;
        this.direction = right;
        this.speedBase = .3; // .3 is better no matter what anyone tells you
    }
    
    /**
     * turns robot in a curve of designated radius, direction, and speed
     * @param delay
     * @param targetAngle
     * @param radius
     * @param right
     * @param speed
     */
    public DriveAngle(double delay, double targetAngle, double radius, boolean right, double speed) {
        super(delay);
        this.targetAngle = targetAngle;
        this.radius = radius;
        this.direction = right;
        this.speedBase = speed;
    }

    @Override
    protected void init() {
        feedback.getDriveTrainFeedback().resetEncoders();

        // If going in a direction, assign radius to the variable
        leftRadius = direction ? radius + diameter : radius;
        rightRadius = direction ? radius : radius + diameter;

        leftDistance = (2 * Math.PI * (leftRadius)) * (targetAngle/360);
        rightDistance = (2 * Math.PI * (rightRadius)) * (targetAngle/360);

        leftSpeed = direction ? speedBase : speedBase * (radius/(radius+diameter));
        rightSpeed = direction ? speedBase * (radius/(radius + diameter)) : speedBase;

        SmartDashboard.putNumber("[DriveAngle]leftSpeedAngle", leftSpeed);
        SmartDashboard.putNumber("[DriveAngle]rightSpeedAngle", rightSpeed);
    }    

    @Override
    protected void continous() {
        input.getDriveBaseInput().setLeftSpeed(-leftSpeed);
        input.getDriveBaseInput().setRightSpeed(rightSpeed);
        SmartDashboard.putNumber("[DriveAngle]rightDistance", feedback.getDriveTrainFeedback().getRightDistance());
        SmartDashboard.putNumber("[DriveAngle]leftDistance", feedback.getDriveTrainFeedback().getLeftDistance()));
    }

    @Override
    protected boolean endCondition() {
        // Check to see if distances have been reached
        return ((feedback.getDriveTrainFeedback().getLeftDistance() >= leftDistance) && (feedback.getDriveTrainFeedback().getRightDistance() >= rightDistance))
    }

    @Override
    protected void end() {
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
    }
}