package org.texastorque.auto.commands;

import org.texastorque.auto.Command;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveDistance extends Command {

    ScheduledPID drivePID;
    LowPassFilter lowPass;

    double currentDifference;
    double position;
    double pidCalc;

    double distance;

    /**
     * Create a new DriveDistance
     * @param delay
     * @param distance Distance in inches(?) to drive
     * @apiNote There is a .2 foot offset for stopping in case of a encoder underread! 
     */
    public DriveDistance(double delay, double distance) {
        super(delay);
        drivePID = new ScheduledPID.Builder(0,1,1)
            .setPGains(0.1)
            .setIGains(0.0005)
            .build();
        lowPass = new LowPassFilter(0.5);
        this.distance = distance / 12.0;

    }

    @Override
    protected void init() {
        feedback.getDriveTrainFeedback().resetEncoders();
    }
    
    @Override
    protected void continuous() {
        currentDifference = distance - feedback.getDriveTrainFeedback().getLeftDistance();
        SmartDashboard.putNumber("[DriveDistance]leftDistance", feedback.getDriveTrainFeedback().getLeftDistance());
        SmartDashboard.putNumber("[DriveDistance]rightDistance", feedback.getDriveTrainFeedback().getRightDistance());
        position = lowPass.filter(currentDifference);
        pidCalc = drivePID.calculate(position);
        input.getDriveBaseInput().setLeftSpeed(-.4);
        input.getDriveBaseInput().setRightSpeed(.4);
    }

    @Override
    protected boolean endCondition() {
        // System.out.println(currentDifference);
        if (Math.abs(currentDifference) < .2) {
            return true;
        }
        return false;
    }

    @Override
    protected void end() {
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);

    }

}