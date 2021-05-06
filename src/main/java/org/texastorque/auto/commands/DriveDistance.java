package org.texastorque.auto.commands;

import org.texastorque.auto.Command;
import org.texastorque.inputs.Feedback;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveDistance extends Command {

    SceduledPID drivePID;
    LowPassFilter lowPass;

    double currentDifference;
    double position;
    double pidCalc;

    double distance;

    public DriveDistance(double delay, double distance) {
        super(delay);
        drivePID = new ScheduledPID.Builder(0,1,1)
            .setPGains(0.1)
            .setIGains(0.0005)
            .build();
        lowPass = new LowPassFilter(0.5);
        this.distance = distance / 11.9;

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
        input.getDriveBaseInput().setLeftSpeed(-40/100);
        input.getDriveBaseInput().setRightSpeed(40/100);
    }

}