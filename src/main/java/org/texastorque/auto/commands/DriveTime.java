package org.texastorque.auto.commands;

import org.texastorque.auto.Command;

public class DriveTime extends Command {

    private double startTime;
    private double speed;
    private double time;
    
    /**
     * drives for a designated time and speed
     * @param delay
     * @param time
     * @param speed
     */
    public DriveTime(double delay, double time, double speed) {
        super(delay);
        this.time = time;
        this.speed = speed;
    }

    @Override
    protected void init() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    @Override
    protected void continuous() {
        input.getDriveBaseInput().setLeftSpeed(-speed);
        input.getDriveBaseInput().setRightSpeed(speed);
    }

    @Override
    protected boolean endCondition() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > time;
    }

    @Override
    protected void end() {
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
    }
    






}