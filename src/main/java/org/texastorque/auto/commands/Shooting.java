package org.texastorque.auto.commands;

import org.texastorque.auto.Command;
import org.texastorque.inputs.State.HoodSetpoint;

public class Shooting extends Command{
    private double startTime;
    private double power;
    private int time;
    
    public Shooting(double delay, double pwr){
        super(delay);
        power = pwr;
    }

    @Override
    protected void init() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        time = 3;
        input.getShooterInput().setHoodSetpoint(HoodSetpoint.LONGSHOT);
    }

    @Override
    protected void continuous() {
        input.getMagazineInput().setAutoMag(true);
        input.getShooterInput().setFlywheelSpeed(power);
        if(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > time){
            input.getMagazineInput().setGate(true);
        }

    }

    @Override
    protected boolean endCondition() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > time+1;
    }

    @Override
    protected void end() {
        input.getMagazineInput().setGate(false);
        input.getShooterInput().setFlywheelSpeed(0);
        input.getShooterInput().setHoodSetpoint(HoodSetpoint.NEUTRAL);
    }
}
