package org.texastorque.auto.commands;

import org.texastorque.auto.Command;

import edu.wpi.first.wpilibj.Timer;

public class EmptyMagazine extends Command {
    private double time;
    private double start;

    /**
     * @param delay
     * @param time How long to run the routine - go over if anything but try to tune
     */
    public EmptyMagazine(double delay, double time) {
        super(delay);
        this.time = time;
    }

    @Override
    protected void init() {
        start = Timer.getFPGATimestamp();
        input.getMagazineInput().setShootingNow(true);
    }

    @Override
    protected void continuous() {}

    @Override
    protected boolean endCondition() {
        return Timer.getFPGATimestamp() - start >= time;
    }

    @Override
    protected void end() {
        input.getMagazineInput().setShootingNow(false);
    }
    
}
