package org.texastorque.auto.commands;

import org.texastorque.auto.Command;
import org.texastorque.inputs.State.FlywheelSpeed;
import org.texastorque.inputs.State.HoodSetpoint;

public class SetShooter extends Command {
    private HoodSetpoint hoodSetpoint;
    private FlywheelSpeed flywheelSpeed;

    public SetShooter(double delay, HoodSetpoint hoodSetpoint, FlywheelSpeed flywheelSpeed) {
        super(delay);
        this.hoodSetpoint = hoodSetpoint;
        this.flywheelSpeed = flywheelSpeed;
    }

    @Override
    protected void init() {
        double speed = flywheelSpeed == FlywheelSpeed.LIMELIGHT ? input.getShooterInput().getLimelightFlywheelSpeed()
                : flywheelSpeed.getValue();
        input.getShooterInput().setFlywheelSpeed(speed);
        input.getShooterInput().setHoodSetpoint(hoodSetpoint);
    }

    @Override
    protected void continuous() {
    }

    @Override
    protected boolean endCondition() {
        // Stop if flywheel speed is 0 (natural ramp down) or the speed is +/- 200 of
        // requested
        return flywheelSpeed.getValue() == 0 || input.getShooterInput().flywheelSpeedInBounds(200);
    }

    @Override
    protected void end() {
    }

}
