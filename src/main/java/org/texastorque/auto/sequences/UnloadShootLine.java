package org.texastorque.auto.sequences;

import java.util.ArrayList;

import org.texastorque.auto.Command;
import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.EmptyMagazine;
import org.texastorque.auto.commands.SetShooter;
import org.texastorque.inputs.State.FlywheelSpeed;
import org.texastorque.inputs.State.HoodSetpoint;

public class UnloadShootLine extends Sequence {

    @Override
    protected void init() {
        // Set the shooter
        ArrayList<Command> prepareShooter = new ArrayList<>();
        prepareShooter.add(new SetShooter(0, HoodSetpoint.LIMELIGHT, FlywheelSpeed.LIMELIGHT));
    
        // Execute output
        ArrayList<Command> shootBalls = new ArrayList<>();
        shootBalls.add(new EmptyMagazine(0, 5));

        // Reset Shooter
        ArrayList<Command> resetShooter = new ArrayList<>();
        resetShooter.add(new SetShooter(0, HoodSetpoint.NEUTRAL, FlywheelSpeed.NEUTRAL));
    }
    
}
