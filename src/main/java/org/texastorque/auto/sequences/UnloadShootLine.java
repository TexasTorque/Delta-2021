package org.texastorque.auto.sequences;

import java.util.ArrayList;

import org.texastorque.auto.Command;
import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.DriveDistance;
import org.texastorque.auto.commands.DriveTime;
import org.texastorque.auto.commands.EmptyMagazine;
import org.texastorque.auto.commands.SetShooter;
import org.texastorque.inputs.State.FlywheelSpeed;
import org.texastorque.inputs.State.HoodSetpoint;

public class UnloadShootLine extends Sequence {

    @Override
    protected void init() {
        // Set the shooter
        ArrayList<Command> prepareShooter = new ArrayList<>();
        prepareShooter.add(new SetShooter(0, HoodSetpoint.UNLOADSHOOTLINE, FlywheelSpeed.UNLOADSHOOTLINE));

        // Execute output
        ArrayList<Command> shootBalls = new ArrayList<>();
        shootBalls.add(new EmptyMagazine(0, 3));

        // Reset Shooter
        ArrayList<Command> resetShooter = new ArrayList<>();
        resetShooter.add(new SetShooter(0, HoodSetpoint.NEUTRAL, FlywheelSpeed.NEUTRAL));

        // Go back
        ArrayList<Command> goBack = new ArrayList<>();
        // goBack.add(new DriveTime(0, .25, -.4));
        goBack.add(new DriveDistance(0,-20));

        // Go forward
        ArrayList<Command> goForward = new ArrayList<>();
        // goForward.add(new DriveTime(0, .75, .4));
        goForward.add(new DriveDistance(0, 30));

        addBlock(prepareShooter);
        addBlock(shootBalls);
        addBlock(resetShooter);
        addBlock(goBack);
        addBlock(goForward);
    }

}
