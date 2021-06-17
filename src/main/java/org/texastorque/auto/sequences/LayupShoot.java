package org.texastorque.auto.sequences;

import java.util.ArrayList;

import org.texastorque.auto.Command;
import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.DriveTime;
import org.texastorque.auto.commands.EmptyMagazine;
import org.texastorque.auto.commands.SetShooter;
import org.texastorque.inputs.State.FlywheelSpeed;
import org.texastorque.inputs.State.HoodSetpoint;

public class LayupShoot extends Sequence {

    @Override
    protected void init() {
        ArrayList<Command> goTo = new ArrayList<>();
        goTo.add(new DriveTime(0, 1.5, .4));

        ArrayList<Command> revShooter = new ArrayList<>();
        revShooter.add(new SetShooter(0, HoodSetpoint.LAYUP, FlywheelSpeed.LAYUP));

        ArrayList<Command> shootBalls = new ArrayList<>();
        shootBalls.add(new EmptyMagazine(0, 3));

        ArrayList<Command> resetShooter = new ArrayList<>();
        resetShooter.add(new SetShooter(0, HoodSetpoint.NEUTRAL, FlywheelSpeed.NEUTRAL));

        addBlock(goTo);
        addBlock(revShooter);
        addBlock(shootBalls);
        addBlock(resetShooter);
    }

}
