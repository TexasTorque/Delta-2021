package org.texastorque.auto.sequences;

import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.*;

import java.util.ArrayList;
import org.texastorque.auto.Command;

import org.texastorque.inputs.Input;

public class RawShoot extends Sequence {
    
    @Override
    protected void init() {
        ArrayList<Command> driveTime1 = new ArrayList<>();
        //add shoot 3 times
        driveTime1.add(new Shooting(0,4250));
        driveTime1.add(new Shooting(0,4250));
        driveTime1.add(new Shooting(0,4250));
        driveTime1.add(new DriveTime(0, 0.3, -2));
        driveTime1.add(new DriveTime(0, 0.6, 2));
        addBlock(driveTime1);
    }
    
}