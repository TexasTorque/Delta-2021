package org.texastorque.auto.sequences;

import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.*;
import java.util.ArrayList;
import org.texastorque.auto.Command;

public class Testing extends Sequence {
    
    @Override
    protected void init() {
        ArrayList<Command> driveTime1 = new ArrayList<>();
        // driveTime1.add(new DriveTime(0, 1, 0.5));
        // driveTime1.add(new DriveAngle(0, 90, 1, true));
        driveTime1.add(new DriveDistance(0, 80));
        addBlock(driveTime1);
    }
    
}