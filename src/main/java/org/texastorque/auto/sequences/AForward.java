package org.texastorque.auto.sequences;

import java.util.ArrayList;

import org.texastorque.auto.Command;
import org.texastorque.auto.Sequence;
import org.texastorque.auto.commands.PathWeaver4;
import org.texastorque.auto.commands.Weaver5VelocityTesting;

public class AForward extends Sequence {

    @Override
    protected void init() {
        ArrayList<Command> a = new ArrayList<>();
        // a.add(new PathWeaver(0, "AForward.wpilib.json"));
        a.add(new PathWeaver4(0, "AForward.wpilib.json"));
        // a.add(new Weaver5VelocityTesting(0, "AForward.wpilib.json"));


        addBlock(a);
    }
    
}