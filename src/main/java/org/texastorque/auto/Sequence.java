package org.texastorque.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public abstract class Sequence {
    private ArrayList<ArrayList<Command>> sequence;
    private boolean ended, started;
    private double startTime;
    private int blockIndex;

    
    public Sequence() {
        sequence = new ArrayList<ArrayList<Command>>();
        started = false;
        ended = false;
        blockIndex = 0;
        init();
    }

    protected abstract void init();

    protected void addBlock(ArrayList<Command> block) {
        sequence.add(block);
    }
    
    public void run() {
        if(!started) {
            startTime = Timer.getFPGATimestamp();
            started = true;
            System.out.println("Starting sequence: "+sequence);
        }

        // Soooo basically keep running the commands, if there are none that are still running then stop
        if(blockIndex < sequence.size()) { // If there are blocks left to be run
            boolean blockEnded = true;
            double currentTime = Timer.getFPGATimestamp();
            for(Command command : sequence.get(blockIndex)) {
                if(currentTime - startTime > command.getDelay()) {
                    if(!command.run()) {
                        blockEnded = false;
                    }
                } else {
                    blockEnded = false;
                }
            }

            if (blockEnded) { // if the block has ended
                blockIndex++; // increment block index
                startTime = Timer.getFPGATimestamp();
                System.out.println("Block " + blockIndex + " ended.");
            }
        } 
        else if (!ended) { // If there are no blocks lef
            ended = true;
        }
    }
    
    public boolean hasEnded() {
        return ended;
    }

    public void reset() {
        blockIndex = 0;
        started = false;
        ended = false;
    }
    
    public double getStartTime() {
        return startTime;
    }
}

