package org.texastorque.auto;

import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State;

/**
 * A single action to perform in auto
 */
public abstract class Command {
   
    
    // Instances
    protected Input input = Input.getInstance();
    protected Feedback feedback = Feedback.getInstance();
    protected State state = State.getInstance();

    // State
    private double delay;
    private boolean ended, started;

    /**
     * Create a new command with a set delay.
     * All commands are set to activate in a command after the set delay.
     */
    public Command(double delay) {
        this.delay = delay;
        ended = false;
        started = false;
    }

    public boolean run() {
        if(ended) return ended;
        
        // First loop
        if(!started) {
            init();
            started = true;
        }

        continuous();

        if(endCondition()) {
            end();
            ended = true;
        }
        return ended;
    }
    
    public double getDelay() {
        return delay;
    }

    /**
     * Run on first loop
     */
    protected abstract void init();
    
    /**
     * Run on iteration
     */
    protected abstract void continuous();

    /**
     * Checked every iteration, if true, runs end
     */
    protected abstract boolean endCondition();
    
    /**
     * Run after end condition returns true and before command exits
     */
    protected abstract void end();
}

