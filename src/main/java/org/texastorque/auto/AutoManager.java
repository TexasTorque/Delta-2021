package org.texastorque.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

import org.texastorque.auto.sequences.*;

public class AutoManager {
    private static volatile AutoManager instance;

    private HashMap<String, Sequence> autoSequences;
    private SendableChooser<String> autoSelector = new SendableChooser<String>();
    
    private Sequence currentSequence;
    private boolean sequenceEnded;
    
    private String autoSelectorKey = "AutoList";

    private AutoManager() {
        autoSequences = new HashMap<String, Sequence>();
       
        addSequence("Empty", new Empty());
        addSequence("Testing", new Testing());
        addSequence("AForward", new AForward());
        addSequence("RawShoot", new RawShoot());
        
        displayChoices();
    }

    private void addSequence(String name, Sequence seq) {
        autoSequences.put(name, seq);

        if (autoSequences.size() == 0) {
            autoSelector.setDefaultOption(name, name);
        } else {
            autoSelector.addOption(name, name);
        }
    }

    public void runCurrentSequence() {
        currentSequence.run();
        sequenceEnded = currentSequence.hasEnded(); // manage state of sequence
    }
    
    public void chooseCurrentSequence() {
        String autoChoice = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(autoSelectorKey).getEntry("selected").getString("N/A");

        if(autoSequences.containsKey(autoChoice)) {
            System.out.println("Switching to auto: "+autoChoice);
            currentSequence = autoSequences.get(autoChoice);
        } else {
            currentSequence = new Empty();
        }

        resetCurrentSequence();
        sequenceEnded = false;
    }

    /**
     * Set sequence with sequence object
     */
    public void setCurrentSequence(Sequence seq) {
        currentSequence = seq;
        resetCurrentSequence();
    }

    /**
     * Send sequence list to SmartDashboard
     */
    public void displayChoices() {
        SmartDashboard.putData(autoSelectorKey, autoSelector);
    }

    public void resetCurrentSequence() {
        currentSequence.reset();
    }

    /**
     * Return the state variable that shows whether the sequence is ended or not
     */
    public boolean getSequenceEnded() {
        return sequenceEnded;
    }
     
    /**
     * Get the AutoManager instance
     * @return AutoManager
     */
    public static synchronized AutoManager getInstance() {
        if(instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }
}