package org.texastorque.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

public class AutoManager {
    private static volatile AutoManager instance;

    private HashMap<String, Sequence> autoSequences;
    private SendableChooser<String> autoSelector = new SendableChooser<String>();
    
    private Sequence currentSequence;
    private boolean sequenceEnded;
    
    private NetworkTableInstance NT_instance;
    private NetworkTableEntry NT_offsetEntry;
    private String autoSelectorKey = "AutoList";


    private AutoManager() {
        autoSequences = new HashMap<String, Sequence>();
        

    }

    private void addSequence(String name, Sequence seq) {
        autoSequences.put(name, seq);
        autoSelector.addOption(name, name);
    }

    public void displayChoices() {
        SmartDashboard.putData(autoSelectorKey, autoSelector);
    }

    public void chooseSequence() {
        
        String autoChoice = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(autoSelectorKey).getEntry("selected").getString("N/A");

        if(autoSequences.containsKey(autoChoice)) {
            currentSequence = autoSequences.get(autoChoice);
        } else {
            // INVALID SEQUENCES AHHHHHH
        }


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