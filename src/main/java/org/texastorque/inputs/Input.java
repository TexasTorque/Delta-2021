package org.texastorque.inputs;

import org.texastorque.torquelib.util.GenericController;

public class Input {
    public static volatile Input instance;

    // Cached state
    private volatile State state;

    // Controllers    
    private GenericController driver;
    private GenericController operator;

    /**
     * Load the state and create driver/operator controllers
     */
    private Input() {
        state = State.getInstance();
        driver = new GenericController(0, 0.1);
        operator = new GenericController(1, 0.1);
    }

    public void update() {
        updateDrive();
    }

    // ======
    // Drivebase
    // ======

    public void updateDrive() {
        double leftRight = driver.getRightXAxis();
        
    }

    /**
     * Get the Input instance
     * @return Input
     */
    public static synchronized Input getInstance() {
        if(instance == null) {
            instance = new Input();
        }
        return instance;
    }
}
