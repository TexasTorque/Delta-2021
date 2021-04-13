package org.texastorque.inputs;

import org.texastorque.torquelib.util.GenericController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Input {
    public static volatile Input instance;

    // Cached state
    private volatile State state;

    // Controllers    
    private GenericController driver;
    private GenericController operator;

    // Instances
    private DriveBaseInput driveBaseInput;

    /**
     * Load the state and create driver/operator controllers
     */
    private Input() {
        state = State.getInstance();
        driver = new GenericController(0, 0.1);
        operator = new GenericController(1, 0.1);

        // Load instances
        driveBaseInput = new DriveBaseInput();
    }

    public void update() {
        driveBaseInput.update();
        smartDashboard();
    }

    // ======
    // Drivebase
    // ======

    public class DriveBaseInput implements TorqueInput {
        private volatile double leftSpeed = 0;
        private volatile double rightSpeed = 0;
        
        /**
         * Update the left and right speeds
         */
        public void update() {
            double leftRight = driver.getRightXAxis();
            leftSpeed = .2*(driver.getLeftYAxis() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
            rightSpeed = .2*(-driver.getLeftYAxis() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
        }

        /**
         * Reset the left and right speeds
         */
        public void reset() {
            leftSpeed = 0;
            rightSpeed = 0;
        }

        /**
         * @return The left speed of the DriveBase
         */
        public double getLeftSpeed() {
            return leftSpeed;
        }

        /**
         * @return The right speed of the DriveBase
         */
        public double getRightSpeed() {
            return rightSpeed;
        }

        /**
         * Update the SmartDasboard values
         */
        public void smartDashboard() {
            SmartDashboard.putNumber("[Input]leftSpeed", leftSpeed);
            SmartDashboard.putNumber("[Input]rightSpeed", rightSpeed);
        }
    }

    // ======
    // Getters
    // ======
    
    /**
     * @return The instance of the DriveBaseInput
     */
    public DriveBaseInput getDriveBaseInput() {
        return driveBaseInput;
    }    


    // =====
    // Misc
    // =====

    /**
     * Update the values in SmartDashboard
    */
    public void smartDashboard() {
        driveBaseInput.smartDashboard();
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
