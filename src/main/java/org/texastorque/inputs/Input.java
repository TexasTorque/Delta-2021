package org.texastorque.inputs;

import org.texastorque.inputs.State.ClimberState;
import org.texastorque.torquelib.util.GenericController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Input {
    public static volatile Input instance;

    // Cached state
    private volatile State state = State.getInstance();

    // Controllers    
    private GenericController driver; // driver controller parameters
    private GenericController operator; // operator  - -
 
    // Instances
    private DriveBaseInput driveBaseInput;
    private IntakeInput intakeInput;
    private MagazineInput magazineInput;

    /**
     * Load the state and create driver/operator controllers
     */
    private Input() {
        driver = new GenericController(0, 0.1);
        operator = new GenericController(1, 0.1);

        // Load instances
        driveBaseInput = new DriveBaseInput();
        intakeInput = new IntakeInput();
        magazineInput = new MagazineInput();
    }

    public void update() {
        driveBaseInput.update();
        intakeInput.update();
        magazineInput.update();
        smartDashboard();
    }

    // ======
    // Drivebase
    // ======

    public class DriveBaseInput implements TorqueInputModule {
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

    // =====
    // Intake
    // =====

    public class IntakeInput implements TorqueInputModule {
        private volatile double rotaryPositionLeft;
        private volatile double rotaryPositionRight;
        private double rotarySpeed = 0;
        private double rollerSpeed = 0;

        private double[] rotarySetpointsLeft = {0, -8, -38};
        private double[] rotarySetpointsRight = {0, 8, 44};

        // Position to start with
        private int neutral = 2;

        @Override
        public void update() {
            if(driver.getRightTrigger()) {
                rotaryPositionLeft = rotarySetpointsLeft[2];
                rotaryPositionRight = rotarySetpointsRight[2];
                rollerSpeed = .8;
            } else if (driver.getLeftTrigger()) {
                rotaryPositionLeft = rotarySetpointsLeft[2];
                rotaryPositionRight = rotarySetpointsRight[2];
                rollerSpeed = -.8;
            } else {
                rotaryPositionLeft = rotarySetpointsLeft[neutral];
                rotaryPositionRight = rotarySetpointsRight[neutral];
                rollerSpeed = 0;
            }
            if(driver.getYButton()) {
                rollerSpeed = 0.5;
            }
        }

        /**
         * Reset rotary to default
         */
        @Override
        public void reset() {
            rotaryPositionLeft = rotarySetpointsLeft[neutral];
            rotaryPositionRight = rotarySetpointsRight[neutral];
            rollerSpeed = 0;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[Input]rollerSpeed", rollerSpeed); 
            SmartDashboard.putNumber("[Input]rotarySpeed", rotarySpeed);
            SmartDashboard.putNumber("[Input]rotaryPositionLeft", rotaryPositionLeft);
            SmartDashboard.putNumber("[Input]rotaryPositionRight", rotaryPositionRight);
        }

        /**
         * @return The rotary speed
         */
        public double getRotarySpeed() {
            return rotarySpeed;
        }

        /**
         * @return The rotary's left position
         */
        public double getRotaryPositionLeft() {
            return rotaryPositionLeft;
        }

        /**
         * @return The rotary's right position
         */
        public double getRotaryPositionRight() {
            return rotaryPositionRight;
        }

        /**
         * @return The roller speed
         */
        public double getRollerSpeed() {
            return rollerSpeed;
        }

        /**
         * Update the roller speed
         * @param rollerSpeed Speed between [-1,1]
         */
        public void setRollerSpeed(double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }

        /**
         * Update the rotary position (for auto)
         * @param rotaryIndex The index of rotarySetpoints to set
         */
        public void setRotaryPosition(int rotaryIndex) {
            rotaryPositionLeft = rotarySetpointsLeft[rotaryIndex];
            rotaryPositionRight = rotarySetpointsRight[rotaryIndex];
        }
    }

    // =====
    // Magazine
    // =====

    public class MagazineInput implements TorqueInputModule {
        private double velocityLow = 0;
        private double velocityHigh = 0;
        private double velocityGate = 0;

        private double speedLow = .6;
        private double speedHigh = .5;
        private double speedGate = 1;
        
        // 0=nothing, 1=forward, -1=backward
        private int magLow; 
        private int magHigh; 

        private boolean autoMag = false;
        private boolean lastShooting = false;
        private boolean shootingNow = false;
        
        @Override
        public void update() {
            reset();

            if(operator.getLeftCenterButton()) autoMag = true; // left d pad turn on automag
            else if (operator.getRightCenterButton()) autoMag = false; // right d pad turn off automag
            
            // = High Mag
            if(operator.getLeftTrigger()) { // == Ball In
                magHigh = 1;
                velocityHigh = operator.getLeftZAxis() * speedHigh;
            } else if(operator.getLeftBumper()) { // == Ball Out
                magHigh = -1;
                velocityHigh = -speedHigh;
            }

            // = Low Mag
            if(operator.getRightTrigger()){ // == Balls In
                magLow = 1;
                velocityLow = -operator.getRightZAxis() * speedLow;
            }
            else if(operator.getRightBumper()){
                magLow = -1;
                velocityLow = speedLow;
            }

            // = Gate
            if(operator.getDPADDown() || operator.getDPADLeft()) {
                velocityGate = speedGate;
            }

            shootingNow = operator.getDPADUp();
        }

        @Override
        public void reset() {
            velocityLow = 0;
            velocityHigh = 0;
            velocityGate = 0;
            magLow = 0;
            magHigh = 0;
        }

        @Override
        public void smartDashboard() {}

        public boolean needPreShoot() {
            return operator.getDPADUp();
        }

        public double getVelocityLow() {
            return velocityLow;
        }

        public double getVelocityHigh() {
            return velocityHigh;
        }

        public double getVelocityGate() {
            return velocityGate;
        }

        public int getMagLow() {
            return magLow;
        }

        public int getMagHigh() {
            return magHigh;
        }

        public boolean getAutoMag() {
            return autoMag;
        }

        /**
         * Turn off/on the gate
         * @param on True=on, false=off
         */
        public void setGate(boolean on) { 
            velocityGate = on ? -speedGate : 0; 
        }

        /**
         * Turn off/on the highMag
         */
        public void setHighMag(boolean on) {
            velocityHigh = on ? speedHigh : 0;
        }

        /**
         * Turn off/on the lowMag
         */
        public void setLowMag(boolean on) {
            velocityLow = on ? speedLow : 0;
        }
    }

    // =====
    // Climber
    // =====
    public class ClimberInput implements TorqueInputModule {
        private double climberLeft = 0;
        private double climberRight = 0;
        
        private ClimberState climberStatus = ClimberState.NEUTRAL;
        private boolean climberServoLocked = true;
        private boolean climbStarted = false;
        private boolean climbStartedDT = false;
        private boolean manualClimb = false;
        private int sideToExtend = 0;

        @Override
        public void update() {
            manualClimb = false;
            climberStatus = ClimberState.NEUTRAL;
            sideToExtend = 0;

            if(driver.getDPADUp()) { // Extend climber
                if(!climbStarted) { // if the climb has not been started
                    // Climber.resetClimb
                    climbStarted = true;
                    climbStartedDT = true;
                }
                climberStatus = ClimberState.EXTEND;
            } else if (driver.getDPADDown()) { // Retract climber
                climberStatus = ClimberState.RETRACT;
            } else if (driver.getDPADLeft()) {// extend left
                manualClimb = true;
                sideToExtend = -1;
            } else if (driver.getDPADRight()) { // extend right
                manualClimb = true;
                sideToExtend = 1;
            }
        }

        @Override
        public void reset() {
            // TODO Auto-generated method stub

        }

        @Override
        public void smartDashboard() {
            // TODO Auto-generated method stub

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

    /**
     * @return The instance of the IntakeInput
     */
    public IntakeInput getIntakeInput() {
        return intakeInput;
    }

    /**
     * @return The instance of the MagazineInput
     */
    public MagazineInput getMagazineInput() {
        return magazineInput;
    }

    // =====
    // Misc
    // =====

    /**
     * Update the values in SmartDashboard
    */
    public void smartDashboard() {
        driveBaseInput.smartDashboard();
        intakeInput.smartDashboard();
        magazineInput.smartDashboard();
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
