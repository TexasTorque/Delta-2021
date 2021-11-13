package org.texastorque.inputs;

import org.texastorque.constants.Constants;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.controlLoop.TorqueSlewLimiter;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.torquelib.util.TorqueClick;
import org.texastorque.torquelib.util.TorqueLogiPro;
import org.texastorque.torquelib.util.TorqueMathUtil;
import org.texastorque.torquelib.util.TorqueToggle;
import org.texastorque.torquelib.util.TorqueLogiPro.PovState;
import org.texastorque.inputs.State.*;
import org.texastorque.subsystems.Climber;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Input {
    private static volatile Input instance;

    // Cached state
    private State state = State.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // Controllers
    private TorqueLogiPro driver; // driver controller parameters
    private GenericController operator; // operator - -

    // Instance
    private DrivebaseInput drivebaseInput;
    private IntakeInput intakeInput;
    private MagazineInput magazineInput;
    private ClimberInput climberInput;
    private ShooterInput shooterInput;

    private Input() {
        driver = new TorqueLogiPro(0);
        operator = new GenericController(1, 0.1); 

        drivebaseInput = new DrivebaseInput();
        intakeInput = new IntakeInput();
        magazineInput = new MagazineInput();
        climberInput = new ClimberInput();
        shooterInput = new ShooterInput();
    }

    public void update() {
        drivebaseInput.update();
    }

    public class DrivebaseInput extends TorqueInput {
        private volatile double leftSpeed = 0;
        private volatile double rightSpeed = 0;
        public volatile double speedMult = 0.7;

        private TorqueClick speedUp = new TorqueClick();
        private TorqueClick speedDown = new TorqueClick();

        private TorqueToggle rollOrYaw = new TorqueToggle();

        public DrivebaseInput() {
        }

        @Override
        public void update() {
            rollOrYaw.calc(driver.getButtonByIndex(12));
            double leftRight = rollOrYaw.get() ? driver.getRoll() : driver.getYaw(); // get joystick position


            if (driver.getButtonByIndex(11)) {
                getClimberInput().setClimbStartedDT(false);
            }
            if (driver.getButtonByIndex(5)) { // Vision toggle
                if (state.getRobotState() == RobotState.TELEOP)
                    state.setRobotState(RobotState.VISION);
                else if (state.getRobotState() == RobotState.VISION)
                    state.setRobotState(RobotState.TELEOP);
            }
            if (getClimberInput().getClimbStartedDT()) {
                leftSpeed = .4 * (driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
                rightSpeed = .4 * (-driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
            } else if (getShooterInput().getHoodSetpoint() == HoodSetpoint.NEUTRAL.getValue()) { // maximum speed when
                // neutral
                leftSpeed = driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight);
                rightSpeed = -driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight);
            } else {
                leftSpeed =  0.2 * (driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
                rightSpeed = 0.2 * (-driver.getPitch() - 0.4 * Math.pow(leftRight, 4) * Math.signum(leftRight));
            }

            boolean up = speedUp.calc(driver.getButtonByIndex(4));
            boolean down = speedDown.calc(driver.getButtonByIndex(3));
            if (up) speedUp();
            else if (down) speedDown();
            else if (up && down) speedReset();
        }

        public void speedUp() {
            speedMult = (double)TorqueMathUtil.constrain(speedMult+.1, 0.2, 0.8);
        }
    
        public void speedDown() {
            speedMult = (double)TorqueMathUtil.constrain(speedMult-.1, 0.2, 0.8);
        }
    
        public void speedReset() {
            speedMult = 0.6;
        }
    
        public double getSpeedMult() {
            return speedMult;
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
            SmartDashboard.putNumber("[Input]speedMult", speedMult);
        }

        // ===
        // Auto Only
        // ===
        private boolean doingVelocity = false;

        /**
         * Get whether DB should output in velocity
         */
        public boolean getDoingVelocity() {
            return doingVelocity;
        }

        /**
         * AUTO ONLY
         */
        public void setRightSpeed(double rightSpeed) {
            this.rightSpeed = rightSpeed;
        }

        /**
         * AUTO ONLY
         */
        public void setLeftSpeed(double leftSpeed) {
            this.leftSpeed = leftSpeed;
        }

        /**
         * AUTO ONLY
         */
        public void setDoingVelocity(boolean doingVelocity) {
            this.doingVelocity = doingVelocity;
        }
    }


    public class IntakeInput extends TorqueInput {
        private volatile double rotaryPositionLeft;
        private volatile double rotaryPositionRight;
        private double rotarySpeed = 0;
        private double rollerSpeed = 0;

        private double[] rotarySetpointsLeft = { 0, -21.5, -38 };
        private double[] rotarySetpointsRight = { 0, 21.5, 38 };

        // Position to start with
        private RotaryState neutral = RotaryState.PRIME;

        @Override
        public void update() {
            if (driver.getButtonByIndex(2)) {
                neutral = neutral == RotaryState.PRIME ? RotaryState.UP : RotaryState.PRIME;
            }
            if (driver.getButtonByIndex(1)) {
                rotaryPositionLeft = rotarySetpointsLeft[RotaryState.DOWN.getValue()];
                rotaryPositionRight = rotarySetpointsRight[RotaryState.DOWN.getValue()];
                rollerSpeed = .7;
            } else if (driver.getButtonByIndex(7)) {
                rotaryPositionLeft = rotarySetpointsLeft[RotaryState.DOWN.getValue()];
                rotaryPositionRight = rotarySetpointsRight[RotaryState.DOWN.getValue()];
                rollerSpeed = -.7;
            } else {
                rotaryPositionLeft = rotarySetpointsLeft[neutral.getValue()];
                rotaryPositionRight = rotarySetpointsRight[neutral.getValue()];
                rollerSpeed = 0;
            }
            if (driver.getButtonByIndex(11)) {
                rollerSpeed = 0.5;
            }
        }

        /**
         * Reset rotary to default
         */
        @Override
        public void reset() {
            rotaryPositionLeft = rotarySetpointsLeft[neutral.getValue()];
            rotaryPositionRight = rotarySetpointsRight[neutral.getValue()];
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

        /*
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
         * 
         * @param rollerSpeed Speed between [-1,1]
         */
        public void setRollerSpeed(double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }

        /**
         * Update the rotary position (for auto)
         * 
         * @param rotaryIndex The index of rotarySetpoints to set
         */
        public void setRotaryPosition(int rotaryIndex) {
            rotaryPositionLeft = rotarySetpointsLeft[rotaryIndex];
            rotaryPositionRight = rotarySetpointsRight[rotaryIndex];
        }
    }

    public class MagazineInput extends TorqueInput {
        private double velocityLow = 0;
        private double velocityHigh = 0;

        private final double speedLow = .5;
        private final double speedHigh = 1.;

        private final double speedLowAuto = .5;
        private final double speedHighAuto = 1.;

        // 0=nothing, 1=forward, -1=backward
        private int magLow;
        private int magHigh;

        private boolean autoMag = false;
        private boolean shootingNow = false;

        @Override
        public void update() {
            reset();

            if (operator.getLeftCenterButton()) {
                feedback.getMagazineFeedback().resetAutomag(); // Reset automag
                autoMag = true; // left d pad turn on automag
            } else if (operator.getRightCenterButton())
                autoMag = false; // right d pad turn off automag

            // = High Mag
            if (operator.getLeftTrigger()) { // == Ball In
                magHigh = 1;
                velocityHigh = speedHigh;
            } else if (operator.getLeftBumper()) { // == Ball Out
                magHigh = -1;
                velocityHigh = -speedHigh;
            }

            // = Low Mag
            if (operator.getRightTrigger()) { // == Balls In
                magLow = 1;
                velocityLow = speedLow;
            } else if (operator.getRightBumper()) {
                magLow = -1;
                velocityLow = -speedLow;
            }

            shootingNow = operator.getDPADUp();
        }

        @Override
        public void reset() {
            velocityLow = 0;
            velocityHigh = 0;
            magLow = 0;
            magHigh = 0;
        }

        @Override
        public void smartDashboard() {
        }

        public boolean shootingNow() {
            return shootingNow;
        }

        public double getVelocityLow() {
            return velocityLow;
        }

        public double getVelocityHigh() {
            return velocityHigh;
        }

        public double getSetSpeedLowAuto() {
            return speedLowAuto;
        }

        public double getSetSpeedHighAuto() {
            return speedHighAuto;
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

        /**
         * Turn off/on the autoMag
         */
        public void setAutoMag(boolean on) {
            autoMag = on;
        }

        /**
         * @param on Turn off/on the shooting now routine
         */
        public void setShootingNow(boolean on) {
            shootingNow = on;
        }
    }

    public class ClimberInput extends TorqueInput {
        private double climberLeft = 0;
        private double climberRight = 0;

        private ClimberState climberStatus = ClimberState.NEUTRAL;
        private ClimberSide sideToExtend = ClimberSide.NEUTRAL;
        private boolean climbStarted = false;
        private boolean climbStartedDT = false;
        private boolean manualClimb = false;

        @Override
        public void update() {
            reset();
            if (driver.atPovState(PovState.NORTH)) { // Extend climber
                if (!climbStarted) { // if the climb has not been started
                    Climber.getInstance().resetClimb();
                    climbStarted = true;
                    climbStartedDT = true;
                }
                climberStatus = ClimberState.EXTEND;
            } else if (driver.atPovState(PovState.SOUTH)) { // Retract climber
                climberStatus = ClimberState.RETRACT;
                climbStartedDT = false;
            } else if (driver.atPovState(PovState.WEST)) {// extend left
                manualClimb = true;
                sideToExtend = ClimberSide.LEFT;
            } else if (driver.atPovState(PovState.EAST)) { // extend right
                manualClimb = true;
                sideToExtend = ClimberSide.RIGHT;
            } else {
                climbStarted = false;
            }
        }

        @Override
        public void reset() {
            manualClimb = false;
            climberStatus = ClimberState.NEUTRAL;
            sideToExtend = ClimberSide.NEUTRAL;
        }

        /**
         * Update Climb Started DT (for controlling DriveTrain)
         * 
         * @param val Value to set to
         */
        public void setClimbStartedDT(boolean val) {
            climbStartedDT = val;
        }

        public boolean getManualClimb() {
            return manualClimb;
        }

        public ClimberSide getSideToExtend() {
            return sideToExtend;
        }

        public ClimberState getClimberStatus() {
            return climberStatus;
        }

        public double getClimberLeft() {
            return climberLeft;
        }

        public double getClimberRight() {
            return climberRight;
        }

        public boolean getClimbStartedDT() {
            return climbStartedDT;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putString("[Input]sideToExtend", sideToExtend == ClimberSide.LEFT ? "LEFT"
                    : sideToExtend == ClimberSide.RIGHT ? "RIGHT" : "NEUTRAL");
            SmartDashboard.putString("[Input]climberStatus", climberStatus == ClimberState.EXTEND ? "EXTEND"
                    : climberStatus == ClimberState.RETRACT ? "RETRACT" : "NEUTRAL");
            SmartDashboard.putBoolean("[Input]climbStarted", climbStarted);
            SmartDashboard.putBoolean("[Input]climbStartedDT", climbStartedDT);
            SmartDashboard.putBoolean("[Input]manualClimb", manualClimb);

        }

    }

    public class ShooterInput extends TorqueInput {
        private double flywheelSpeed = 0;
        private double distanceAway = 0;
        // On-the-fly fine-tuning for hood & shooter by operator ≧◠‿◠≦✌
        private double hoodFine = 0;
        private double shooterFine = 0;

        private boolean doRumble = false;

        private double hoodSetpoint;

        public ShooterInput() {
            SmartDashboard.putNumber("[Input]ManualHood", 0);
            SmartDashboard.putNumber("[Input]ManualFlywheel", 0);
        }

        @Override
        public void update() {
            reset();

            hoodFine = -operator.getLeftYAxis() * 10;
            shooterFine = -operator.getRightYAxis() * 100;

            if (operator.getYButton()) { // Layup
                hoodSetpoint = HoodSetpoint.LAYUP.getValue();
                flywheelSpeed = FlywheelSpeed.LAYUP.getValue() + shooterFine;
            } else if (operator.getBButton()) { // Trench
                hoodSetpoint = HoodSetpoint.TRENCH.getValue();
                flywheelSpeed = FlywheelSpeed.TRENCH.getValue() + shooterFine;
            } else if (operator.getAButton()) { // Longshot
                hoodSetpoint = HoodSetpoint.LONGSHOT.getValue();
                flywheelSpeed = FlywheelSpeed.LONGSHOT.getValue() + shooterFine;
            } else if (operator.getXButton()) { // limelight
                // hoodSetpoint = HoodSetpoint.LIMELIGHT;
                // flywheelSpeed = getLimelightFlywheelSpeed();
                hoodSetpoint = SmartDashboard.getNumber("[Input]ManualHood", 0);
                flywheelSpeed = SmartDashboard.getNumber("[Input]ManualFlywheel", 0);
            }

            doRumble = flywheelSpeedInBounds(200);
            operator.setRumble(doRumble);
        }

        /**
         * TODO: Make this automatically create teh perfect setpoint/speed -- not
         * highest priority.
         */
        public double getLimelightFlywheelSpeed() {
            feedback.getLimelightFeedback().setLimelightOn(true);
            distanceAway = feedback.getLimelightFeedback().getDistanceAway();
            return FlywheelSpeed.LIMELIGHT.getValue();
            // return flywheelSpeed = 4170.043 + 51.84663*distanceAway -
            // 3.67*Math.pow(distanceAway,2) + 0.1085119*Math.pow(distanceAway,3) -
            // 0.0009953746*Math.pow(distanceAway, 4);
        }

        /**
         * @param delta +/- #
         * @return Whether the encoder flywheel speed is within delta of the requested
         *         speed (& not 0!)
         */
        public boolean flywheelSpeedInBounds(double delta) {
            return (flywheelSpeed != 0)
                    && (Math.abs(flywheelSpeed - feedback.getShooterFeedback().getShooterVelocity()) <= delta);
        }

        public double getFlywheelSpeed() {
            return flywheelSpeed;
        }

        public double getHoodSetpoint() {
            return hoodSetpoint;
        }

        public void setFlywheelSpeed(double flywheelSpeed) {
            this.flywheelSpeed = flywheelSpeed;
        }

        public void setHoodSetpoint(HoodSetpoint setPoint) {
            hoodSetpoint = setPoint.getValue();
        }

        @Override
        public void reset() {
            hoodSetpoint = HoodSetpoint.NEUTRAL.getValue();
            flywheelSpeed = 0;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[Input]flywheelSpeed", flywheelSpeed);
            SmartDashboard.putNumber("[Input]hoodSetpoint", hoodSetpoint);
            SmartDashboard.putNumber("[Input]hoodFine", hoodFine);
            SmartDashboard.putNumber("[Input]shooterFine", shooterFine);
        }
    }


    public DrivebaseInput getDrivebaseInput() {
        return drivebaseInput;
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

    /**
     * @return The instance of ClimberInput
     */
    public ClimberInput getClimberInput() {
        return climberInput;
    }

    /**
     * @return The instance of ShooterInput
     */
    public ShooterInput getShooterInput() {
        return shooterInput;
    }

    // =====
    // Misc
    // =====

    /**
     * Update the values in SmartDashboard
     */
    public void smartDashboard() { // calls the smartDashboard method of each subclass within
        drivebaseInput.smartDashboard();
        intakeInput.smartDashboard();
        magazineInput.smartDashboard();
        climberInput.smartDashboard();
        shooterInput.smartDashboard();
    }

	public static synchronized Input getInstance() {
		return (instance == null) ? instance = new Input() : instance;
	}
}
