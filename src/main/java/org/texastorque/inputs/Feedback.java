package org.texastorque.inputs;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import org.texastorque.constants.Constants;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Feedback {
    private static volatile Feedback instance;

    // Cached modules
    private static DriveTrainFeedback driveTrainFeedback;
    private static IntakeFeedback intakeFeedback;
    private static ShooterFeedback shooterFeedback;
    private static GyroFeedback gyroFeedback;
    private static WheelOfFortuneFeedback wheelOfFortuneFeedback;

    private static ArrayList<TorqueFeedbackModule> modules;

    /**
     * Load in the modules
     */
    private Feedback() {
        driveTrainFeedback = new DriveTrainFeedback();
        intakeFeedback = new IntakeFeedback();
        shooterFeedback = new ShooterFeedback();
        gyroFeedback = new GyroFeedback();
        wheelOfFortuneFeedback = new WheelOfFortuneFeedback();

        modules = new ArrayList<>();
        modules.add(driveTrainFeedback);
        modules.add(intakeFeedback);
        modules.add(shooterFeedback);
        modules.add(gyroFeedback);
        modules.add(wheelOfFortuneFeedback);
    }

    /**
     * Update each Feedback module
     */
    public void update() {
        modules.forEach(TorqueFeedbackModule::update);
        smartDashboard();
    }

    /**
     * Update SmartDashboard values
     */
    public void smartDashboard() {
        modules.forEach(TorqueFeedbackModule::smartDashboard);
    }

    // =====
    // Drive Train
    // =====
    public class DriveTrainFeedback extends TorqueFeedbackModule {
        private double leftTare = 0;
        private double rightTare = 0;
        private double leftPosition;
        private double rightPosition;
        private double leftVelocity;
        private double rightVelocity;

        /**
         * Set the left position
         * 
         * @param leftPosition Usually TorqueSparkMax.getPosition()
         */
        public void setLeftPosition(double leftPosition) {
            this.leftPosition = leftPosition / Constants.TICKS_PER_FOOT_DB;
        }

        /**
         * Set the right position
         * 
         * @param rightPosition Usually TorqueSparkMax.getPosition()
         */
        public void setRightPosition(double rightPosition) {
            this.rightPosition = rightPosition / Constants.TICKS_PER_FOOT_DB;
        }

        /**
         * Set the left velocity
         * 
         * @param leftVelocity Usually TorqueSparkMax.getVelocity()
         */
        public void setLeftVelocity(double leftVelocity) {
            this.leftVelocity = leftVelocity;
        }

        /**
         * Set the right velocity
         * 
         * @param rightVelocity Usually TorqueSparkMax.getVelocity()
         */
        public void setRightVelocity(double rightVelocity) {
            this.rightVelocity = rightVelocity;
        }

        /**
         * @return The distance traveled on the left
         */
        public double getLeftDistance() {
            return -leftPosition + leftTare;
        }

        /**
         * @return The distance traveled on the right
         */
        public double getRightDistance() {
            return rightPosition - rightTare;
        }

        /**
         * @return The left velocity
         */
        public double getLeftVelocity() {
            return leftVelocity;
        }

        /**
         * @return The right velocity
         */
        public double getRightVelocity() {
            return rightVelocity;
        }

        /**
         * Reset the drive encoders (tare)
         */
        public void resetEncoders() {
            leftPosition = 0;
            rightPosition = 0;
            leftTare = leftPosition / Constants.TICKS_PER_FOOT_DB;
            rightTare = rightPosition / Constants.TICKS_PER_FOOT_DB;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[Feedback]leftDistance", getLeftDistance());
            SmartDashboard.putNumber("[Feedback]rightDistance", getRightDistance());
        }
    }

    // =====
    // Intake
    // =====
    public class IntakeFeedback extends TorqueFeedbackModule {
        private double rotaryPositionLeft;
        private double rotaryPositionRight;

        /**
         * Update the rotary position left
         * 
         * @param rotaryPositionLeft The position to set
         */
        public void setRotaryPositionLeft(double rotaryPositionLeft) {
            this.rotaryPositionLeft = rotaryPositionLeft;
        }

        /**
         * Update the rotary position right
         * 
         * @param rotaryPositionRight The position to set
         */
        public void setRotaryPositionRight(double rotaryPositionRight) {
            this.rotaryPositionRight = rotaryPositionRight;
        }

        /**
         * @return Rotary position left
         */
        public double getRotaryPositionLeft() {
            return rotaryPositionLeft;
        }

        /**
         * @return Rotary position right
         */
        public double getRotaryPositionRight() {
            return rotaryPositionRight;
        }
    }

    // =====
    // Shooter (Hood & Flywheel)
    // =====
    public class ShooterFeedback extends TorqueFeedbackModule {
        private double shooterVelocity;
        private double hoodPosition;

        /**
         * @return The shooter velocity
         */
        public double getShooterVelocity() {
            return shooterVelocity;
        }

        /**
         * @return The hood position
         */
        public double getHoodPosition() {
            return hoodPosition;
        }

        /**
         * Set the shooter velocity
         * 
         * @param shooterVelocity The velocity to set to
         */
        public void setShooterVelocity(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }

        /**
         * Set the hood position
         * 
         * @param hoodPosition The hood position to set
         */
        public void setHoodPosition(double hoodPosition) {
            this.hoodPosition = hoodPosition;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[Feedback]shooterVelocity", getShooterVelocity());
        }

    }

    // =====
    // Gyro
    // =====
    public class GyroFeedback extends TorqueFeedbackModule {
        private final AHRS NX_gyro;

        private double NX_pitch;
        private double NX_yaw;
        private double NX_roll;

        private GyroFeedback() {
            NX_gyro = new AHRS(SPI.Port.kMXP);
        }

        /**
         * Update the gyro values
         */
        @Override
        public void update() {
            NX_pitch = NX_gyro.getPitch();
            NX_yaw = NX_gyro.getYaw();
            NX_roll = NX_gyro.getRoll();
        }

        /**
         * Reset the gyro
         */
        public void resetNavX() {
            NX_gyro.reset();
        }

        /**
         * Set the current yaw offset as the baseline
         */
        public void zeroYaw() {
            NX_gyro.zeroYaw();
        }

        /**
         * @return The pitch of the gyro
         */
        public double getNX_pitch() {
            return NX_pitch;
        }

        /**
         * @return The yaw of the gyro
         */
        public double getNX_yaw() {
            return NX_yaw;
        }

        /**
         * @return The roll of the gyro
         */
        public double getNX_roll() {
            return NX_roll;
        }

        /**
         * @return The rotation of the gyro
         */
        public Rotation2d getRotation2d() {
            return NX_gyro.getRotation2d();
        }
    }

    // ====
    // Wheel of Fortune
    // ====
    public class WheelOfFortuneFeedback extends TorqueFeedbackModule {
        private Color detectedColor = Color.kBlack;

        @Override
        public void update() {
        }

        /**
         * @return Get the color detected by the sensor
         */
        public Color getDetectedColor() {
            return detectedColor;
        }

        public void setDetectedColor(Color color) {
            detectedColor = color;
        }

        @Override
        public void smartDashboard() {
            try {
                SmartDashboard.putString("[Feedback]detectedColor",
                        String.format("(%f, %f, %f)", detectedColor.red, detectedColor.green, detectedColor.blue));
            } catch (Exception e) {
                System.out.println("Failed to detect color!");
            }
        }
    }

    // =====
    // Get Modules
    // =====
    /**
     * @return The DriveTrainFeedback
     */
    public DriveTrainFeedback getDriveTrainFeedback() {
        return driveTrainFeedback;
    }

    /**
     * @return The IntakeFeedback
     */
    public IntakeFeedback getIntakeFeedback() {
        return intakeFeedback;
    }

    /**
     * @return The ShooterFeedback
     */
    public ShooterFeedback getShooterFeedback() {
        return shooterFeedback;
    }

    /**
     * @return The GyroFeedback
     */
    public GyroFeedback getGyroFeedback() {
        return gyroFeedback;
    }

    /**
     * @return The WheelOfFortuneFeedback
     */
    public WheelOfFortuneFeedback getWheelOfFortuneFeedback() {
        return wheelOfFortuneFeedback;
    }

    /**
     * Get the Feedback instance
     * 
     * @return Feedback
     */
    public static synchronized Feedback getInstance() {
        if (instance == null) {
            instance = new Feedback();
        }
        return instance;
    }
}