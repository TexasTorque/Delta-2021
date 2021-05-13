package org.texastorque.inputs;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import org.texastorque.constants.Constants;
import org.texastorque.constants.Ports;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feedback {
    private static volatile Feedback instance;
    
    // Cached modules
    private static DriveTrainFeedback driveTrainFeedback;
    private static IntakeFeedback intakeFeedback;
    private static ShooterFeedback shooterFeedback;
    private static MagazineFeedback magazineFeedback;
    private static LimelightFeedback limelightFeedback;
    private static GyroFeedback gyroFeedback;

    private static ArrayList<TorqueFeedbackModule> modules;

    /**
     * Load in the modules
     */
    private Feedback() {
        driveTrainFeedback = new DriveTrainFeedback();
        intakeFeedback = new IntakeFeedback();
        shooterFeedback = new ShooterFeedback();
        magazineFeedback = new MagazineFeedback();
        limelightFeedback = new LimelightFeedback();
        gyroFeedback = new GyroFeedback();

        modules = new ArrayList<>();
        modules.add(driveTrainFeedback);
        modules.add(intakeFeedback);
        modules.add(shooterFeedback);
        modules.add(magazineFeedback);
        modules.add(limelightFeedback);
        modules.add(gyroFeedback);
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
         * @param leftPosition Usually TorqueSparkMax.getPosition()
         */
        public void setLeftPosition(double leftPosition) {
            this.leftPosition = leftPosition / Constants.TICKS_PER_FOOT_DB;
        }
        
        /**
         * Set the right position
         * @param rightPosition Usually TorqueSparkMax.getPosition()
         */
        public void setRightPosition(double rightPosition) {
            this.rightPosition = rightPosition / Constants.TICKS_PER_FOOT_DB;
        }
        
        /**
         * Set the left velocity
         * @param leftVelocity Usually TorqueSparkMax.getVelocity()
         */
        public void setLeftVelocity(double leftVelocity) {
            this.leftVelocity = leftVelocity; 
        }
        
        /**
         * Set the right velocity 
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
    }

    // =====
    // Intake
    // =====
    public class IntakeFeedback extends TorqueFeedbackModule {
        private double rotaryPositionLeft;
        private double rotaryPositionRight;

        /**
         * Update the rotary position left
         * @param rotaryPositionLeft The position to set
         */
        public void setRotaryPositionLeft(double rotaryPositionLeft) {
            this.rotaryPositionLeft = rotaryPositionLeft;
        }

        /**
         * Update the rotary position right
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
         * @param shooterVelocity The velocity to set to 
         */
        public void setShooterVelocity(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }

        /**
         * Set the hood position 
         * @param hoodPosition The hood position to set
         */
        public void setHoodPosition(double hoodPosition) {
            this.hoodPosition = hoodPosition;
        }
    }

    // =====
    // Magazine
    // =====
    public class MagazineFeedback extends TorqueFeedbackModule {
        private DigitalInput magHighCheck;
        private DigitalInput magLowCheck;
        
        private boolean highMag = false;
        private boolean lowMag = false;
        private boolean highMagPast = false;

        private boolean ballLast;
        private int count;

        private MagazineFeedback() {
            magHighCheck = new DigitalInput(Ports.MAG_SENSOR_HIGH);
            magLowCheck = new DigitalInput(Ports.MAG_SENSOR_LOW);
        }

        @Override
        public void update() {
            highMag = magHighCheck.get();
            lowMag = magLowCheck.get();
            
            if(ballLast != lowMag) {
                if(!lowMag) count++;
                ballLast = lowMag;
            }

            if(!highMagPast && !highMag) highMagPast = true;
        }

        /**
         * Reset the count and highMagPast
         */
        public void resetCount() {
            count = 0;
            highMagPast = false;
        }

        /**
         * @return The ball count
         */
        public int getCount() {
            return count;
        }

        /**
         * @return The high mag past
         */
        public boolean getMagHighPast() {
            return highMagPast;
        }

        /**
         * @return True if seeing a ball in the high mag
         */
        public boolean getMagHigh() {
            return highMag;
        }

        /**
         * @return True if seeing a ball in the low mag
         */
        public boolean getMagLow() {
            return lowMag;
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putBoolean("[Feedback]MagHigh", magHighCheck.get());
            SmartDashboard.putBoolean("[Feedback]MagLow", magLowCheck.get());
        }
    }

    // ====
    // Limelight
    // =====
    public class LimelightFeedback extends TorqueFeedbackModule {
        private double targetArea;
        private double hOffset;
        private double vOffset;

        /**
         * Update Limelight readings
         */
        @Override
        public void update() {
            getLimelightTable().getEntry("pipeline").setNumber(0);
            targetArea = getLimelightTable().getEntry("ta").getDouble(0);
            hOffset = getLimelightTable().getEntry("tx").getDouble(0);
            vOffset = getLimelightTable().getEntry("ty").getDouble(0);
        }

        /**
         * Forcefully set the limelight ledMode
         * @param on If it should be on or not
         */
        public void setLimelightOn(boolean on) {
            if(on) {
                getLimelightTable().getEntry("ledMode").forceSetNumber(3);
            } else {
                getLimelightTable().getEntry("ledMode").forceSetNumber(1);
            }
        }

        /**
         * @return The hOffset of the Limelight
         */
        public double getXOffset() {
            return hOffset;
        }

        /**
         * @return The vOffset of the Limelight
         */
        public double getYOffset() {
            return vOffset;
        }

        /**
         * @return The distance away from the centerpoint
         */
        public double getDistanceAway() {
            return Constants.DIFFERENCE_CENTERPORT_LIMELIGHT / Math.tan(Math.toRadians(vOffset + Constants.LIMELIGHT_ANGLE_OFFSET));
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[Feedback]hOffset", hOffset);
            SmartDashboard.putNumber("[Feedback]vOffset", vOffset);
        }

        /**
         * Internally get the Limelight table
         * @return The Limelight table
         */
        private NetworkTable getLimelightTable() {
            return NetworkTableInstance.getDefault().getTable("limelight");
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
     * @return The MagazineFeedback
     */
    public MagazineFeedback getMagazineFeedback() {
        return magazineFeedback;
    }
    
    /**
     * @return The LimelightFeedback
     */
    public LimelightFeedback getLimelightFeedback() {
        return limelightFeedback;
    }

    /**
     * @return The GyroFeedback
     */
    public GyroFeedback getGyroFeedback() {
        return gyroFeedback;
    }

    /**
     * Get the Input instance
     * @return Input
     */
    public static synchronized Feedback getInstance() {
        if(instance == null) {
            instance = new Feedback();
        }
        return instance;
    }
}