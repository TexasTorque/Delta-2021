package org.texastorque.inputs;

import com.kauailabs.navx.frc.AHRS;

import org.texastorque.constants.Constants;

import edu.wpi.first.wpilibj.SPI;

public class Feedback {
    private static volatile Feedback instance;
    
    // Conversions
    public final double DISTANCE_PER_PULSE = Math.PI * Constants.WHEEL_DIAMETER / Constants.PULSES_PER_ROTATION;
    public final double ANGLE_PER_PULSE = 360.0 / Constants.PULSES_PER_ROTATION;
    public final double LF_FEET_CONVERSION = Math.PI * (1.0 / 20) / Constants.PULSES_PER_ROTATION; // Using approximate                                                                                             // shaft diameter
    public final double ULTRA_CONVERSION = 1.0 / 84;

    // Cached modules
    private static DriveTrainFeedback driveTrainFeedback;

    private final AHRS NX_gyro;

    private Feedback() {
        // Load modules
        driveTrainFeedback = new DriveTrainFeedback();

        NX_gyro = new AHRS(SPI.Port.kMXP);

    }

    // =====
    // Drive Train
    // =====
    public class DriveTrainFeedback implements TorqueFeedbackModule {
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
        public double getrightDistance() {
            return -rightPosition + rightTare;
        }
        
        
    }

    // =====
    // Get Modules
    // =====
    public DriveTrainFeedback getDriveTrainFeedback() {
        return driveTrainFeedback;
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