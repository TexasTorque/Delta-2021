package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends Subsystem {
    public static volatile DriveBase instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // Create the SparkMax motors
    private TorqueSparkMax DBLeft = new TorqueSparkMax(Ports.DB_LEFT_1);
    private TorqueSparkMax DBRight = new TorqueSparkMax(Ports.DB_RIGHT_1);

    // Variables to hold the current speed
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double speedMult = .55;

    // Values for vision
    private double position;
    private double pidValue;

    // PIDs
    private ScheduledPID linePid = new ScheduledPID.Builder(0, -1, 1, 1)
        .setPGains(.01)
        .setIGains(.005)
        .setDGains(.000005)
        .build();
    private LowPassFilter lowPassFilter = new LowPassFilter(.5);

    /**
     * Instantiate a new DriveBase
     */
    private DriveBase() {
        DBLeft.addFollower(Ports.DB_LEFT_2);
        DBRight.addFollower(Ports.DB_RIGHT_2);
    }

    /**
     * Reset the encoders when auto is initialized
     */
    @Override
    public void initAuto() {
        resetEncoders();
    }

    /**
     * Reset the speeds when teleop is initialized
     */
    @Override
    public void initTeleop() {
        leftSpeed = 0;
        rightSpeed = 0;
    }
    
    /**
     * Move during telepo
     */
    @Override
    public void runTeleop(RobotState state) {
        updateFeedback();
        
        if(state == RobotState.AUTO) {
            leftSpeed = input.getDriveBaseInput().getLeftSpeed();    
            rightSpeed = input.getDriveBaseInput().getRightSpeed();
        } else if (state == RobotState.VISION) {
            runVision();
        } else if (state == RobotState.TELEOP || state == RobotState.SHOOTING || state == RobotState.MAGLOAD) {
            runTeleopShootingMagload();
        }

        output();
    }

    /**
     * Output the values
     */
    @Override
    protected void output() {
        DBLeft.set(leftSpeed);
        DBRight.set(rightSpeed);
    }

    /**
     * Code for teleop vision
     */
    private void runVision() {
        SmartDashboard.putBoolean("[DB]vision", true);
        SmartDashboard.putNumber("[DB]hOffset", feedback.getLimelightFeedback().getXOffset());
        SmartDashboard.putBoolean("[DB]linedUp", feedback.getLimelightFeedback().getXOffset() < 3);

        feedback.getLimelightFeedback().setLimelightOn(true);
        position = lowPassFilter.filter(-feedback.getLimelightFeedback().getXOffset());
        pidValue = -linePid.calculate(position);
        leftSpeed = pidValue;
        rightSpeed = pidValue;
    }

    /**
     * Code for teleop/shooting/magload
     */
    private void runTeleopShootingMagload() {
        SmartDashboard.putBoolean("[DB]vision", true);
        feedback.getLimelightFeedback().setLimelightOn(false);

        linePid.reset();
        linePid.setLastError(0);
        lowPassFilter.clear();

        feedback.getLimelightFeedback().setLimelightOn(true);
        
        double dbLeft = input.getDriveBaseInput().getLeftSpeed();
        double dbRight = input.getDriveBaseInput().getRightSpeed();
        leftSpeed = dbLeft < 0 ? ((dbLeft * dbLeft) * (-1)) * speedMult : ((dbLeft * dbLeft)) * speedMult;
        rightSpeed = dbRight < 0 ? ((dbRight * dbRight) * (-1)) * speedMult : ((dbRight * dbRight)) * speedMult;
    }

    /**
     * Update the feedback positions
     */
    @Override
    protected void updateFeedback() {
        feedback.getDriveTrainFeedback().setLeftPosition(DBLeft.getPosition());
        feedback.getDriveTrainFeedback().setRightPosition(DBRight.getPosition());
        feedback.getDriveTrainFeedback().setLeftVelocity(DBLeft.getVelocity());
        feedback.getDriveTrainFeedback().setRightVelocity(DBRight.getVelocity());
    }
    
    /**
     * Set the zero values for the encoder to the current values
     */
    public void resetEncoders() {
        DBLeft.tareEncoder();
        DBRight.tareEncoder();
    }

    /**
     * @return The left drive distance
     */
    public double getLeftDistance() {
        return -(DBLeft.getPosition());
    }

    /**
     * @return The right drive distance
     */
    public double getRightDistance() {
        return -(DBRight.getPosition());
    }
    
    /**
     * Update values in SmartDashboard
     */
    @Override
    public void smartDashboard() {
        // Speeds
        SmartDashboard.putNumber("[DB]leftSpeed", leftSpeed);
        SmartDashboard.putNumber("[DB]rightSpeed", rightSpeed); 
        // Distance
        SmartDashboard.putNumber("[DB]leftDistance", getLeftDistance()); 
        SmartDashboard.putNumber("[DB]rightDistance", getRightDistance()); 
    }

    /**
     * Get the DriveBase instance
     * @return DriveBase
     */
    public static synchronized DriveBase getInstance() {
        if(instance == null) {
            instance = new DriveBase();
        }
        return instance;
    }
}
