package org.texastorque.subsystems;

import org.texastorque.constants.Constants;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.util.KPID;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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


    // PIDs
    // SAM NOTE
    private ScheduledPID linePid = new ScheduledPID.Builder(0, -1, 1, 1).setPGains(.01).setIGains(.005).setDGains(0)
            .build();
    private LowPassFilter lowPassFilter = new LowPassFilter(.5);
    private KPID leftDefaultPID = new KPID(2.29, 0, 0, 0, -1, 1);
    private KPID rightDefaultPID = new KPID(2.29, 0, 0, 0, -1, 1);

    private DifferentialDriveOdometry odometry;

    /**
     * Instantiate a new DriveBase
     */
    private DriveBase() {
        DBLeft.configurePID(leftDefaultPID);
        DBRight.configurePID(rightDefaultPID);
        DBLeft.addFollower(Ports.DB_LEFT_2);
        DBRight.addFollower(Ports.DB_RIGHT_2);
        DBLeft.setAlternateEncoder();
        DBLeft.disableVoltageCompensation();
        DBRight.disableVoltageCompensation();
        odometry = new DifferentialDriveOdometry(feedback.getGyroFeedback().getRotation2d());
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
    public void runTeleop() {
        updateFeedback();
        runTeleopShootingMagload();
        output();
    }
    /**
     * Output the values
     */
    @Override
    protected void output() {
        DBLeft.set(leftSpeed * Constants.DB_E_SPEED_MULTIPLIER);
        DBRight.set(rightSpeed * Constants.DB_E_SPEED_MULTIPLIER);
    }

    /**
     * Code for teleop/shooting/magload
     */
    private void runTeleopShootingMagload() {
        SmartDashboard.putBoolean("[DB]vision", true);

        linePid.reset();
        linePid.setLastError(0);
        lowPassFilter.clear();

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
        odometry.update(feedback.getGyroFeedback().getRotation2d(),
                feedback.getDriveTrainFeedback().getLeftDistance() * Constants.FOOT_TO_METER,
                feedback.getDriveTrainFeedback().getRightDistance() * Constants.FOOT_TO_METER);
    }

    /**
     * Set the zero values for the encoder to the current values
     */
    public void resetEncoders() {
        DBLeft.tareEncoder();
        DBRight.tareEncoder();
    }

    /**
     * Use the default PIDs for DriveBase (also the vision ones!)
     */
    public void setDefaultKPID() {
        DBLeft.updatePID(leftDefaultPID);
        DBRight.updatePID(rightDefaultPID);
    }

    /**
     * Set the DriveBase PIDS
     * 
     * @param left
     * @param right
     */
    public void setKPID(KPID left, KPID right) {
        DBLeft.updatePID(left);
        DBRight.updatePID(right);
    }

    /**
     * @deprecated should not be used
     * @return Wheel speeds for odometry
     */
    /*
     * public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : check if
     * need one negative return new
     * DifferentialDriveWheelSpeeds(feedback.getDriveTrainFeedback().getLeftDistance
     * ()*0.0653176971,
     * feedback.getDriveTrainFeedback().getRightVelocity()*0.0653176971); }
     */

    /**
     * @return The left drive distance
     */
    public double getLeftDistance() {
        return DBLeft.getPosition();
    }

    /**
     * @return Current pose of odometry
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return The right drive distance
     */
    public double getRightDistance() {
        return DBRight.getPosition();
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
        SmartDashboard.putNumber("[DB]odometryX", getPose().getX());
        SmartDashboard.putNumber("[DB]odometryY", getPose().getY());
        SmartDashboard.putNumber("[DB]odometryRadians", getPose().getRotation().getRadians());
        SmartDashboard.putNumber("[DB]odometryDegrees", getPose().getRotation().getDegrees());

    }

    /**
     * Get the DriveBase instance
     * 
     * @return DriveBase
     */
    public static synchronized DriveBase getInstance() {
        if (instance == null) {
            instance = new DriveBase();
        }
        return instance;
    }
}
