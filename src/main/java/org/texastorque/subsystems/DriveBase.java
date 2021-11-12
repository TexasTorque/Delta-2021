package org.texastorque.subsystems;

import org.texastorque.torquelib.base.TorqueSubsystem;

import com.revrobotics.ControlType;

import org.texastorque.constants.Constants;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.controlLoop.LowPassFilter;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.util.KPID;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivebase extends TorqueSubsystem {
    private static volatile Drivebase instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();
    private State state = State.getInstance();

    // Create the SparkMax motors
    private TorqueSparkMax DBLeft = new TorqueSparkMax(Ports.DB_LEFT_1);
    private TorqueSparkMax DBRight = new TorqueSparkMax(Ports.DB_RIGHT_1);

    // Variables to hold the current speed
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    // Values for vision
    private double position;
    private double pidValue;

    // PIDs
    // SAM NOTE
    private ScheduledPID linePid = new ScheduledPID.Builder(0, -1, 1, 1).setPGains(.01).setIGains(.005).setDGains(0)
            .build();
    private LowPassFilter lowPassFilter = new LowPassFilter(.5);
    private KPID leftDefaultPID = new KPID(2.29, 0, 0, 0, -1, 1);
    private KPID rightDefaultPID = new KPID(2.29, 0, 0, 0, -1, 1);

    private DifferentialDriveOdometry odometry;

    private Drivebase() {
        DBLeft.configurePID(leftDefaultPID);
        DBRight.configurePID(rightDefaultPID);
        DBLeft.addFollower(Ports.DB_LEFT_2);
        DBRight.addFollower(Ports.DB_RIGHT_2);
        DBLeft.setAlternateEncoder();
        DBLeft.disableVoltageCompensation();
        DBRight.disableVoltageCompensation();
        odometry = new DifferentialDriveOdometry(feedback.getGyroFeedback().getRotation2d());
    }

    @Override
    public void initTeleop() {
        leftSpeed = 0;
        rightSpeed = 0;
    }

    @Override 
    public void initAuto() {
    }

    @Override
    public void updateTeleop() {
        updateFeedback();
        if (state.getRobotState() == RobotState.AUTO) {
            leftSpeed = input.getDrivebaseInput().getLeftSpeed();
            rightSpeed = input.getDrivebaseInput().getRightSpeed();
        } else if (state.getRobotState() == RobotState.VISION) {
            runVision();
        } else if (state.getRobotState() == RobotState.TELEOP 
                || state.getRobotState() == RobotState.SHOOTING 
                || state.getRobotState() == RobotState.MAGLOAD) {
            runTeleopShootingMagload();
        }
    }

    @Override
    public void updateAuto() {

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

        if (Math.abs(feedback.getLimelightFeedback().getXOffset()) < .5) {
            SmartDashboard.putBoolean("[DB]LinedUp", true);
            // state.setRobotState(RobotState.TELEOP);
        } else {
            SmartDashboard.putBoolean("[DB]LinedUp", false);
        }
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

        double dbLeft = input.getDrivebaseInput().getLeftSpeed();
        double dbRight = input.getDrivebaseInput().getRightSpeed();
        leftSpeed = dbLeft < 0 ? ((dbLeft * dbLeft) * (-1)) * input.getDrivebaseInput().getSpeedMult() : ((dbLeft * dbLeft)) * input.getDrivebaseInput().getSpeedMult();
        rightSpeed = dbRight < 0 ? ((dbRight * dbRight) * (-1)) * input.getDrivebaseInput().getSpeedMult() : ((dbRight * dbRight)) * input.getDrivebaseInput().getSpeedMult();
    }

    @Override
    public void output() {
        DBLeft.set(leftSpeed);
        DBRight.set(rightSpeed);
    }

    @Override
    public void updateFeedbackTeleop() {
        feedback.getDriveTrainFeedback().setLeftPosition(DBLeft.getPosition());
        feedback.getDriveTrainFeedback().setRightPosition(DBRight.getPosition());
        feedback.getDriveTrainFeedback().setLeftVelocity(DBLeft.getVelocity());
        feedback.getDriveTrainFeedback().setRightVelocity(DBRight.getVelocity());
        odometry.update(feedback.getGyroFeedback().getRotation2d(),
                feedback.getDriveTrainFeedback().getLeftDistance() * Constants.FOOT_TO_METER,
                feedback.getDriveTrainFeedback().getRightDistance() * Constants.FOOT_TO_METER);
    }

    @Override
    public void updateFeedbackAuto() {
    }

    @Override
    public void updateSmartDashboard() {
    }

    /**
     * Set the zero values for the encoder to the current values
     */
    public void resetEncoders() {
        DBLeft.tareEncoder();
        DBRight.tareEncoder();
    }

    /**
     * Reset the odometry
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, feedback.getGyroFeedback().getRotation2d());
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

    public static synchronized Drivebase getInstance() {
        return (instance == null) ? instance = new Drivebase() : instance;
    }
}
