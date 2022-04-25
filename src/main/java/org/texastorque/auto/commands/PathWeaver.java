package org.texastorque.auto.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import org.texastorque.auto.Command;
import org.texastorque.constants.Constants;
import org.texastorque.subsystems.DriveBase;
import org.texastorque.util.KPID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class PathWeaver extends Command {
    private final Supplier<Pose2d> getPose = DriveBase.getInstance()::getPose;
    private final RamseteController follower = new RamseteController(2.74, 0.7);
    private final Timer timer = new Timer();

    private Trajectory trajectory;

    public PathWeaver(double delay, String name) {
        super(delay);
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/"+name);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException e) {
            DriverStation.reportError("Failed to read "+name, false);
        }
    }

    @Override
    protected void init() {
        DriveBase.getInstance().resetOdometry(trajectory.getInitialPose());
        DriveBase.getInstance().setKPID(new KPID(2.74, 0, 0, 0, -1, 1), new KPID(2.74, 0, 0, 0, -1, 1));
        feedback.getDriveTrainFeedback().resetEncoders();
        input.getDriveBaseInput().setDoingVelocity(true);
        timer.reset();
        timer.start();
    }

    @Override
    protected void continuous() {
        double curTime = timer.get();


        // Note:
        // The starting position is set to a non-zero value on the field.
        // Possible solution: Find first offset, use to offset in position.

        DifferentialDriveWheelSpeeds targetWheelSpeeds = 
            Constants.kDriveKinematics.toWheelSpeeds(follower.calculate(getPose.get(), trajectory.sample(curTime)));
        
        System.out.println("CURRENT POSE: "+getPose.get());
        System.out.println("TARGET: "+trajectory.sample(curTime));
        System.out.printf("Target Wheel Speed: (%f,%f)%n", targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);
        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        
        
        double leftOutput = leftSpeedSetpoint; // m/s
        double rightOutput = rightSpeedSetpoint; // m/s

        input.getDriveBaseInput().setLeftSpeed(leftOutput);
        input.getDriveBaseInput().setRightSpeed(rightOutput);
    }

    @Override
    protected boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected void end() {
        timer.stop();
        DriveBase.getInstance().setDefaultKPID();
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
        input.getDriveBaseInput().setDoingVelocity(false);
    }
}