package org.texastorque.auto.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.texastorque.auto.Command;
import org.texastorque.constants.Constants;
import org.texastorque.subsystems.DriveBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class PathWeaver4 extends Command {
    private final Supplier<Pose2d> getPose = DriveBase.getInstance()::getPose;
    private final RamseteController follower = new RamseteController(2, 0.7);
    private final Timer timer = new Timer();

    private Trajectory trajectory;

    public PathWeaver4(double delay, String name) {
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
        feedback.getDriveTrainFeedback().resetEncoders();
        input.getDriveBaseInput().setDoingVelocity(true);
        timer.reset();
        timer.start();
    }

    @Override
    protected void continuous() {
        double curTime = timer.get();

        // Possible error: getPose is returning negative left.
        DifferentialDriveWheelSpeeds targetWheelSpeeds = 
            Constants.kDriveKinematics.toWheelSpeeds(follower.calculate(getPose.get(), trajectory.sample(curTime)));
        
        System.out.println(getPose.get() + "\n" + trajectory.sample(curTime));
        System.out.printf("Target Wheel Speed: (%f,%f)%n", targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);
        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        
        
        double leftOutput = leftSpeedSetpoint; // m/s
        double rightOutput = rightSpeedSetpoint; // m/s

        Trajectory.State state = trajectory.sample(curTime);
        input.getDriveBaseInput().setLeftSpeed(state.velocityMetersPerSecond);
        input.getDriveBaseInput().setRightSpeed(-state.velocityMetersPerSecond);
    }

    @Override
    protected boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected void end() {
        timer.stop();
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
        input.getDriveBaseInput().setDoingVelocity(false);
    }
}