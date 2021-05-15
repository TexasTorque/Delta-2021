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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class PathWeaver extends Command {
    private final Timer timer = new Timer();
    
    private Trajectory trajectory;
    private double previousTime;
    private DifferentialDriveWheelSpeeds previousSpeeds;

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
        feedback.getDriveTrainFeedback().resetEncoders();
        previousTime = -1;
        Trajectory.State initialState = trajectory.sample(0);
        timer.start();
    }

    @Override
    protected void continuous() {
        double curTime = timer.get();
        double dt = curTime - previousTime;

        if(previousTime < 0) {
            previousTime = curTime;
            return;
        }

        // DifferentialDriveWheelSpeeds targetWheelSpeeds = 
            // Constants.kDriveKinematics.toWheelSpeeds(follower.calculate(getPose.get(), trajectory.sample(curTime)));
        // System.out.printf("Target Wheel Speed: (%f,%f)%n", targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);
        // double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        // double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        
        // // if not using PID
        // double leftOutput = leftSpeedSetpoint;
        // double rightOutput = rightSpeedSetpoint;
        
        // previousSpeeds = targetWheelSpeeds;
        // previousTime = curTime;
    }

    @Override
    protected boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected void end() {
        timer.stop();
        // outputVolts.accept(0.0, 0.0);
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
    }
}
