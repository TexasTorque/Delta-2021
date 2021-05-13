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
    private final BiConsumer<Double, Double> outputVolts = input.getDriveBaseInput()::setVolts;
    private final Supplier<Pose2d> getPose = DriveBase.getInstance()::getPose;
    private final RamseteController follower = new RamseteController(2, 0.7);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
    private final Supplier<DifferentialDriveWheelSpeeds> speeds = DriveBase.getInstance()::getWheelSpeeds;
    private final Timer timer = new Timer();
    
    private PIDController leftController = new PIDController(Constants.kPDriveVel,0,0);
    private PIDController rightController = new PIDController(Constants.kPDriveVel,0,0);

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
        input.getDriveBaseInput().setDoingAutoVolts(true);
        previousTime = -1;
        Trajectory.State initialState = trajectory.sample(0);
        previousSpeeds = Constants.kDriveKinematics.toWheelSpeeds(
            new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
            initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        timer.reset();
        timer.start();
        leftController.reset();
        rightController.reset();
    }

    @Override
    protected void continuous() {
        double curTime = timer.get();
        double dt = curTime - previousTime;

        if(previousTime < 0) {
            outputVolts.accept(0.0, 0.0);
            previousTime = curTime;
            return;
        }

        DifferentialDriveWheelSpeeds targetWheelSpeeds = 
            Constants.kDriveKinematics.toWheelSpeeds(follower.calculate(getPose.get(), trajectory.sample(curTime)));
        System.out.printf("Target Wheel Speed: (%f,%f)%n", targetWheelSpeeds.leftMetersPerSecond, targetWheelSpeeds.rightMetersPerSecond);
        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        
        double leftFeedforward = feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - previousSpeeds.leftMetersPerSecond) / dt);
        double rightFeedforward = feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - previousSpeeds.rightMetersPerSecond) / dt);

        double leftOutput = leftFeedforward + leftController.calculate(speeds.get().leftMetersPerSecond, leftSpeedSetpoint);
        double rightOutput = rightFeedforward + rightController.calculate(speeds.get().rightMetersPerSecond, rightSpeedSetpoint);

        
        // // if not using PID
        // double leftOutput = leftSpeedSetpoint;
        // double rightOutput = rightSpeedSetpoint;
        
        outputVolts.accept(leftOutput, rightOutput);
        previousSpeeds = targetWheelSpeeds;
        previousTime = curTime;
    }

    @Override
    protected boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected void end() {
        timer.stop();
        outputVolts.accept(0.0, 0.0);
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
        input.getDriveBaseInput().setDoingAutoVolts(false);
    }
}
