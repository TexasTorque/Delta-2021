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

public class Weaver5VelocityTesting extends Command {
    private final Supplier<Pose2d> getPose = DriveBase.getInstance()::getPose;
    private final RamseteController follower = new RamseteController(2, 0.7);
    private final Timer timer = new Timer();

    private Trajectory trajectory;

    public Weaver5VelocityTesting(double delay, String name) {
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
        input.getDriveBaseInput().setLeftSpeed(10);
        input.getDriveBaseInput().setRightSpeed(10);
    }

    @Override
    protected boolean endCondition() {
        return timer.hasElapsed(6);
    }

    @Override
    protected void end() {
        timer.stop();
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
        input.getDriveBaseInput().setDoingVelocity(false);
    }
}