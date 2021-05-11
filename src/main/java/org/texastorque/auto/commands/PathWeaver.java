package org.texastorque.auto.commands;

import java.io.IOException;
import java.nio.file.Path;

import org.texastorque.auto.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class PathWeaver extends Command {
    private Trajectory trajectory;
    private double time;
    private int on;

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
        time = Timer.getFPGATimestamp();
        on = 0;
    }

    @Override
    protected void continuous() {
        double newTime = Timer.getFPGATimestamp();
        Trajectory.State state = trajectory.getStates().get(on);
    }

    @Override
    protected boolean endCondition() {
        return on == trajectory.getStates().size();
    }

    @Override
    protected void end() {
        input.getDriveBaseInput().setLeftSpeed(0);
        input.getDriveBaseInput().setRightSpeed(0);
    }
    
}
