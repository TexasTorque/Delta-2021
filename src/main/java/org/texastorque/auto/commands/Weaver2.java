package org.texastorque.auto.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;

import org.texastorque.auto.Command;
import org.texastorque.constants.Constants;
import org.texastorque.subsystems.DriveBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class Weaver2 extends Command {
    private Trajectory trajectory;
    private DriveBase db = DriveBase.getInstance();

    private RamseteController ramseteController = new RamseteController(2.0, .7);
    public Weaver2(double delay, String name) {
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
        db.resetOdometry(db.getPose());
        feedback.getDriveTrainFeedback().resetEncoders();
    }

    @Override
    protected void continuous() {
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, db::getPose, ramseteController, Constants.kDriveKinematics, (BiConsumer<Double, Double>) (Double leftSpeed, Double rightSpeed)->{
            System.out.printf("(%f,%f)%n", leftSpeed, rightSpeed);
        });

        ramseteCommand.andThen(()->{System.out.println("done");});
    }

    @Override
    protected boolean endCondition() {
        return true;
    }

    @Override
    protected void end() {
        // TODO Auto-generated method stub

    }
    
}
