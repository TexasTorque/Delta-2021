package org.texastorque.subsystems;

import com.revrobotics.ControlType;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.HoodSetpoint;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import org.texastorque.torquelib.component.TorqueTalon;
import org.texastorque.torquelib.controlLoop.ScheduledPID;
import org.texastorque.util.KPID;

public class Shooter extends Subsystem {
    private static volatile Shooter instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // PIDs
    private ScheduledPID shooterPID;
    private KPID hoodKPID = new KPID(0.1, 0, 0, 0, -1, 1);
    private KPID neoKPID = new KPID(0, 0, 0, 0, 0, 1);

    // Variables
    private double flywheelSpeed;
    private double pidOutput = 0;
    private double hoodSetpoint = HoodSetpoint.NEUTRAL.getValue();

    // Motors
    private TorqueTalon flywheel = new TorqueTalon(Ports.FLYWHEEL_LEAD);
    private TorqueSparkMax hood = new TorqueSparkMax(Ports.SHOOTER_HOOD);

    private Shooter() {
        flywheel.configurePID(neoKPID); // add PID to flywheel
        flywheel.addFollower(Ports.FLYWHEEL_FOLLOW);
        flywheel.invertFollower();

        hood.configurePID(hoodKPID); // add PID to hood
        hood.tareEncoder();

        shooterPID = new ScheduledPID.Builder(0, -1, 1, 1).setPGains(0.0003) // 0.002
                .setIGains(0).setDGains(0).setFGains(0.000150) // 0.000115
                .build();
    }

    public void runTeleop(RobotState state) {
        updateFeedback();

        flywheelSpeed = input.getShooterInput().getFlywheelSpeed();
        hoodSetpoint = input.getShooterInput().getHoodSetpoint();
        shooterPID.changeSetpoint(flywheelSpeed); // change target speed of flywheel to requested speed
        pidOutput = shooterPID.calculate(feedback.getShooterFeedback().getShooterVelocity());

        System.out.printf("%f, %f, %f%n", flywheelSpeed, flywheel.getRPM(),
        pidOutput);
        output();
    };

    public void runAuto(RobotState state) {
        updateFeedback();

        // Equivalent to runTeleop
        flywheelSpeed = input.getShooterInput().getFlywheelSpeed();
        hoodSetpoint = input.getShooterInput().getHoodSetpoint();
        shooterPID.changeSetpoint(flywheelSpeed); // change target speed of flywheel to requested speed
        // System.out.printf("%f, %f%n", flywheelSpeed, flywheel.getRPM());
        pidOutput = shooterPID.calculate(feedback.getShooterFeedback().getShooterVelocity());

        output();
    };

    protected void output() {
        hood.set(hoodSetpoint, ControlType.kPosition);
        pidOutput = Math.max(0, pidOutput);
        flywheel.set(pidOutput);
    };

    protected void updateFeedback() {
        feedback.getShooterFeedback().setHoodPosition(hood.getPosition());
        feedback.getShooterFeedback().setShooterVelocity(flywheel.getRPM());
        // System.out.printf("Shooter: %f%n",flywheel.getRPM());
    };

    /**
     * Get the Shooter instance
     * 
     * @return Shooter
     */
    public static synchronized Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
}
