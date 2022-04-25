package org.texastorque.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
        flywheel.configurePID(new KPID(.0001, 0, 0, .0015, -1, 1));
        flywheel.invertFollower();

        hood.configurePID(hoodKPID); // add PID to hood
        hood.tareEncoder();
    }

    public void runTeleop(RobotState state) {
        updateFeedback();

        flywheelSpeed = input.getShooterInput().getFlywheelSpeed();
        hoodSetpoint = input.getShooterInput().getHoodSetpoint();

        output();
    };

    public void runAuto(RobotState state) {
        updateFeedback();
        // Equivalent to runTeleop
        flywheelSpeed = input.getShooterInput().getFlywheelSpeed();
        hoodSetpoint = input.getShooterInput().getHoodSetpoint();
        output();
    };

    protected void output() {
        // hood.set(hoodSetpoint, com.revrobotics.CANSparkMax.ControlType.kPosition);
        flywheel.set(flywheelSpeed / 7000.);
    };

    protected void updateFeedback() {
        feedback.getShooterFeedback().setHoodPosition(hood.getPosition());
        feedback.getShooterFeedback().setShooterVelocity(flywheel.getRPM());
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
