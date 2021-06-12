package org.texastorque.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ControlType;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;
import edu.wpi.first.wpilibj.PWMSparkMax;
import org.texastorque.util.KPID;

public class Intake extends Subsystem {
    private static volatile Intake instance;

    // Cached instances
    private Input input = Input.getInstance();
    private Feedback feedback = Feedback.getInstance();

    // Speeds
    private double rotaryPositionLeft = 0;
    private double rotaryPositionRight = 0;
    private double rollerSpeed = 0;

    // Motors
    private TorqueSparkMax rotaryLeft = new TorqueSparkMax(Ports.INTAKE_ROTARY_LEFT);
    private TorqueSparkMax rotaryRight = new TorqueSparkMax(Ports.INTAKE_ROTARY_RIGHT);
    private VictorSPX rollers = new VictorSPX(Ports.INTAKE_ROLLERS);

    // PIDs
    private KPID kPIDRotaryLeft = new KPID(0.07, 0.00005, 0.00002, 0, -.7, .4);
    private KPID kPIDRotaryRight = new KPID(0.07, 0.00005, 0.00002, 0, -.4, .7);

    /**
     * Instantiate a new Intake
     */
    private Intake() {
        rotaryLeft.configurePID(kPIDRotaryLeft);
        rotaryRight.configurePID(kPIDRotaryRight);

        rotaryLeft.tareEncoder();
        rotaryRight.tareEncoder();
    }

    /**
     * Update the feedback values and set the motors
     */
    @Override
    public void runTeleop(RobotState state) {
        updateFeedback();

        rollerSpeed = input.getIntakeInput().getRollerSpeed();
        rotaryPositionLeft = input.getIntakeInput().getRotaryPositionLeft();
        rotaryPositionRight = input.getIntakeInput().getRotaryPositionRight();

        output();
    };

    /**
     * Uses the same code as Teleop
     */
    @Override
    public void runAuto(RobotState state) {
        runTeleop(state);
    };

    /**
     * Output to motors
     */
    @Override
    protected void output() {
        rollers.set(ControlMode.PercentOutput, rollerSpeed);
        // System.out.printf("(%f,%f)%n", rotaryPositionLeft, rotaryPositionRight);
        rotaryLeft.set(rotaryPositionLeft, ControlType.kPosition);
        rotaryRight.set(rotaryPositionRight, ControlType.kPosition);
    }

    @Override
    protected void updateFeedback() {
        feedback.getIntakeFeedback().setRotaryPositionLeft(rotaryLeft.getPosition());
        feedback.getIntakeFeedback().setRotaryPositionRight(rotaryRight.getPosition());
    }

    /**
     * Get the Intake instance
     * 
     * @return Intake
     */
    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}