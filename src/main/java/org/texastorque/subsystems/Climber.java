package org.texastorque.subsystems;

import org.texastorque.constants.Ports;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.ClimberState;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {
    private static volatile Climber instance;

    // Cached instances
    private Input input = Input.getInstance();

    // Motors
    private TorqueSparkMax climberLeft = new TorqueSparkMax(Ports.CLIMBER_LEFT);
    private TorqueSparkMax climberRight = new TorqueSparkMax(Ports.CLIMBER_RIGHT);

    private Servo leftRatchet = new Servo(Ports.LEFT_RATCHET);
    private Servo rightRatchet = new Servo(Ports.RIGHT_RATCHET);

    // Vars
    private ClimberState climbStatus = ClimberState.NEUTRAL;
    private double startTime = 0;

    private boolean notStarted = true;
    private boolean inReverse = false;

    private double climberLeftSpeed = 0;
    private double climberRightSpeed = 0;

    private double leftRatchetPos = 0;
    private double rightRatchetPos = 0.15;

    /**
     * Resets climber speed to zero when teleop is actived
     */
    @Override
    public void initTeleop() {
        climbStatus = ClimberState.NEUTRAL;
        climberLeftSpeed = 0;
        climberRightSpeed = 0;
    }

    /**
     * Decides climber motor and servo parameters
     */
    @Override
    public void runTeleop(RobotState state) {
        climbStatus = input.getClimberInput().getClimberStatus(); // climber neutral, extend, retract
        if (!input.getClimberInput().getManualClimb()) { // if climbing is not manual...
            switch (climbStatus) {
                case RETRACT:
                    climberLeftSpeed = 0.3;
                    climberRightSpeed = -0.3;
                    leftRatchetPos = 0;
                    rightRatchetPos = 0.15;
                    break;
                case NEUTRAL:
                    climberLeftSpeed = 0;
                    climberRightSpeed = 0;
                    break;
                case EXTEND:
                    if (notStarted) { // if extension has not started
                        leftRatchetPos = 0.15;
                        rightRatchetPos = 0;
                        notStarted = false;
                        inReverse = true;
                        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                    } else if (inReverse) { // if in reverse...
                        if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime < .1) { // if climb started less
                                                                                               // than 0.1 seconds ago
                            climberLeftSpeed = 0.1;
                            climberRightSpeed = -0.1;
                        } else { // after 0.1 seconds, stop reversing
                            inReverse = false;
                        }
                    } else { // if started, just set speed to extend
                        climberLeftSpeed = -0.3;
                        climberRightSpeed = 0.3;
                    }
                    break;
            }
        } else { // if in manual climb mode ...
            switch (input.getClimberInput().getSideToExtend()) { // check the side requested to shift to
                case LEFT:
                    climberLeftSpeed = -0.3;
                    climberRightSpeed = 0;
                    break;
                case NEUTRAL:
                    climberLeftSpeed = 0;
                    climberRightSpeed = 0;
                    break;
                case RIGHT:
                    climberLeftSpeed = 0;
                    climberRightSpeed = 0.3;
                    break;
            }
        }

        output();
    }

    public void resetClimb() {
        notStarted = true;
        inReverse = false;
    }

    /**
     * set sparkmaxes with outputs from runTeleop
     */
    protected void output() {
        climberLeft.set(climberLeftSpeed);
        climberRight.set(climberRightSpeed);
        leftRatchet.set(leftRatchetPos);
        rightRatchet.set(rightRatchetPos);
        // leftRatchet.set(0.15);
        // rightRatchet.set(0.15);

    };

    @Override
    public void smartDashboard() {
        SmartDashboard.putNumber("[Climber]leftSpeed", climberLeftSpeed);
        SmartDashboard.putNumber("[Climber]rightSpeed", climberRightSpeed);

    }

    /**
     * Get the Climber instance
     * 
     * @return Climber
     */
    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }
}
