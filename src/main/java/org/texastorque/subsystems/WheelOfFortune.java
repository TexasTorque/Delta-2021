package org.texastorque.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

public class WheelOfFortune extends Subsystem {
    private static volatile WheelOfFortune instance;

    private static Feedback feedback = Feedback.getInstance();
    private static Input input = Input.getInstance();

    private Servo leftTurner = new Servo(Ports.WHEEL_OF_FORTUNE_LEFT);
    private Servo rightTurner = new Servo(Ports.WHEEL_OF_FORTUNE_RIGHT);

    private TorqueSparkMax wheel = new TorqueSparkMax(Ports.WHEEL_OF_FORTUNE);

    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final ColorMatch colorMatcher = new ColorMatch();

    // Uncalibrated color matches
    private final Color BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color GreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color YellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    // Default to black color
    private Color detectedColor = Color.kBlack;

    // Set points
    private final double setWheelPosition = .8; // TODO: Need to tune!
    private final double setWheelSpeed = .2;
    // Variables
    private double wheelPosition = 0;
    private double wheelSpeed = 0;
    private double timeStarting;
    private boolean executing = false;

    public WheelOfFortune() {
        // Add colors to match to
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);
    }

    @Override
    public void initTeleop() {
    };

    @Override
    public void initAuto() {
    };

    @Override
    public void runTeleop(RobotState state) {
        updateFeedback();

        // If a start signal is received and we aren't currently executing, start
        // executing
        if (input.getWheelOfFortuneInput().getStart() && !executing) {
            executing = true;
            wheelPosition = setWheelPosition;
            timeStarting = Timer.getFPGATimestamp();
        }
        // If a stop signal is received, stop executing
        if (input.getWheelOfFortuneInput().getDown()) {
            executing = false;
            wheelPosition = 0;
        }

        if (executing) {
            wheelSpeed = setWheelSpeed;

            // Wait for the rotary to be fully up. We don't have encoders I believe... so
            // this has to be time-based. TODO: Tune
            if (Timer.getFPGATimestamp() - timeStarting > 1.2) {
                // TODO: Make a way to decide what color to stop on....
                Color requestedColor = BlueTarget;
                Color detectedColor = colorSensor.getColor();
                // If we have reached the detected color, stop
                if (detectedColor.equals(requestedColor)) {
                    wheelSpeed = 0;
                    executing = false;
                }
            }
        } else {
            wheelSpeed = 0;
        }
        // lift attatchment
        // spin wheel slowly
        // if (color is right color){ stop wheel }

        output();
    };

    @Override
    public void runAuto(RobotState state) {
    };

    @Override
    protected void output() {
        leftTurner.set(wheelPosition);
        rightTurner.set(wheelPosition);
        wheel.set(wheelSpeed);
    };

    @Override
    protected void updateFeedback() {
        feedback.getWheelOfFortuneFeedback().setDetectedColor(detectedColor);
    };

    /**
     * Get the Climber instance
     * 
     * @return Climber
     */
    public static synchronized WheelOfFortune getInstance() {
        if (instance == null) {
            instance = new WheelOfFortune();
        }
        return instance;
    }
}