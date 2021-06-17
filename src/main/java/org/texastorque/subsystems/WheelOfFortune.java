package org.texastorque.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.Feedback;
import org.texastorque.inputs.Input;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final Color BlueTarget = ColorMatch.makeColor(0.1238, 0.418, 0.458);
    private final Color GreenTarget = ColorMatch.makeColor(0.169, 0.581, 0.249);
    private final Color RedTarget = ColorMatch.makeColor(0.536, 0.336, 0.127);
    private final Color YellowTarget = ColorMatch.makeColor(0.3295, 0.557, 0.113);

    // Default to black color
    private Color detectedColor = Color.kBlack;

    // Set points
    private final double setWheelPositionLeftDown = 1;
    private final double setWheelPositionLeftUp = .38;
    private final double setWheelPositionRightDown = 0;
    private final double setWheelPositionRightUp = .64;
    private final double setWheelSpeed = .5;
    private final double setWheelSpeedSlow = .3;

    // Variables
    private double wheelPositionLeft = setWheelPositionLeftDown;
    private double wheelPositionRight = setWheelPositionRightDown;
    private double wheelSpeed = 0;
    private double timeStarting;
    private boolean executing = false;
    private int passes = 0;
    private Color lastColor = Color.kBlack;

    public WheelOfFortune() {
        // Add colors to match to
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);
        colorMatcher.addColorMatch(Color.kBlack);
    }

    @Override
    public void initTeleop() {
    };

    @Override
    public void initAuto() {
    };

    public boolean getExecuting() {
        return executing;
    }

    @Override
    public void runTeleop(RobotState state) {
        updateFeedback();
        detectedColor = colorSensor.getColor();
        // If a start signal is received and we aren't currently executing, start
        // executing
        if (input.getWheelOfFortuneInput().getStart() && !executing) {
            executing = true;
            wheelPositionLeft = setWheelPositionLeftUp;
            wheelPositionRight = setWheelPositionRightUp;
            timeStarting = Timer.getFPGATimestamp();
            passes = 0;
        }
        // If a stop signal is received, stop executing
        if (input.getWheelOfFortuneInput().getDown()) {
            executing = false;
            wheelPositionLeft = setWheelPositionLeftDown;
            wheelPositionRight = setWheelPositionRightDown;
        }

        if (executing) {
            wheelSpeed = passes == 5 ? setWheelSpeedSlow : setWheelSpeed;

            // TODO: Tune
            if (Timer.getFPGATimestamp() - timeStarting > .5) {
                String gameData = DriverStation.getInstance().getGameSpecificMessage();
                if (gameData.length() > 0) {
                    Color requestedColor = Color.kBlack;
                    switch (gameData.charAt(0)) {
                        case 'B':
                            requestedColor = RedTarget;
                            break;
                        case 'G':
                            requestedColor = YellowTarget;
                            break;
                        case 'R':
                            requestedColor = BlueTarget;
                            break;
                        case 'Y':
                            requestedColor = GreenTarget;
                            break;
                        default:
                            wheelSpeed = setWheelSpeed;
                            break;
                    }
                    // If we have reached the detected color, stop
                    if (colorMatcher.matchClosestColor(detectedColor).color.equals(requestedColor)) {
                        if (passes == 6) {
                            wheelSpeed = 0;
                            System.out.println("Color done!");
                            executing = false;
                        } else if (!colorMatcher.matchClosestColor(lastColor).color.equals(requestedColor)) {
                            passes++;
                            System.out.println(passes);
                        }
                    }
                    lastColor = detectedColor;
                }
            }
        } else {
            wheelSpeed = 0;
        }

        output();
    };

    @Override
    public void runAuto(RobotState state) {
    };

    @Override
    protected void output() {
        // leftTurner.set(setWheelPositionLeftUp);
        // rightTurner.set(setWheelPositionRightUp);
        // wheel.set(setWheelSpeed);
        leftTurner.set(wheelPositionLeft);
        rightTurner.set(wheelPositionRight);
        wheel.set(wheelSpeed);
    };

    @Override
    protected void updateFeedback() {
        feedback.getWheelOfFortuneFeedback().setDetectedColor(detectedColor);
        // Color closet = colorMatcher.matchClosestColor(detectedColor).color;
        // System.out.printf("(%f, %f, %f)%n", closet.red, closet.green, closet.blue);
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