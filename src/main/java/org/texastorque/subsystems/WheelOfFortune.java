package org.texastorque.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import org.texastorque.constants.Ports;
import org.texastorque.inputs.State.RobotState;
import org.texastorque.torquelib.component.TorqueSparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

public class WheelOfFortune extends Subsystem {
    private static volatile WheelOfFortune instance;

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
        // lift attatchment
        // spin wheel slowly
        // if (color is right color){ stop wheel }
    };

    @Override
    public void runAuto(RobotState state) {
    };

    @Override
    protected void output() {
    };

    @Override
    protected void updateFeedback() {
    };

    @Override
    public void disable() {
    };

    @Override
    public void smartDashboard() {

    }

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