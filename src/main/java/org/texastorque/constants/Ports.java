package org.texastorque.constants;

public class Ports {
    // talon CAN id
    public static final int FLYWHEEL_LEAD = 1; // right
    public static final int FLYWHEEL_FOLLOW = 0; // left

    // neo CAN id - DO NOT HAVE ID 0 IT DOES NOT WORK
    public static final int DB_LEFT_1 = 1; // done
    public static final int DB_LEFT_2 = 2; // done
    public static final int DB_RIGHT_1 = 3; // done
    public static final int DB_RIGHT_2 = 4; // done

    public static final int CLIMBER_LEFT = 12;
    public static final int CLIMBER_RIGHT = 13;

    public static final int WHEEL_OF_FORTUNE = 14;

    // neo 550 CAN id - DO NOT HAVE ID 0 IT DOES NOT WORK
    public static final int INTAKE_ROTARY_LEFT = 5; // left side spark max
    public static final int INTAKE_ROTARY_RIGHT = 6; // right side spark max
    public static final int INTAKE_ROLLERS = 7; // rollers

    public static final int BELT_HIGH = 8;
    // public static final int BELT_GATE = 9;
    public static final int BELT_LOW = 11;

    public static final int SHOOTER_HOOD = 10;

    // testing individual motors
    public static final int TALON_PORT = 0;
    public static final int SPARKMAX_PORT_1 = 1;
    public static final int SPARKMAX_PORT_2 = 2;

    // PWM
    public static final int CLIMB_SERVO_LEFT = 0;
    public static final int CLIMB_SERVO_RIGHT = 1;
    public static final int WHEEL_OF_FORTUNE_LEFT = 2;
    public static final int WHEEL_OF_FORTUNE_RIGHT = 3;

    // Sensors
    public static final int MAG_SENSOR_HIGH = 6;
    public static final int MAG_SENSOR_MIDDLE = 5;
    public static final int MAG_SENSOR_LOW = 4;

    // Control Panel
    public static final int CP_MOTOR_PORT = 0;

    // Servos
    public static final int LEFT_RATCHET = 0;
    public static final int RIGHT_RATCHET = 1;
}
