package frc.robot;

import java.lang.invoke.MethodHandles;

public final class Constants
{
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    public static final double kTrackWidth = 0.7126; // meters outside wheel to outside wheel
    // private static final double kWheelRadius = 0.0508; // meters
    // private static final int kEncoderResolution = 4096;

    public static final int FRONT_LEFT_MOTOR_PORT = 4;
    public static final int FRONT_RIGHT_MOTOR_PORT = 1;
    public static final int BACK_RIGHT_MOTOR_PORT = 2;
    public static final int BACK_LEFT_MOTOR_PORT = 3;

    public static final int TALONFX_FRONT_LEFT_MOTOR_PORT = 1;
    public static final int TALONFX_FRONT_RIGHT_MOTOR_PORT = 4;
    public static final int TALONFX_BACK_RIGHT_MOTOR_PORT = 3;
    public static final int TALONFX_BACK_LEFT_MOTOR_PORT = 2;

    // public static final int PRIMARY_MOTOR_CURRENT_LIMIT = 35;
    // public static final int SECONDARY_MOTOR_CURRENT_LIMIT = 45;

    public static final double DRIVE_RAMP_TIME = 0.10;

    // public static final double MOTOR_DEADBAND = 0.01;

    // public static final double STARTING_SPEED = 0.3;
    // public static final double STOPPING_SPEED = 0.175;
    // public static final int ROTATE_THRESHOLD = 10;

    // public static final int LEFT_ENCODER_CHANNEL_A = 18;
    // public static final int LEFT_ENCODER_CHANNEL_B = 16;
    // public static final int RIGHT_ENCODER_CHANNEL_A = 14;
    // public static final int RIGHT_ENCODER_CHANNEL_B = 15;

    // 4096 ticks per motor revolution native NEO brushless
    //public static final double ENCODER_TICKS_PER_INCH = (360.0 * 4.0) / (3.25 * Math.PI);
    public static final double ENCODER_METER_PER_REV = 1.0 / 19.05; // approximately
    public static final double WHEEL_CIRCUMFERENCE_IN_METERS = 4.0 * Math.PI * 2.54 / 100.0;
    public static final double GEAR_RATIO_IN_MOTOR_ROT_PER_WHEEL_ROT = (60.0 / 12.0) * (28.0 / 28.0) * (36.0 / 30.0);
    public static final double METERS_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE_IN_METERS / GEAR_RATIO_IN_MOTOR_ROT_PER_WHEEL_ROT;
    // public static final double ENCODER_METER_PER_TICK = ENCODER_METER_PER_REV / 4096.0;

    public static final double COMP_BOT_WHEEL_CIRCUMFERENCE_IN_METERS = 6.20 * Math.PI * 2.54 / 100.0; // original wheel diameter was 6.25
    public static final double COMP_BOT_MAIN_GEAR_RATIO = (60.0 / 12.0) * (36.0 / 20.0);
    public static final double COMP_BOT_HIGH_GEAR_RATIO = COMP_BOT_MAIN_GEAR_RATIO * (34.0 / 32.0);
    public static final double COMP_BOT_LOW_GEAR_RATIO = COMP_BOT_MAIN_GEAR_RATIO * (44.0 / 22.0);
    public static final double COMP_BOT_HIGH_GEAR_METERS_PER_MOTOR_ROTATION = COMP_BOT_WHEEL_CIRCUMFERENCE_IN_METERS / COMP_BOT_HIGH_GEAR_RATIO;
    public static final double COMP_BOT_LOW_GEAR_METERS_PER_MOTOR_ROTATION = COMP_BOT_WHEEL_CIRCUMFERENCE_IN_METERS / COMP_BOT_LOW_GEAR_RATIO;

    public static final double COMP_BOT_kP = 4.0;  //6.0
    public static final double COMP_BOT_kI = 0.0;
    public static final double COMP_BOT_kD = 0.0;
    public static final double COMP_BOT_FF_kS = 2.0;
    public static final double COMP_BOT_FF_kV = 5.0 * 0.85;
    public static final double COMP_BOT_FF_kA = 5.0;
    public static final double COMP_BOT_kTrackWidth = 0.7126;

    public static final int NEO_MOTOR_STALL_CURRENT = 150;
    public static final double NEO_MOTOR_FREE_CURRENT = 1.8;
    public static final int NEO_SMART_CURRENT_LIMIT = (int) (0.5 * (NEO_MOTOR_STALL_CURRENT - NEO_MOTOR_FREE_CURRENT) + NEO_MOTOR_FREE_CURRENT);

    public static final int DRIVETRAIN_LIMIT = 1;
    public static final int ROBOT_LIMIT = 1;
    public static final int TELEOP_LIMIT = 1;
    public static final int AUTONOMOUS_LIMIT = 1;
    public static final int TEST_LIMIT = 1;
    public static final int DISABLED_LIMIT = 1;
    public static final int NAVX_LIMIT = 1;

    public static final String CLASS_LOADING         = "++++ Class Loading         : ";
    public static final String CONSTRUCTOR_STARTING  = "  >> Starting Constructor  : ";
    public static final String CONSTRUCTOR_FINISHING = "  << Finishing Constructor : ";

    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + MethodHandles.lookup().lookupClass().getCanonicalName());
    }
}

