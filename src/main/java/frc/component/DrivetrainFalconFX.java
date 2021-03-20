// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.component;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import frc.robot.Constants;

/** Represents a differential drive style drivetrain. */
public class DrivetrainFalconFX extends DifferentialDrive
{
    private static int drivetrainCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final NavX navX = new NavX();
    // private static final NavX navX = new NavX(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 100); 

    // private static final CANSparkMax leftLeader = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final CANSparkMax rightLeader = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final CANSparkMax leftFollower = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final CANSparkMax rightFollower = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.TALONFX_BACK_LEFT_MOTOR_PORT);
    private static final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.TALONFX_FRONT_LEFT_MOTOR_PORT);
    private static final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.TALONFX_BACK_RIGHT_MOTOR_PORT);
    private static final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.TALONFX_FRONT_RIGHT_MOTOR_PORT);

    private static final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    private static final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    // private static final CANEncoder leftEncoder  = leftLeader.getEncoder();
    // private static final CANEncoder rightEncoder = rightLeader.getEncoder();
    private static final PIDController leftPIDController = new PIDController(4, 0, 0);
    private static final PIDController rightPIDController = new PIDController(4, 0, 0);

    // private static final SupplyCurrentLimitConfiguration supplyCurrentLimitConfig = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5);
    // private static final StatorCurrentLimitConfiguration statorCurrentLimitConfig = new StatorCurrentLimitConfiguration(true, 40, 60, 0.5);

    // private static DifferentialDrive drivetrain = null;
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);
    private static final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navX.getRotation2d());

    // Gains are for example purposes only - must be determined for your own robot!
    // private static final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.22, 4, 0.2);    


    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }


    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
     * gyro.
     */
    public DrivetrainFalconFX() 
    {
        // super(leftLeader, rightLeader);  // MUST BE FIRST STATEMENT IN CONSTRUCTOR
        super(leftGroup, rightGroup);  // MUST BE FIRST STATEMENT IN CONSTRUCTOR

        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(drivetrainCounter >= Constants.DRIVETRAIN_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            drivetrainCounter++;

            navX.reset();


            // leftFollower.follow(leftLeader);
            // rightFollower.follow(rightLeader); 

            initializeMotor(leftLeader, false);
            initializeMotor(leftFollower, false);
            initializeMotor(rightLeader, true);
            initializeMotor(rightFollower, true);

            // rightLeader.setInverted(true); // right motor mounted mirror image - inverted - from the left
            // rightFollower.setInverted(true);
            // leftLeader.setInverted(false);
            // leftFollower.setInverted(false);
            // Set the distance per pulse for the drive encoders. We can simply use the
            // distance traveled for one rotation of the wheel divided by the encoder
            // leftEncoder.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV / 60.0);
            // rightEncoder.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV / 60.0);

            // leftEncoder.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV);
            // rightEncoder.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV);

            resetEncoders();
            setRightSideInverted(false);
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    private void initializeMotor(WPI_TalonFX motor, boolean isInverted)
    {
        // motor.restoreFactoryDefaults();
        // motor.setSmartCurrentLimit(Constants.NEO_SMART_CURRENT_LIMIT);
        // motor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
        // motor.setIdleMode(IdleMode.kBrake);
        // motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        // motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        // motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
        // motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);

        motor.configFactoryDefault();
        motor.setInverted(isInverted);
        motor.setNeutralMode(NeutralMode.Brake);

        //feedback sensor
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        //soft limits
        motor.configReverseSoftLimitThreshold(0);
        motor.configReverseSoftLimitEnable(false);
        motor.configForwardSoftLimitThreshold(0);
        motor.configForwardSoftLimitEnable(false);

        motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        
        //current limits
        // motor.configOpenloopRamp(0.1);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5), 10);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 60, 0.5), 10);
    }

    public double getVelocity(WPI_TalonFX motor)
    {
        double vel = motor.getSelectedSensorVelocity() * 10.0 / 2048.0 * Constants.COMP_BOT_LOW_GEAR_METERS_PER_MOTOR_ROTATION;
        return vel;
    }

    public double getPosition(WPI_TalonFX motor)
    {
        double selectPos = motor.getSelectedSensorPosition();
        double trajPos = motor.getSensorCollection().getIntegratedSensorPosition();
        double pos = selectPos / 2048.0 * Constants.COMP_BOT_LOW_GEAR_METERS_PER_MOTOR_ROTATION;
        // System.out.println("Sel Pos = " + selectPos + "  Pos = " + pos + "  Traj Pos = " + trajPos);
        return pos;
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds)
    {
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond, 0.0);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond, 0.0);

        // final double leftOutput = leftPIDController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        // final double rightOutput = rightPIDController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
        final double leftOutput = leftPIDController.calculate(getVelocity(leftLeader), speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(getVelocity(rightLeader), speeds.rightMetersPerSecond);

        leftGroup.setVoltage(leftOutput + leftFeedforward);
        rightGroup.setVoltage(rightOutput + rightFeedforward);

        System.out.println("Left Voltage = " + (leftOutput + leftFeedforward));
        // leftLeader.setVoltage(leftOutput + leftFeedforward);
        // rightLeader.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot)
    {
        final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    public void stopMotor()
    {
        super.stopMotor();
    }

    /** Updates the field-relative position. */
    public void updateOdometry()
    {
        // odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        odometry.update(navX.getRotation2d(), getPosition(leftLeader), getPosition(rightLeader));
    }

    /**
     * Resets the field-relative position to a specific location.
     *
     * @param pose The position to reset to.
     */
    public void resetOdometry(Pose2d pose)
    {
        odometry.resetPosition(pose, navX.getRotation2d());
    }

    public void resetEncoders()
    {
        // leftEncoder.setPosition(0.0);
        // rightEncoder.setPosition(0.0);
        leftLeader.setSelectedSensorPosition(0.0);
        leftFollower.setSelectedSensorPosition(0.0);
        rightLeader.setSelectedSensorPosition(0.0);
        rightFollower.setSelectedSensorPosition(0.0);
    }

    /**
     * Returns the pose of the robot.
     *
     * @return The pose of the robot.
     */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveKinematics getKinematics()
    {
        return kinematics;
    }

    public double getAngle()
    {
        return navX.getAngle();
    }

    public SimpleMotorFeedforward getMotorFeedforward()
    {
        return feedforward;
    }

    public double getDirection()
    {
        return navX.getAngle();
    }
    
    @Override
    public String toString()
    {
        String str;
        // str = String.format("LV: %.2f, RV: %.2f LP: %.2f, RP %.2f Yaw: %.2f",
        //     leftEncoder.getVelocity(), rightEncoder.getVelocity(),
        //     leftEncoder.getPosition(), rightEncoder.getPosition(),
        //     navX.getAngle());

        str = String.format("LP: %.2f, RP %.2f Yaw: %.2f Pose: %s",
            leftLeader.getSelectedSensorPosition(), rightLeader.getSelectedSensorPosition(),
            navX.getAngle(), getPose());


        return str;
    }
}
