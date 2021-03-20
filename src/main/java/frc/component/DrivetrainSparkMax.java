// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.component;

import java.lang.invoke.MethodHandles;

import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.SerialPort;
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
public class DrivetrainSparkMax extends DifferentialDrive
{
    private static int drivetrainCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final NavX navX = new NavX(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 100); 

    private static final CANSparkMax leftLeader = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax rightLeader = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final CANSparkMax leftFollower = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final CANSparkMax rightFollower = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // private static final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    // private static final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    private static final CANEncoder leftEncoder  = leftLeader.getEncoder();
    private static final CANEncoder rightEncoder = rightLeader.getEncoder();
    private static final PIDController leftPIDController = new PIDController(1, 0, 0);
    private static final PIDController rightPIDController = new PIDController(1, 0, 0);

    // private static DifferentialDrive drivetrain = null;
    private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);
    private static final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(navX.getRotation2d());

    // Gains are for example purposes only - must be determined for your own robot!
    // private static final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.22, 1.98, 0.2);    


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
    public DrivetrainSparkMax() 
    {
        super(leftLeader, rightLeader);  // MUST BE FIRST STATEMENT IN CONSTRUCTOR
        // super(leftGroup, rightGroup);  // MUST BE FIRST STATEMENT IN CONSTRUCTOR

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

            initializeMotor(leftLeader);
            initializeMotor(rightLeader);
            rightLeader.setInverted(true); // right motor mounted mirror image - inverted - from the left
            super.setRightSideInverted(false);

            // Set the distance per pulse for the drive encoders. We can simply use the
            // distance traveled for one rotation of the wheel divided by the encoder
            leftEncoder.setVelocityConversionFactor(Constants.METERS_PER_MOTOR_ROTATION / 60.0);
            rightEncoder.setVelocityConversionFactor(Constants.METERS_PER_MOTOR_ROTATION / 60.0);

            leftEncoder.setPositionConversionFactor(Constants.METERS_PER_MOTOR_ROTATION);
            rightEncoder.setPositionConversionFactor(Constants.METERS_PER_MOTOR_ROTATION);

            resetEncoders();
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    private void initializeMotor(CANSparkMax motor)
    {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.NEO_SMART_CURRENT_LIMIT);
        motor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
        motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds)
    {
        final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = leftPIDController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput = rightPIDController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

        // leftGroup.setVoltage(leftOutput + leftFeedforward);
        // rightGroup.setVoltage(rightOutput + rightFeedforward);
        leftLeader.setVoltage(leftOutput + leftFeedforward);
        rightLeader.setVoltage(rightOutput + rightFeedforward);
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
        super.feed();
    }

    public void stopMotor()
    {
        super.stopMotor();
    }

    /** Updates the field-relative position. */
    public void updateOdometry()
    {
        odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
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
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
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

    public SimpleMotorFeedforward getMotorFeedforward()
    {
        return feedforward;
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
            leftEncoder.getPosition(), rightEncoder.getPosition(),
            navX.getAngle(), getPose());


        return str;
    }
}
