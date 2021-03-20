package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Teleop 
{
    private static int teleopCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;
    // private static final DrivetrainSparkMax drivetrain = Robot.DRIVETRAIN;
    private static final XboxController driverController = Robot.DRIVER_CONTROLLER;
    
    
    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }


    public Teleop()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(teleopCounter >= Constants.TELEOP_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            teleopCounter++;
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }
    
    public void init()
    {

    }   
    
    public void periodic()
    {
        System.out.println("Angle = " + drivetrain.getAngle());
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double speed = driverController.getY(GenericHID.Hand.kLeft);
        if(Math.abs(speed) < 0.15)
            speed = 0.0;
        // final double xSpeed = -m_speedLimiter.calculate(y) * Drivetrain.kMaxSpeed;
        // final double speed = -leftY * Constants.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        // double rightX = driverController.getX(GenericHID.Hand.kRight);
        // double rightY = driverController.getY(GenericHID.Hand.kRight);
        // double joystickDirection = Math.atan2(rightY, rightX) * 180.0 / Math.PI;
        // double robotDirection = drivetrain.getDirection();
            
        // final double rot = -m_rotLimiter.calculate(x) * Drivetrain.kMaxAngularSpeed;
        double rot = driverController.getX(GenericHID.Hand.kRight);
        if(Math.abs(rot) < 0.15)
            rot = 0.0;

        drivetrain.arcadeDrive(speed, rot);

        // System.out.println(drivetrain);
    }
    
    public void end()
    {
        
    }
}
