package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Teleop 
{
    private static int teleopCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;
    // private static final DrivetrainSparkMax drivetrain = Robot.DRIVETRAIN;
    private static final XboxController driverController = Robot.DRIVER_CONTROLLER;
    private static final PowerDistributionPanel pdp = new PowerDistributionPanel();
    private static double[] previousVel = {0,0};
    
    
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
        double[] vel = drivetrain.getVelocity();
        double speed = -driverController.getY(GenericHID.Hand.kLeft);
        if(Math.abs(speed) < 0.15)
            speed = 0.0;        
        
        System.out.printf("Volts=%5.2f", speed * pdp.getVoltage());
        System.out.printf("  Angle=%5.2f", drivetrain.getAngle());
        System.out.printf("  Lvel=%5.2f", vel[0]);
        System.out.printf("  Rvel=%5.2f", vel[1]);
        System.out.printf("  Lacc=%5.2f", (vel[0]-previousVel[0]) * 50.0);
        System.out.printf("  Racc=%5.2f", (vel[1]-previousVel[1]) * 50.0);
        System.out.printf("  Racc=%5.2f\n", (vel[1]-previousVel[1]) * 50.0);

        previousVel[0] = vel[0];
        previousVel[1] = vel[1];





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
