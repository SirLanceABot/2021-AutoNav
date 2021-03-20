package frc.robot;

import java.lang.invoke.MethodHandles;

import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Disabled 
{
    private static int disabledCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;
    
    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }

    public Disabled()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(disabledCounter >= Constants.DISABLED_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            disabledCounter++;
        }
        
        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }
    public void init()
    {
        drivetrain.resetEncoders();
    }

    public void periodic()
    {

    }

    public void end()
    {
        
    }
}
