// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Robot extends TimedRobot 
{
    private static int robotCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // PUBLIC ROBOT COMPONENTS -- These must be final (constant)
    public static final DrivetrainFalconFX DRIVETRAIN = new DrivetrainFalconFX();
    // public static final DrivetrainSparkMax DRIVETRAIN = new DrivetrainSparkMax();
    public static final XboxController DRIVER_CONTROLLER = new XboxController(0);


    private static final Autonomous autonomous = new Autonomous();
    private static final Teleop teleop = new Teleop();
    private static final Test test = new Test();
    private static final Disabled disabled = new Disabled();


    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }

    public enum RobotState
    {
        kNone,
        kStartup,
        kDisabledBeforeGame,
        kAutonomous,
        kDisabledBetweenAutonomousAndTeleop,
        kTeleop,
        kDisabledAfterGame,
        kTest;
    }

    private static RobotState robotState = RobotState.kNone;

    public Robot()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(robotCounter >= Constants.ROBOT_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
            robotState = RobotState.kStartup;
        }
        else
        {
            robotCounter++;
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    @Override
    public void robotInit() 
    {
        System.out.println("  -> Starting Robot Init");
        
        System.out.println("  -> Finishing Robot Init");
    }

    @Override
    public void robotPeriodic() 
    {

    }

    @Override
    public void autonomousInit() 
    {
        robotState = RobotState.kAutonomous;
        autonomous.init();
    }

    @Override
    public void autonomousPeriodic() 
    {
        autonomous.periodic();
    }

    @Override
    public void teleopInit()
    {
        robotState = RobotState.kTeleop;
        teleop.init();
    }

    @Override
    public void teleopPeriodic() 
    {
        teleop.periodic();
    }

    @Override
    public void testInit()
    {
        robotState = RobotState.kTest;
        test.init();
    }

    @Override
    public void testPeriodic()
    {
        test.periodic();
    }

    @Override
    public void disabledInit()
    {
        if (robotState == RobotState.kStartup)
        {
            robotState = RobotState.kDisabledBeforeGame;
        }
        else if (robotState == RobotState.kAutonomous)
        {
            autonomous.end();
            robotState = RobotState.kDisabledBetweenAutonomousAndTeleop;
        }
        else if (robotState == RobotState.kTeleop)
        {
            teleop.end();
            robotState = RobotState.kDisabledAfterGame;
        }
        else if (robotState == RobotState.kTest)
        {
            test.end();
            robotState = RobotState.kDisabledBeforeGame;
        }
        disabled.init();
    }

    @Override
    public void disabledPeriodic()
    {
        disabled.periodic();
    }

    /**
     * This method returns the current state of the robot
     * @return the robot state
     * @see RobotState
     */
    public static RobotState getRobotState()
    {
        return robotState;
    }
}
