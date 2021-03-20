package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Autonomous 
{
    private static int autonomousCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;

    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    private static final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
    private static final ArrayList<Trajectory> trajectory = trajectoryLoader.getTrajectory(TrajectoryLoader.PathOption.Bounce);

    // The Ramsete Controller to follow the trajectory.
    private static final RamseteController ramseteController = new RamseteController(2.0, 0.7);

    // The timer to use during the autonomous period.
    private static final Timer timer = new Timer();

    private static int currentPath = 0;
    private static boolean newPathStarted = false;
    private static boolean finished = false;
    private static double currentPathTotalTime = 0.0;


    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }


    public Autonomous()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(autonomousCounter >= Constants.AUTONOMOUS_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            autonomousCounter++;

            // System.out.println("Autonomous: " + trajectory.size() + " paths");
        }
        
        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    public void init()
    {
        currentPath = 0;
        newPathStarted = true;
        finished = false;
        currentPathTotalTime = 0.0;

        // Initialize the timer.
        timer.reset();
        timer.start();

        // Reset the drivetrain's odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.get(0).getInitialPose());
    }

    public void periodic()
    {
        if(!finished)
        {
            if(newPathStarted)
            {
                newPathStarted = false;
                currentPath++;
                currentPathTotalTime = 0.0;
                if(currentPath <= trajectory.size())
                {
                    currentPathTotalTime = trajectory.get(currentPath-1).getTotalTimeSeconds();
                }
                else
                {
                    finished = true;
                }

                // System.out.println(currentPath);
                timer.reset();
            }

            // Update odometry.
            drivetrain.updateOdometry();

            if (currentPath <= trajectory.size() && timer.get() < currentPathTotalTime) 
            {
                // Get the desired pose from the trajectory.
                // State desiredPose = trajectory[m_currentPath-1].sample(m_timer.get());
                State desiredPose = trajectory.get(currentPath-1).sample(timer.get());

                // Get the reference chassis speeds from the Ramsete controller.
                ChassisSpeeds refChassisSpeeds = ramseteController.calculate(drivetrain.getPose(), desiredPose);
                // System.out.println("Cur Pose = " + drivetrain.getPose() + "  Desired Pose = " + desiredPose);
                System.out.print("Time = " + timer.get());
                // Set the linear and angular speeds.
                drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
            } 
            else 
            {
                drivetrain.stopMotor();  // Redundant intentionally to stop the robot
                newPathStarted = true;
            }
        }
        drivetrain.feed();
    }

    public void end()
    {
        
    }
}
