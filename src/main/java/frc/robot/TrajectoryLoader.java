package frc.robot;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class TrajectoryLoader
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    
    public enum PathOption {BarrelRacing, Bounce, Slalom, Manual};
    
    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;;
    private static final ArrayList<Trajectory> trajectory = new ArrayList<Trajectory>();
    private static final DifferentialDriveVoltageConstraint voltageConstraint = 
            new DifferentialDriveVoltageConstraint(drivetrain.getMotorFeedforward(), drivetrain.getKinematics(), 10.0);
    private static final TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(2.0, 1.0).setKinematics(drivetrain.getKinematics()).addConstraint(voltageConstraint);


    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }


    public TrajectoryLoader()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    public ArrayList<Trajectory> getTrajectory(PathOption pathOption)
    {
        switch(pathOption)
        {
        case BarrelRacing:
        case Bounce:
        case Slalom:
            createTrajectoryFromFile(pathOption);
            break;
        case Manual:
            createTrajectoryFromPoints();
            break;
        }

        return trajectory;
    }

    private void createTrajectoryFromPoints()
    {
        trajectory.clear();
        trajectory.add(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig
            )
        );

        System.out.println(trajectory.get(0).getStates());
    }

    private void createTrajectoryFromFile(PathOption pathOption)
    {
        int pathNumber = 1;
        boolean done = false;
        String trajectoryJSON;
        Path trajectoryPath;

        trajectory.clear();

        while(!done)
        {
            trajectoryJSON = "output/" + pathOption.name() + pathNumber + ".wpilib.json";
            try
            {
                trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                // trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                if(trajectoryPath.toFile().exists())
                {
                    trajectory.add(TrajectoryUtil.fromPathweaverJson(trajectoryPath));
                    pathNumber++; 
                }
                else
                {
                    done = true;
                }
            }
            catch (IOException ex)
            {
                DriverStation.reportError("Unable to open trajectory : " + trajectoryJSON, ex.getStackTrace());
            }
        }
    }
}
