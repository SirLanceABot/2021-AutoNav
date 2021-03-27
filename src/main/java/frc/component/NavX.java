//////////// NAVX GYRO SUBSYSTEM ////////////

// the NavX software (class AHRS) provides a complete source of information
// that no new methods are here except a couple of display values methods
// handy for debugging

// checking for AHRS instantiation and calibration down time used in past years
// could be added here

package frc.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import java.lang.invoke.MethodHandles;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

public class NavX /*extends AHRS*/
{
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static AHRS ahrs;

    // private static final AHRS navX;

    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }
    
    public NavX()
    {
        //super(serial_port_id);


        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here
        try{
            ahrs = new AHRS(I2C.Port.kOnboard, (byte) 100);
        } catch (RuntimeException ex )
        {
           DriverStation.reportError("Error instantiating the Navx: " + ex.getMessage(), true);
        }
        // init();
                
        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    public NavX(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) 
    {
        // super(serial_port_id, data_type, update_rate_hz);

        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here
        try{
            ahrs = new AHRS(serial_port_id, data_type, update_rate_hz);
        } catch (RuntimeException ex )
        {
           DriverStation.reportError("Error instantiating the Navx: " + ex.getMessage(), true);
        }
        init();
        
        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    private void init() 
    {
        try 
        {
            Timer.delay(2.0); // make sure AHRS USB communication is done before doing
            // other USB. Also give it time to settle in; a few more seconds never hurt
            ahrs.enableLogging(true);
            System.out.println(ahrs.getActualUpdateRate() + " " + ahrs.getUpdateCount() + " " + ahrs.getRequestedUpdateRate());
        }
        catch (final RuntimeException ex)
        {
            DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
        }

        reset();
    }
    
    public Rotation2d getRotation2d()
    {
        return ahrs.getRotation2d();
    }

    public void reset()
    {
        ahrs.reset();
    }

    public double getAngle()
    {
        return ahrs.getAngle();
    }

    public void displayOnSmartDashboard()
    {
        // fused heading 0 to 360
        // IMU total yaw -inf to +inf
        // IMU yaw -180 to +180
        // IMU yaw rate dps -big to +big
        SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
    
        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */
        SmartDashboard.putBoolean("Magnetic Disturbance", ahrs.isMagneticDisturbance());
        SmartDashboard.putBoolean("Magnetometer Calibrated", ahrs.isMagnetometerCalibrated());
    
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
    
        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());
    
        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */
        SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());
    
        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putNumber("IMU_Accel_Z", ahrs.getWorldLinearAccelZ());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());
    
        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */
        SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
        SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
        SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Displacement_Z", ahrs.getDisplacementZ());
    
        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */
        SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
        SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());
    
        SmartDashboard.putNumber("Fused Heading", ahrs.getFusedHeading());
    
        /* Omnimount Yaw Axis Information */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        final AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
        // System.out.println(AHRS.BoardAxis.kBoardAxisX + " " +
        // AHRS.BoardAxis.kBoardAxisY + " " + AHRS.BoardAxis.kBoardAxisZ +
        // yaw_axis.board_axis.getValue());
        final String kBoardAxisAlpha[] = { "BoardAxisX", "BoardAxisY", "BoardAxisZ" };
        SmartDashboard.putString("YawAxisAlpha", kBoardAxisAlpha[yaw_axis.board_axis.getValue()]);
    
        /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());
    
        /* Quaternion Data */
        /* Quaternions are fascinating, and are the most compact representation of */
        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        /* from the Quaternions. If interested in motion processing, knowledge of */
        /* Quaternions is highly recommended. */
        SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
        SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
        SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
        SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());
    
        /* Connectivity Debugging Support */
        SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
        SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
        // if (ahrs->IsAltitudeValid()) // Aero only
        // {
        // SmartDashboard::PutNumber( "Barometric Pressure",
        // ahrs->GetBarometricPressure() );
        // SmartDashboard::PutNumber( "Altitude", ahrs->GetAltitude() );
        // SmartDashboard::PutNumber( "Pressure", ahrs->GetPressure() );
        // }
        // else
        // {
        // SmartDashboard::PutString( "Barometric Pressure", (llvm::StringRef)"Not
        // Available" );
        // SmartDashboard::PutString( "Altitude", (llvm::StringRef)"Not Available" );
        // SmartDashboard::PutString( "Pressure", (llvm::StringRef)"Not Available" );
        // }
    
        /* Display 6-axis Processed Angle Data */
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    }

    public String toString() 
    {
        return "angle:" + ahrs.getAngle() + ", rate:" + ahrs.getRate() + ", " + ahrs.getRotation2d();
    }

}

/***********************************************************************
 * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
 * 
 * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
 * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
 * 
 * Multiple navX-model devices on a single robot are supported.
 ************************************************************************/
// ahrs = new AHRS(SerialPort.Port.kUSB1);
// ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 200);
// ahrs = new AHRS(SPI.Port.kMXP);
// ahrs = new AHRS(I2C.Port.kMXP);
// ahrs = new AHRS(I2C.Port.kOnboard, (byte) 60);


// ** Interface for yaw rate gyros. */
// public interface Gyro extends AutoCloseable {
//     /**
//      * Calibrate the gyro. It's important to make sure that the robot is not moving while the
//      * calibration is in progress, this is typically done when the robot is first turned on while it's
//      * sitting at rest before the match starts.
//      */
//     void calibrate();
  
//     /**
//      * Reset the gyro. Resets the gyro to a heading of zero. This can be used if there is significant
//      * drift in the gyro and it needs to be recalibrated after it has been running.
//      */
//     void reset();
  
//     /**
//      * Return the heading of the robot in degrees.
//      *
//      * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
//      * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
//      * 360 to 0 on the second time around.
//      *
//      * <p>The angle is expected to increase as the gyro turns clockwise when looked at from the top.
//      * It needs to follow the NED axis convention.
//      *
//      * <p>This heading is based on integration of the returned rate from the gyro.
//      *
//      * @return the current heading of the robot in degrees.
//      */
//     double getAngle();
  
//     /**
//      * Return the rate of rotation of the gyro.
//      *
//      * <p>The rate is based on the most recent reading of the gyro analog value
//      *
//      * <p>The rate is expected to be positive as the gyro turns clockwise when looked at from the top.
//      * It needs to follow the NED axis convention.
//      *
//      * @return the current rate in degrees per second
//      */
//     double getRate();
  
//     /**
//      * Return the heading of the robot as a {@link edu.wpi.first.wpilibj.geometry.Rotation2d}.
//      *
//      * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
//      * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
//      * 360 to 0 on the second time around.
//      *
//      * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
//      * top. It needs to follow the NWU axis convention.
//      *
//      * <p>This heading is based on integration of the returned rate from the gyro.
//      *
//      * @return the current heading of the robot as a {@link
//      *     edu.wpi.first.wpilibj.geometry.Rotation2d}.
//      */
//     default Rotation2d getRotation2d() {
//       return Rotation2d.fromDegrees(-getAngle());
//     }
//   }
