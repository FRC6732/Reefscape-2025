package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // The deadzone for the joysticks on the controllers
    public static final double kStickDeadband = 0.15;

    // X, Y, Theta slew rate limiters. Makes joystick inputs more gentle; 1/3 sec from 0 to 1.
    public static final SlewRateLimiter kXSlewRateLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter kYSlewRateLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter kTSlewRateLimiter = new SlewRateLimiter(3);
  }

  public static final class DriveConstants {
    public static final double kMaxSpeed = 4; // m / s
    public static final double kMaxAngularSpeed = Math.PI; // radians / s

    // Distance between left and right wheels
    public static final double kTrackWidth = Units.inchesToMeters(25);
    // Distance between front and back wheels
    public static final double kTrackHeight = Units.inchesToMeters(24);

    // Swerve kinematics (Polar coords) (FL, FR, BL, BR)
    public static final SwerveDriveKinematics kDriveKinematics = 
      new SwerveDriveKinematics(
        new Translation2d(kTrackWidth / 2, kTrackHeight / 2),
        new Translation2d(kTrackWidth / 2, -kTrackHeight / 2),
        new Translation2d(-kTrackWidth / 2, kTrackHeight / 2),
        new Translation2d(-kTrackWidth / 2, -kTrackHeight / 2)
      );

    // Spark CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kBackLeftDrivingCanId = 5;
    public static final int kBackRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kBackLeftTurningCanId = 6;
    public static final int kBackRightTurningCanId = 8;
  }

  public static final class ModuleConstants {
    // Some motor and gearbox constants
    public static final double kMotorFreeSpeedRpm = 5676;
    public static final double kDrivingMotorReduction = (50 * 17 * 45) / (14 * 27 * 15);
    
    // Some physical dimensions
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    // Scaling factors for each of the modules
    // Meters/rotation : wheel circumference divided by the gear reduction ratio
    public static final double kDrivingFactor = kWheelCircumference / kDrivingMotorReduction;
    // Radians/rotation : using a absolte encoder, a full circle in radians
    public static final double kTurningFactor = Math.PI * 2;
    // FF : 1 divided by freespeed from converted to rps
    public static final double kDrivingFF = 60 / kMotorFreeSpeedRpm;
  }
}
