package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  // Drive modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      0);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      0);

  private final SwerveModule m_backLeft = new SwerveModule(
      DriveConstants.kBackLeftDrivingCanId,
      DriveConstants.kBackLeftTurningCanId,
      0);

  private final SwerveModule m_backRight = new SwerveModule(
      DriveConstants.kBackRightDrivingCanId,
      DriveConstants.kBackRightTurningCanId,
      0);

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(360 - m_gyro.getAngle()); // invert the gyro
  }

  @Override
  public void periodic() {
    m_odometry.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Factor in our speed scales
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeed;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    setModuleStates(swerveModuleStates);
  }

  // Really bad hardcode, but this is the fallback
  public void driveHardcode(double xSpeed, double ySpeed, double rot) {
    if (rot != 0) {
      double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
      setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(rotDelivered, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(rotDelivered, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(rotDelivered, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(rotDelivered, Rotation2d.fromDegrees(-45))
      });

      return;
    }

    // Vector length and direction
    double speedDelivered = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)) * DriveConstants.kMaxSpeed;
    double dirDelivered = Math.atan(ySpeed / xSpeed) + Math.PI / 2;

    setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(speedDelivered, Rotation2d.fromRadians(dirDelivered)),
      new SwerveModuleState(speedDelivered, Rotation2d.fromRadians(dirDelivered)),
      new SwerveModuleState(speedDelivered, Rotation2d.fromRadians(dirDelivered)),
      new SwerveModuleState(speedDelivered, Rotation2d.fromRadians(dirDelivered))
    });
  }

  public void setStationary() {
    setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    });
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeed);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }
}
