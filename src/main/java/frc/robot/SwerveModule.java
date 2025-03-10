package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private final SparkMax m_drivingMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingController;
  private final SparkClosedLoopController m_turningController;

  // The direction the encoder is facing in relation to the chassis direction
  private final double m_rotationOffset;

  // Other than a local pointer to the set desired state, the purpose of this variable makes no sense to me...
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs and configures an SDS MK4i Swerve Module (NEO motor and absolute
   * encoder configuration)
   *
   * @param drivingCAN     The CAN ID for the drive motor
   * @param turningCAN     The CAN ID for the turn motor
   * @param rotationOffset The offset of the absolute encoder rotation in radians
   */
  public SwerveModule(int drivingCAN, int turningCAN, double rotationOffset) {
    m_drivingMotor = new SparkMax(drivingCAN, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningCAN, MotorType.kBrushless);

    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_drivingController = m_drivingMotor.getClosedLoopController();
    m_turningController = m_turningMotor.getClosedLoopController();

    m_drivingMotor.configure(Configs.kDrivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(Configs.kTurningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_rotationOffset = rotationOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    resetEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        Rotation2d.fromRadians(m_turningEncoder.getPosition() - m_rotationOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        Rotation2d.fromRadians(m_turningEncoder.getPosition() - m_rotationOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedState = new SwerveModuleState();
    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_rotationOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees
    correctedState.optimize(Rotation2d.fromRadians(m_turningEncoder.getPosition()));
    // and scale speed by cosine of angle error for smoother driving
    // m_desiredState.cosineScale(Rotation2d.fromRadians(m_turningEncoder.getPosition()));

    // Apply the speed and rotation to the controllers
    m_drivingController.setReference(correctedState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningController.setReference(correctedState.angle.getRadians(), ControlType.kPosition);

    // Why?
    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
