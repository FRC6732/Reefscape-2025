package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  private final SparkMax m_leftCarriage;
  private final SparkMax m_rightCarriage;

  private final SparkMax m_block;

  public CoralSubsystem() {
    m_leftCarriage = new SparkMax(CoralConstants.kLeftLiftCanId, MotorType.kBrushed);
    m_rightCarriage = new SparkMax(CoralConstants.kRightLiftCanId, MotorType.kBrushed);

    m_block = new SparkMax(CoralConstants.kBlockCanId, MotorType.kBrushed);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    SparkMaxConfig blockConfig = new SparkMaxConfig();

    leftConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    rightConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake)
        .follow(CoralConstants.kLeftLiftCanId, true);

    blockConfig
        // .inverted(true)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    m_leftCarriage.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightCarriage.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_block.configure(blockConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void move(double direction) {
    double directionDelivered = direction * CoralConstants.kLiftSpeed + CoralConstants.kLiftConstant;

    m_leftCarriage.set(directionDelivered);
    m_rightCarriage.set(directionDelivered);
  }

  public void release() {
    m_block.set(-1);
  }

  @Override
  public void periodic() {
    super.periodic();

    m_block.set(CoralConstants.kBlockConstant);
  }
}
