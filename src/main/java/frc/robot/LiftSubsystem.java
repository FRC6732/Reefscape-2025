package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
    private final SparkMax m_leftCarriage;
    private final SparkMax m_rightCarriage;

    public LiftSubsystem () {
        m_leftCarriage = new SparkMax(LiftConstants.kLeftCanId, MotorType.kBrushed);
        m_rightCarriage = new SparkMax(LiftConstants.kRightCanId, MotorType.kBrushed);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig(); 

        leftConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake);

        rightConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .follow(LiftConstants.kLeftCanId, true);

        m_leftCarriage.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightCarriage.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void move(double direction) {
        double directionDelivered = direction * LiftConstants.kLiftSpeed + LiftConstants.kLiftConstant;

        m_leftCarriage.set(directionDelivered);
        m_rightCarriage.set(directionDelivered);
    }
}
