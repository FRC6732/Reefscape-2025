package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final SparkMaxConfig kDrivingConfig = new SparkMaxConfig();
  public static final SparkMaxConfig kTurningConfig = new SparkMaxConfig();

  static {
    // Drive motor configs
    kDrivingConfig
        .idleMode(IdleMode.kBrake) // Stop during no input
        .smartCurrentLimit(40); // Max amps before stall

    kDrivingConfig.encoder
        .positionConversionFactor(ModuleConstants.kDrivingFactor) // meters
        .velocityConversionFactor(ModuleConstants.kDrivingFactor / 60); // meters / second

    kDrivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.06, 0, 0) // gains
        .velocityFF(ModuleConstants.kDrivingFF)
        .outputRange(-1, 1);

    // Turn motor configs
    kTurningConfig
        .idleMode(IdleMode.kBrake) // Stop during no input
        .smartCurrentLimit(40); // Max amps before stall

    kTurningConfig.absoluteEncoder
        .inverted(true) // Due to gearing, motor rotates inversely to encoder output
        .positionConversionFactor(ModuleConstants.kTurningFactor) // radians
        .velocityConversionFactor(ModuleConstants.kTurningFactor / 60); // radians/second

    kTurningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(.8, 0, 0) // gains
        .outputRange(-1, 1)
        // Allow the controller to go through 0 when mapping (340 -> 15 adjusts 35
        // degrees rather than 325)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, ModuleConstants.kTurningFactor);
  }
}
