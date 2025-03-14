package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;

public class Robot extends TimedRobot {
  private final Drivetrain m_drive = new Drivetrain();
  private final XboxController m_controller = new XboxController(OperatorConstants.kDriverControllerPort);

  private void configureBindings() {
    new JoystickButton(m_controller, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_drive.setStationary(),
            m_drive));
  }

  @Override
  public void robotInit() {
    configureBindings();

    // If the doesn't work, comment it and uncomment the below drive function
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(
                OperatorConstants.kXSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(m_controller.getLeftY(), OperatorConstants.kStickDeadband)),
                -OperatorConstants.kYSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(m_controller.getLeftX(), OperatorConstants.kStickDeadband)),
                -OperatorConstants.kTSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(m_controller.getRightX(), OperatorConstants.kStickDeadband)),
                false),
            m_drive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
