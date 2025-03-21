package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

public class Robot extends TimedRobot {
  private final Drivetrain m_drive = new Drivetrain();
  private final CoralSubsystem m_lift = new CoralSubsystem();
  private final XboxController m_controller = new XboxController(OperatorConstants.kDriverControllerPort);
  private Command m_autonomousCommand;

  private void configureBindings() {
    new JoystickButton(m_controller, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_drive.setStationary(),
            m_drive));

    new JoystickButton(m_controller, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_lift.release(),
            m_lift));
  }

  @Override
  public void robotInit() {
    configureBindings();

    // If the doesn't work, comment it and uncomment the below drive function
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(
                OperatorConstants.kXSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(
                        m_controller.getLeftY(),
                        OperatorConstants.kStickDeadband)),
                -OperatorConstants.kYSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(
                        m_controller.getLeftX(),
                        OperatorConstants.kStickDeadband)),
                -OperatorConstants.kTSlewRateLimiter
                    .calculate(MathUtil.applyDeadband(
                        m_controller.getRightX(),
                        OperatorConstants.kStickDeadband)),
                true),
            m_drive));

    m_lift.setDefaultCommand(
        new RunCommand(
            () -> m_lift.move(OperatorConstants.kLiftSlewRateLimiter
                .calculate(m_controller.getRightTriggerAxis()
                    - m_controller.getLeftTriggerAxis())),
            m_lift));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeed,
        AutoConstants.kMaxAcceleration)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // End 3 meters straight behind of where we started, facing backward
          new Pose2d(2, 0, new Rotation2d(180))),
        config);

    var thetaController = new ProfiledPIDController(
        1, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController,
        m_drive::setModuleStates,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    m_autonomousCommand = swerveControllerCommand
      .andThen(() -> m_drive.drive(0, 0, 0, false))
      .andThen(() -> m_lift.move(1.0)).withTimeout(2.0)
      .andThen(() -> m_lift.release()).withTimeout(2.0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
}
