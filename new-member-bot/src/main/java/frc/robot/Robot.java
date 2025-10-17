package frc.robot;

import com.team581.Base581Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends Base581Robot {
  private final Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrain);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, swerve);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    finalizeInit();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
    //   fieldCalibrationUtil.log();
    // }
  }

  @Override
  public void autonomousInit() {
    super.autonomousInit();

    // autonomousCommand = autos.getAutoCommand();
    // autonomousCommand.schedule();
  }

  @Override
  public void teleopInit() {
    super.teleopInit();

    autonomousCommand.cancel();
  }

  @Override
  protected void configureBindings() {
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  if (DriverStation.isTeleop()) {
                    swerve.driveTeleop(
                        hardware.driverController.getLeftX(),
                        hardware.driverController.getLeftY(),
                        hardware.driverController.getRightX());
                  }
                })
            .ignoringDisable(true)
            .withName("DefaultSwerveCommand"));

    // hardware.driverController.leftTrigger().onTrue(actions.groundIntakeCommand());
    // hardware.driverController.rightBumper().onTrue(actions.stowCommand());
    hardware.driverController.back().onTrue(localization.getZeroCommand());
  }
}
