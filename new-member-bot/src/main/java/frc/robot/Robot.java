package frc.robot;

import com.team581.Base581Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.AutoAlign;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.wrist.WristSubsystem;

public class Robot extends Base581Robot {
  private final Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final ElevatorSubsystem elevator = new ElevatorSubsystem(hardware.elevatorMotor);
  private final WristSubsystem wrist = new WristSubsystem(hardware.wristMotor, elevator);
  private final ClawSubsystem claw = new ClawSubsystem(hardware.clawMotor);
  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          hardware.climberClimbMotor,
          hardware.climberCANcoder,
          hardware.climberGrabMotor,
          hardware.climberCANrange);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrain);

  private final Limelight limelight =
      new Limelight("main", LimelightState.TAGS, RobotConfig.get().vision().mainLimelightConfig());

  private final VisionSubsystem vision = new VisionSubsystem(imu, limelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, swerve, vision);
  private final AutoAlign autoAlign = new AutoAlign(vision, localization, swerve, false);

  private final RobotManager robotManager =
      new RobotManager(
          claw, elevator, wrist, climber, localization, autoAlign, vision, swerve, imu);
  private final RobotCommands actions = new RobotCommands(robotManager);

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

    hardware.driverController.leftTrigger().onTrue(actions.algaeGroundIntakeCommand());
    hardware.driverController.rightTrigger().onTrue(actions.confirmScoreCommand());

    hardware
        .driverController
        .leftBumper()
        .onTrue(actions.algaeReefIntakeCommand())
        .onFalse(actions.scoringAlignOffCommand());
    hardware.driverController.rightBumper().onTrue(actions.intakeCoralCommand());

    hardware.driverController.y().onTrue(actions.netWaitCommand());
    hardware.driverController.x().onTrue(actions.stowCommand());
    hardware.driverController.a().onTrue(actions.lowLineupCommand());

    hardware.driverController.povUp().onTrue(actions.climberSequenceForwardCommand());
    hardware.driverController.povDown().onTrue(actions.climberSequenceStopCommand());

    hardware.driverController.start().onTrue(actions.unjamCommand());
    hardware.driverController.back().onTrue(localization.getZeroCommand());

    hardware.operatorController.a().onTrue(actions.rehomeElevatorCommand());
    hardware.operatorController.y().onTrue(actions.rehomeWristCommand());
  }
}
