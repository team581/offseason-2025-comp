package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.Autos;
import frc.robot.autos.Trailblazer;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.purple.Purple;
import frc.robot.robot_manager.GamePieceMode;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.collision_avoidance.CollisionBox;
import frc.robot.roll.RollSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.wrist.WristSubsystem;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private final FmsSubsystem fms = new FmsSubsystem();
  private final Hardware hardware = new Hardware();
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(hardware.elevatorLeftMotor, hardware.elevatorRightMotor);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);
  private final Limelight elevatorPurpleLimelight =
      new Limelight(
          "elev",
          LimelightState.PURPLE,
          RobotConfig.get().vision().interpolatedVisionSet().elevatorPurpleSet,
          LimelightModel.THREE);
  private final Limelight frontCoralLimelight =
      new Limelight(
          "front",
          LimelightState.TAGS,
          RobotConfig.get().vision().interpolatedVisionSet().frontCoralSet,
          LimelightModel.FOUR);
  private final Limelight backTagLimelight =
      new Limelight(
          "back",
          LimelightState.TAGS,
          RobotConfig.get().vision().interpolatedVisionSet().backTagSet,
          LimelightModel.THREEG);
  private final Limelight baseTagLimelight =
      new Limelight(
          "base",
          LimelightState.TAGS,
          RobotConfig.get().vision().interpolatedVisionSet().baseTagSet,
          LimelightModel.THREEG);

  private final VisionSubsystem vision =
      new VisionSubsystem(
          imu, elevatorPurpleLimelight, frontCoralLimelight, backTagLimelight, baseTagLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final Purple purple = new Purple(elevatorPurpleLimelight);

  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(hardware.driverController, true);

  private final IntakeSubsystem intake =
      new IntakeSubsystem(hardware.intakeTopMotor, hardware.intakeBottomMotor, hardware.candi);

  private final WristSubsystem wrist = new WristSubsystem(hardware.wristMotor);
  private final RollSubsystem roll = new RollSubsystem(hardware.rollMotor, intake);
  private final LightsSubsystem lights =
      new LightsSubsystem(hardware.candle, elevatorPurpleLimelight);
  private final ClimberSubsystem climber =
      new ClimberSubsystem(/* hardware.climberMotor, hardware.climberCANcoder */ );
  private final RobotManager robotManager =
      new RobotManager(
          intake,
          wrist,
          elevator,
          roll,
          vision,
          imu,
          swerve,
          localization,
          elevatorPurpleLimelight,
          frontCoralLimelight,
          backTagLimelight,
          baseTagLimelight,
          lights,
          purple,
          climber,
          rumbleController);

  private final RobotCommands robotCommands = new RobotCommands(robotManager);

  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false).withNtPublish(RobotConfig.IS_DEVELOPMENT));
    // DogLog.setPdh(hardware.pdh);

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        DogLog.log("Metadata/GitDirty", "All changes committed");
        break;
      case 1:
        DogLog.log("Metadata/GitDirty", "Uncomitted changes");
        break;
      default:
        DogLog.log("Metadata/GitDirty", "Unknown");
        break;
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();

    CollisionBox.visualize();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    Stopwatch.getInstance().start("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    Stopwatch.getInstance().stop("Scheduler/CommandSchedulerPeriodic");

    if (RobotController.getBatteryVoltage() < 12.5) {
      DogLog.logFault("Battery voltage low", AlertType.kWarning);
    } else {
      DogLog.clearFault("Battery voltage low");
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.run(
            () -> {
              if (DriverStation.isTeleop()) {
                swerve.driveTeleop(
                    hardware.driverController.getLeftX(),
                    hardware.driverController.getLeftY(),
                    hardware.driverController.getRightX());
              }
            }));

    hardware.driverController.rightTrigger().onTrue(robotCommands.confirmScoreCommand());
    hardware
        .driverController
        .leftTrigger()
        .onTrue(robotCommands.floorAssistIntakeCommand())
        .onFalse(robotCommands.floorIntakeCommand());
    hardware.driverController.rightBumper().onTrue(robotCommands.stowCommand());
    hardware.driverController.leftBumper().onTrue(robotCommands.intakeStationCommand());
    hardware.driverController.y().onTrue(robotCommands.highLineupCommand());
    hardware.driverController.x().onTrue(robotCommands.l3LineupCommand());
    hardware.driverController.b().onTrue(robotCommands.l2LineupCommand());
    hardware.driverController.a().onTrue(robotCommands.lowLineupCommand());
    hardware.driverController.povUp().onTrue(robotCommands.climbUpCommand());
    hardware.driverController.povDown().onTrue(robotCommands.climbDownCommand());
    hardware
        .driverController
        .povLeft()
        .onTrue(robotCommands.setGamepieceModeCommand(GamePieceMode.CORAL));
    hardware
        .driverController
        .povRight()
        .onTrue(robotCommands.setGamepieceModeCommand(GamePieceMode.ALGAE));
    hardware.driverController.start().onTrue(robotCommands.rehomeRollCommand());
    hardware.driverController.back().onTrue(localization.getZeroCommand());

    hardware.operatorController.a().onTrue(robotCommands.rehomeElevatorCommand());
    hardware.operatorController.b().onTrue(robotCommands.rehomeWristCommand());
    hardware.operatorController.y().onTrue(robotCommands.rehomeRollCommand());
    hardware.operatorController.x().onTrue(robotCommands.unjamCommand());
  }
}
