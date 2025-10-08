package frc.robot;

import com.team581.Base581Robot;
import com.team581.GlobalConfig;
import com.team581.controller.RumbleControllerSubsystem;
import com.team581.trailblazer.Trailblazer;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.field_calibration.FieldCalibrationUtil;
import frc.robot.autos.Autos;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;
import java.lang.management.ManagementFactory;
import java.lang.management.OperatingSystemMXBean;

public class Robot extends Base581Robot {
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final DeploySubsystem deploy = new DeploySubsystem(hardware.deployMotor);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);
  private final Limelight leftBackLimelight =
      new Limelight("leftb", LimelightState.TAGS, LimelightModel.THREEG, true);
  private final Limelight leftFrontLimelight =
      new Limelight("leftf", LimelightState.TAGS, LimelightModel.THREEG, true);

  private final Limelight rightLimelight =
      new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR, true);

  private final Limelight gamePieceDetectionLimelight =
      new Limelight("gp", LimelightState.CORAL, LimelightModel.THREE, false);

  private final VisionSubsystem vision =
      new VisionSubsystem(
          imu, leftBackLimelight, leftFrontLimelight, rightLimelight, gamePieceDetectionLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(hardware.elevatorLeftMotor, hardware.elevatorRightMotor);
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(
          hardware.driverController, true, SubsystemPriority.RUMBLE_CONTROLLER);

  private final ClawSubsystem claw = new ClawSubsystem(hardware.clawMotor, hardware.clawCandi);
  private final IntakeSubsystem intake =
      new IntakeSubsystem(hardware.intakeMotor, hardware.intakeCandi);

  private final ArmSubsystem arm = new ArmSubsystem(hardware.armMotor, elevator);
  private final LightsSubsystem lights = new LightsSubsystem(hardware.candle);
  private final ClimberSubsystem climber =
      new ClimberSubsystem(
          hardware.climberClimbMotor,
          hardware.climberCANcoder,
          hardware.climberGrabMotor,
          hardware.climberCanrange);
  private final AutoAlign autoAlign = new AutoAlign(vision, localization, swerve);
  private final CoralMap coralMap = new CoralMap(localization, swerve, gamePieceDetectionLimelight);
  private final GroundManager gm = new GroundManager(deploy, intake);
  private final RobotManager robotManager =
      new RobotManager(
          gm,
          claw,
          arm,
          elevator,
          vision,
          imu,
          swerve,
          localization,
          coralMap,
          lights,
          autoAlign,
          climber,
          rumbleController);
  private final FieldCalibrationUtil fieldCalibrationUtil =
      new FieldCalibrationUtil(elevator, arm, lights, localization);

  private final RobotCommands robotCommands = new RobotCommands(robotManager);
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    CollisionAvoidance.warmup();

    finalizeInit();
  }

  private final OperatingSystemMXBean operatingSystemMxBean =
      ManagementFactory.getOperatingSystemMXBean();

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      fieldCalibrationUtil.log();
    }

    var cpuLoad =
        operatingSystemMxBean.getSystemLoadAverage()
            / operatingSystemMxBean.getAvailableProcessors();
    DogLog.log("Debug/CpuLoad", cpuLoad);
  }

  @Override
  public void autonomousInit() {
    super.autonomousInit();

    autonomousCommand = autos.getAutoCommand();
    autonomousCommand.schedule();

    autoAlign.clearReefState();
  }

  @Override
  public void teleopInit() {
    super.teleopInit();

    autonomousCommand.cancel();

    if (GlobalConfig.IS_DEVELOPMENT) {
      autoAlign.clearReefState();
    }
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

    hardware.driverController.rightTrigger().onTrue(robotCommands.confirmScoreCommand());
    hardware.driverController.leftTrigger().onTrue(robotCommands.floorIntakeCommand());
    hardware.driverController.leftBumper().onTrue(robotCommands.algaeIntakeGroundCommand());
    hardware.driverController.rightBumper().onTrue(robotCommands.stowCommand());
    hardware
        .driverController
        .y()
        .onTrue(robotCommands.highLineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .x()
        .onTrue(robotCommands.l3LineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .b()
        .onTrue(robotCommands.l2LineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());
    hardware
        .driverController
        .a()
        .onTrue(robotCommands.lowLineupCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());

    hardware.driverController.povUp().onTrue(robotCommands.climbUpCommand());
    hardware.driverController.povDown().onTrue(robotCommands.climbStopCommand());
    hardware.driverController.povLeft().onTrue(robotCommands.lowStowCommand());
    hardware
        .driverController
        .povRight()
        .onTrue(robotCommands.algaeReefIntakeCommand())
        .onFalse(robotCommands.scoringAlignOffCommand());

    hardware.driverController.start().onTrue(robotCommands.forcedL1Request());
    hardware.driverController.back().onTrue(localization.getZeroCommand());

    hardware.operatorController.y().onTrue(robotCommands.rehomeDeployCommand());

    hardware.operatorController.back().onTrue(robotCommands.spinToWinCommand());

    // hardware.operatorController.rightTrigger().onTrue(robotCommands.waitAlgaeFlingCommand());
    hardware.operatorController.leftTrigger().onTrue(robotCommands.testNextLollipopCommand());
    hardware.operatorController.povUp().onTrue(robotCommands.forcedHandoffCommand());
    hardware.operatorController.povDown().onTrue(robotCommands.forcedLowStowCommand());

    hardware.operatorController.povLeft().onTrue(robotCommands.manualReefSwitchLeftCommand());
    hardware.operatorController.povRight().onTrue(robotCommands.manualReefSwitchRightCommand());
  }
}
