package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.RumbleControllerSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.singulator.SingulatorSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Robot extends Base581Robot {
  private final Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrain);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, swerve);
  private final RumbleControllerSubsystem rumble =
      new RumbleControllerSubsystem(
          hardware.driverController, true, SubsystemPriority.RUMBLE_CONTROLLER);

  private final IntakeSubsystem intake = new IntakeSubsystem(hardware.intakeMotor);
  private final DeploySubsystem deploy = new DeploySubsystem(hardware.deployMotor);
  private final SingulatorSubsystem singulator =
      new SingulatorSubsystem(hardware.leftSingulatorMotor, hardware.rightSingulatorMotor);

  private final GroundManager groundManager =
      new GroundManager(intake, deploy, singulator, hardware.topIntakeCANdi, hardware.bottomIntakeCANdi);
      private final ClawSubsystem claw = new ClawSubsystem(hardware.clawMotor, hardware.clawCaNdi);
      private final ElevatorSubsystem elevator = new ElevatorSubsystem(hardware.elevatorMotor);
      private final ArmSubsystem arm = new ArmSubsystem(hardware.armMotor, elevator);


      private final Limelight leftLimelight= new Limelight("left", LimelightState.TAGS, LimelightModel.THREEG, true);
      private final Limelight rightlLimelight= new Limelight("left", LimelightState.TAGS, LimelightModel.THREEG, true);

      private final VisionSubsystem vision = new VisionSubsystem(imu, leftLimelight, rightlLimelight, leftLimelight, rightlLimelight);
      private final LightsSubsystem lights = new LightsSubsystem(hardware.candle);
      private final AutoAlign autoAlign = new AutoAlign(vision, localization, swerve, true);
      private final ClimberSubsystem climber =
          new ClimberSubsystem(
              hardware.climberMotor, hardware.climberCANcoder);
  private final RobotManager robotManager =
      new RobotManager(
          groundManager,
          claw,
          arm,
          elevator,
          vision,
          imu,
          swerve,
          localization,
          lights,
          autoAlign,
          climber,
          rumble);

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

    hardware.driverController.leftTrigger().onTrue(actions.groundIntakeCommand());
    hardware.driverController.rightBumper().onTrue(actions.stowCommand());
    hardware.driverController.back().onTrue(localization.getZeroCommand());
    hardware.operatorController.y().onTrue(actions.rehomeDeployCommand());
  }
}
