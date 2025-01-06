package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.security.InvalidParameterException;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  /** Max speed allowed to make a speaker shot and feeding. */
  private static final double MAX_SPEED_SHOOTING = Units.feetToMeters(0.5);

  private static final double MAX_FLOOR_SPEED_SHOOTING = Units.feetToMeters(1.0);

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.15;
  private static final double leftYDeadband = 0.05;

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private static SwerveModuleConstants constantsForModuleNumber(int moduleNumber) {
    return switch (moduleNumber) {
      case 0 -> TunerConstants.FrontLeft;
      case 1 -> TunerConstants.FrontRight;
      case 2 -> TunerConstants.BackLeft;
      case 3 -> TunerConstants.BackRight;
      default -> throw new InvalidParameterException("Expected an ID from [0, 3]");
    };
  }

  public final SwerveDrivetrain drivetrain =
      new SwerveDrivetrain(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  private final SwerveModule frontLeft = drivetrain.getModule(0);
  private final SwerveModule frontRight = drivetrain.getModule(1);
  private final SwerveModule backLeft = drivetrain.getModule(2);
  private final SwerveModule backRight = drivetrain.getModule(3);

  private double lastSimTime;
  private Notifier simNotifier = null;

  private boolean slowEnoughToShoot = false;
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private boolean slowEnoughToFeed;
  private double goalSnapAngle = 0;

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public boolean isSlowEnoughToShoot() {
    return slowEnoughToShoot;
  }

  public boolean isSlowEnoughToFeed() {
    return slowEnoughToFeed;
  }

  public SwerveDriveState getDrivetrainState() {
    return drivetrainState;
  }

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;

    // We don't necessarily set auto swerve speeds every loop, so this ensures we are always snapped
    // to the right angle during auto. Teleop doesn't need this since teleop speeds are constantly
    // fed into swerve.
    switch (getState()) {
      case AUTO_SNAPS -> {
        sendSwerveRequest();
      }
    }
  }

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    driveToAngle.HeadingController = RobotConfig.get().swerve().snapController();
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveToAngle.HeadingController.setTolerance(0.02);

    // The CTR SwerveModule class will overwrite your torque current limits and the stator current
    // limit with the configured slip current. This logic allows us to exercise more precise control
    // over what current limits are used for each control mode.
    // See https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/81
    for (int i = 0; i < 4; i++) {
      var module = drivetrain.getModule(i);
      var driveMotorConfigurator = module.getDriveMotor().getConfigurator();
      var steerMotorConfigurator = module.getSteerMotor().getConfigurator();
      var usedConstants = constantsForModuleNumber(i);

      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorConfig().CurrentLimits);
      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorConfig().Voltage);
      driveMotorConfigurator.apply(RobotConfig.get().swerve().driveMotorConfig().OpenLoopRamps);
      driveMotorConfigurator.apply(
          new TorqueCurrentConfigs()
              .withPeakForwardTorqueCurrent(80)
              .withPeakReverseTorqueCurrent(80));

      MotorOutputConfigs driveMotorOutput =
          RobotConfig.get().swerve().driveMotorConfig().MotorOutput;
      driveMotorOutput.Inverted =
          usedConstants.DriveMotorInverted
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      driveMotorConfigurator.apply(driveMotorOutput);

      steerMotorConfigurator.apply(RobotConfig.get().swerve().steerMotorConfig().CurrentLimits);
      steerMotorConfigurator.apply(RobotConfig.get().swerve().steerMotorConfig().Voltage);
      MotorOutputConfigs steerMotorOutput =
          RobotConfig.get().swerve().steerMotorConfig().MotorOutput;
      steerMotorOutput.Inverted =
          usedConstants.SteerMotorInverted
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      steerMotorConfigurator.apply(steerMotorOutput);
    }

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
    sendSwerveRequest();
  }

  public void setRobotRelativeAutoSpeeds(ChassisSpeeds speeds) {
    setFieldRelativeAutoSpeeds(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            speeds, Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble())));
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    return switch (currentState) {
      case AUTO, TELEOP -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.TELEOP;
      case INTAKE_ASSIST_AUTO, INTAKE_ASSIST_TELEOP ->
          DriverStation.isAutonomous()
              ? SwerveState.INTAKE_ASSIST_AUTO
              : SwerveState.INTAKE_ASSIST_TELEOP;
      case AUTO_SNAPS, TELEOP_SNAPS ->
          DriverStation.isAutonomous() ? SwerveState.AUTO_SNAPS : SwerveState.TELEOP_SNAPS;
    };
  }

  public void driveTeleop(double x, double y, double theta) {
    double leftY =
        -1.0
            * ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(y, leftYDeadband), 1.5);
    double leftX =
        ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(x, leftXDeadband), 1.5);
    double rightX =
        ControllerHelpers.getExponent(ControllerHelpers.getDeadbanded(theta, rightXDeadband), 2);

    if (RobotConfig.get().swerve().invertRotation()) {
      rightX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertX()) {
      leftX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertY()) {
      leftY *= -1.0;
    }

    if (FmsSubsystem.isRedAlliance()) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    DogLog.log("Swerve/LeftX", leftX);
    DogLog.log("Swerve/LeftY", leftY);
    Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
    double mappedX = mappedpose.getX();
    double mappedY = mappedpose.getY();

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * mappedY * MaxSpeed,
            mappedX * MaxSpeed,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians());

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.speeds;
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    slowEnoughToShoot = calculateMovingSlowEnoughForSpeakerShot(robotRelativeSpeeds);
    slowEnoughToFeed = calculateMovingSlowEnoughForFloorShot(robotRelativeSpeeds);
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, drivetrainState.Pose.getRotation());
  }

  private static boolean calculateMovingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean calculateMovingSlowEnoughForFloorShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FLOOR_SPEED_SHOOTING;
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP ->
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      case TELEOP_SNAPS -> {
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case AUTO ->
          drivetrain.setControl(
              drive
                  .withVelocityX(autoSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                  .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity));
      case AUTO_SNAPS ->
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(autoSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.Velocity));
    }
  }

  public void setState(SwerveState newState) {
    setStateFromRequest(newState);
  }

  public void setSnapsEnabled(boolean newValue) {
    switch (getState()) {
      case TELEOP, TELEOP_SNAPS, INTAKE_ASSIST_TELEOP ->
          setStateFromRequest(newValue ? SwerveState.TELEOP_SNAPS : SwerveState.TELEOP);
      case AUTO, AUTO_SNAPS, INTAKE_ASSIST_AUTO ->
          setStateFromRequest(newValue ? SwerveState.AUTO_SNAPS : SwerveState.AUTO);
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Swerve/SnapAngle", goalSnapAngle);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.speeds);
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
