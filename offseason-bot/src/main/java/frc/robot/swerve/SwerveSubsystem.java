package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.GlobalConfig;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.SwerveBase;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;

public class SwerveSubsystem extends StateMachineSubsystem<SwerveState> implements SwerveBase {
  private static final double LEFT_JOYSTICK_EXPONENT = 2;
  private static final double RIGHT_JOYSTICK_EXPONENT = 2;

  private static final PIDController DRIVE_TO_POSE_TRANSLATION_CONTROLLER =
      new PIDController(3.5, 0.0, 0.0);
  private static final PIDController DRIVE_TO_POSE_ROTATION_CONTROLLER =
      new PIDController(4.0, 0.0, 0.0);
  private static final DoubleSubscriber DRIVE_TO_POSE_TRANSLATION_FF =
      DogLog.tunable("Swerve/DriveToPose/TranslationFF", 0.0);
  private static final DoubleSubscriber DRIVE_TO_POSE_ROTATION_FF =
      DogLog.tunable("Swerve/DriveToPose/RotationFF", 0.0);
  private static final DoubleSubscriber MAX_TRANSLATION_VELOCITY_LIMIT =
      DogLog.tunable("Swerve/DriveToPose/MaxTranslationVelMet", 2.5);

  private static final DoubleSubscriber MAX_ROTATION_VELOCITY_LIMIT_ROT =
      DogLog.tunable("Swerve/DriveToPose/MaxRotationVelRot", 2.5);

  public static final double MaxSpeed = 4.75;
  private static final double maxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      RobotConfig.get().swerve().snapController();

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(0.0, 1.0));

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(maxAngularRate);

  private double lastSimTime;
  private Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private double goalSnapAngle = 0;

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

  private ChassisSpeeds driveToPoseSpeeds = new ChassisSpeeds();

  private Pose2d lastDriveToPoseTarget = new Pose2d();
  private boolean lastUseAngleBisector = true;
  private double lastUsedMaxVelocity = MAX_TRANSLATION_VELOCITY_LIMIT.get();

  private final Timer timeSinceAutoSpeeds = new Timer();
  private double teleopSlowModePercent = 0.0;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  @Override
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;

    // We don't necessarily set auto swerve speeds every loop, so this ensures we are always snapped
    // to the right angle during auto. Teleop doesn't need this since teleop speeds are constantly
    // fed into swerve.
    if (DriverStation.isAutonomous()) {
      sendSwerveRequest();
    }
  }

  private double elevatorHeight;

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    driveToAngle.HeadingController.setTolerance(0.01);
    DRIVE_TO_POSE_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
    timeSinceAutoSpeeds.start();
  }

  @Override
  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
    timeSinceAutoSpeeds.reset();
    trailblazerDriveRequest();
    sendSwerveRequest();
  }

  public void setRobotRelativeAutoSpeeds(ChassisSpeeds speeds) {
    setFieldRelativeAutoSpeeds(
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drivetrainState.Pose.getRotation()));
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    return switch (currentState) {
      case TRAILBLAZER, TELEOP ->
          DriverStation.isAutonomous() ? SwerveState.TRAILBLAZER : SwerveState.TELEOP;
      default -> currentState;
    };
  }

  public void driveTeleop(double x, double y, double theta) {
    rawControllerXValue = x;
    rawControllerYValue = y;

    if (GlobalConfig.IS_DEVELOPMENT) {
      DogLog.log("Swerve/RawJoystickInput", new Translation2d(x, y));
    }

    var leftJoystickMagnitude =
        ControllerHelpers.getJoystickMagnitude(x, y, LEFT_JOYSTICK_EXPONENT);
    var rightJoystickMagnitude =
        ControllerHelpers.getJoystickMagnitude(theta, 0, RIGHT_JOYSTICK_EXPONENT);

    var translation =
        new Translation2d(
            leftJoystickMagnitude, x == 0 && y == 0 ? Rotation2d.kZero : new Rotation2d(x, y));
    var rotation =
        new Translation2d(
            rightJoystickMagnitude, theta == 0 ? Rotation2d.kZero : new Rotation2d(theta, 0));

    var leftX = translation.getX();
    var leftY = -1 * translation.getY();
    var rightX = rotation.getX();

    if (RobotConfig.get().swerve().invertRotation()) {
      rightX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertX()) {
      leftX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertY()) {
      leftY *= -1.0;
    }

    if (FmsUtil.isRedAlliance()) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * leftY * MaxSpeed * teleopSlowModePercent,
            leftX * MaxSpeed * teleopSlowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
    if (getState().equals(SwerveState.DRIVE_TO_POSE)) {
      driveToPoseSpeeds =
          getDriveToPoseSpeeds(lastDriveToPoseTarget, drivetrainState.Pose, lastUseAngleBisector, lastUsedMaxVelocity);
    }
  }

  public ChassisSpeeds getTeleopSpeeds() {
    return teleopSpeeds;
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, drivetrainState.Pose.getRotation());
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
                  .withMaxAbsRotationalRate(
                      TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent)
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
      case DRIVE_TO_POSE -> {
        drivetrain.setControl(
            drive
                .withVelocityX(driveToPoseSpeeds.vxMetersPerSecond)
                .withVelocityY(driveToPoseSpeeds.vyMetersPerSecond)
                .withRotationalRate(driveToPoseSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

      case TRAILBLAZER -> {
        drivetrain.setControl(
            drive
                .withVelocityX(autoSpeeds.vxMetersPerSecond)
                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }
      case CLIMBING -> {
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond / 2)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond / 2)
                  .withMaxAbsRotationalRate(
                      TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond / 2)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond / 2)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
    }
  }

  public void normalDriveRequest() {
    if (DriverStation.isAutonomous()) {
      return;
    }
    setStateFromRequest(SwerveState.TELEOP);
  }

  public void trailblazerDriveRequest() {
    if (!DriverStation.isAutonomous()) {
      return;
    }
    setStateFromRequest(SwerveState.TRAILBLAZER);
  }

  public Translation2d getControllerValues() {
    if (rawControllerXValue == 0 && rawControllerYValue == 0) {
      return Translation2d.kZero;
    }

    return new Translation2d(
        ControllerHelpers.getJoystickMagnitude(
            rawControllerXValue, rawControllerYValue, LEFT_JOYSTICK_EXPONENT),
        new Rotation2d(rawControllerXValue, rawControllerYValue));
  }

  public void snapsDriveRequest(double snapAngle) {
    setSnapToAngle(snapAngle);

    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.TRAILBLAZER);
    } else {
      setStateFromRequest(SwerveState.TELEOP_SNAPS);
    }
  }

  public void driveToPoseRequest(Pose2d pose) {
    driveToPoseRequest(pose, true, MAX_TRANSLATION_VELOCITY_LIMIT.get());
  }

    public void driveToPoseRequest(Pose2d pose, double maxVelocity) {
      driveToPoseRequest(pose, true, maxVelocity);
    }

    public void driveToPoseRequest(Pose2d pose, boolean useAngleBisector) {
      driveToPoseRequest(pose, useAngleBisector, MAX_TRANSLATION_VELOCITY_LIMIT.get());
    }

  public void driveToPoseRequest(Pose2d pose, boolean useAngleBisector, double maxVelocity) {
    if (DriverStation.isTeleop()) {
      lastDriveToPoseTarget = pose;
      lastUseAngleBisector = useAngleBisector;
      driveToPoseSpeeds =
          getDriveToPoseSpeeds(lastDriveToPoseTarget, drivetrainState.Pose, lastUseAngleBisector, maxVelocity);
      setStateFromRequest(SwerveState.DRIVE_TO_POSE);
      sendSwerveRequest();
    }
  }

  public void climbRequest(double snapAngle) {
    setSnapToAngle(snapAngle);
    setStateFromRequest(SwerveState.CLIMBING);
  }

  @Override
  public void whileInState(SwerveState currentState) {
    DogLog.log("Swerve/SnapAngle", goalSnapAngle);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);

    if (timeSinceAutoSpeeds.hasElapsed(0.25) && DriverStation.isAutonomous()) {
      DogLog.logFault("Timeout since auto speeds last set");
    } else {
      DogLog.clearFault("Timeout since auto speeds last set");
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }

  private PolarChassisSpeeds getDriveToPoseSpeeds(
      Pose2d targetPose, Pose2d currentPose, boolean useAngleBisector, double maxVelocity) {
    // Calculate x and y velocities
    double distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    var driveVelocityMagnitude =
        DRIVE_TO_POSE_TRANSLATION_CONTROLLER.calculate(distanceToGoalMeters, 0);

    var rotationSpeed =
        DRIVE_TO_POSE_ROTATION_CONTROLLER.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (Math.abs(distanceToGoalMeters) > Units.inchesToMeters(1.0)) {
      driveVelocityMagnitude +=
          Math.copySign(DRIVE_TO_POSE_TRANSLATION_FF.get(), driveVelocityMagnitude);
    }

    if (!MathUtil.isNear(
        targetPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), 1.0)) {
      rotationSpeed +=
          Math.copySign(Units.rotationsToRadians(DRIVE_TO_POSE_ROTATION_FF.get()), rotationSpeed);
    }

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    if (useAngleBisector
        && Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
            > 0.1
        && distanceToGoalMeters > 0.1) {
      var wantedDirection =
          180 + MathHelpers.getDriveDirection(currentPose, targetPose).getDegrees();
      var currentSpeedDirection =
          Rotation2d.fromRadians(
                  Math.atan2(
                      fieldRelativeSpeeds.vyMetersPerSecond, fieldRelativeSpeeds.vxMetersPerSecond))
              .getDegrees();
      var bisectedAngle =
          MathHelpers.angleModulus(wantedDirection - currentSpeedDirection) / 2
              + currentSpeedDirection;
      DogLog.log("Swerve/DriveToPose/WantedDirection", wantedDirection);
      DogLog.log("Swerve/DriveToPose/CurrentDirection", currentSpeedDirection);
      driveDirection = Rotation2d.fromDegrees(bisectedAngle + 180);
    }

    driveVelocityMagnitude =
        MathUtil.clamp(
            driveVelocityMagnitude,
            -maxVelocity,
            maxVelocity);
    rotationSpeed =
        MathUtil.clamp(
            rotationSpeed,
            Units.rotationsToRadians(-MAX_ROTATION_VELOCITY_LIMIT_ROT.get()),
            Units.rotationsToRadians(MAX_ROTATION_VELOCITY_LIMIT_ROT.get()));

    var speeds = new PolarChassisSpeeds(driveVelocityMagnitude, driveDirection, rotationSpeed);
    DogLog.log("Swerve/DriveToPose/TargetPose", targetPose);
    DogLog.log("Swerve/DriveToPose/DistanceToTarget", distanceToGoalMeters);
    DogLog.log("Swerve/DriveToPose/Speeds", speeds);

    return speeds;
  }
}
