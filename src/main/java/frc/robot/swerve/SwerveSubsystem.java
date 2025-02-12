package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.MagnetismUtil;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Map;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  // TODO: Remove this once magnetism is stable, with current way robot manager is, having both of
  // these enabled doesn't work
  private static final boolean MAGNETISM_ENABLED = false;
  private static final boolean PURPLE_ALIGN_ENABLED = true;

  private static final boolean INTAKE_ASSIST_CORAL_ENABLED = true;

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.15;
  private static final double leftYDeadband = 0.05;

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(40.0, 1.0), Map.entry(40.1, 0.5));

  public final TunerSwerveDrivetrain drivetrain =
      RobotConfig.IS_PRACTICE_BOT
          ? new TunerSwerveDrivetrain(
              PracticeBotTunerConstants.DrivetrainConstants,
              PracticeBotTunerConstants.FrontLeft,
              PracticeBotTunerConstants.FrontRight,
              PracticeBotTunerConstants.BackLeft,
              PracticeBotTunerConstants.BackRight)
          : new TunerSwerveDrivetrain(
              CompBotTunerConstants.DrivetrainConstants,
              CompBotTunerConstants.FrontLeft,
              CompBotTunerConstants.FrontRight,
              CompBotTunerConstants.BackLeft,
              CompBotTunerConstants.BackRight);

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

  private double lastSimTime;
  private Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private double goalSnapAngle = 0;

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

  private ChassisSpeeds magnetizedSpeeds = new ChassisSpeeds();
  private ChassisSpeeds coralAssistSpeedsOffset = new ChassisSpeeds();
  private ChassisSpeeds purpleSpeeds = new ChassisSpeeds();

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTimestamp = 0.0;

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
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

  private double elevatorHeight;

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    driveToAngle.HeadingController = RobotConfig.get().swerve().snapController();
    driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

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

  public void setFieldRelativeCoralAssistSpeedsOffset(ChassisSpeeds speeds) {
    coralAssistSpeedsOffset = speeds;
  }

  public void setPurpleSpeeds(ChassisSpeeds speeds) {
    purpleSpeeds = speeds;
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    return switch (currentState) {
      case AUTO, TELEOP -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.TELEOP;
      case INTAKE_ASSIST_ALGAE_TELEOP, INTAKE_ASSIST_CORAL_TELEOP ->
          DriverStation.isAutonomous() ? SwerveState.AUTO : currentState;
      case REEF_ALIGN_TELEOP, REEF_ALIGN_AUTO ->
          DriverStation.isAutonomous()
              ? SwerveState.REEF_ALIGN_AUTO
              : SwerveState.REEF_ALIGN_TELEOP;
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
    DogLog.log("Swerve/RightX", rightX);
    Translation2d mappedpose = ControllerHelpers.fromCircularDiscCoordinates(leftX, leftY);
    double mappedX = mappedpose.getX();
    double mappedY = mappedpose.getY();

    var slowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * mappedY * MaxSpeed * slowModePercent,
            mappedX * MaxSpeed * slowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians() * slowModePercent);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    magnetizedSpeeds =
        MAGNETISM_ENABLED
            ? MagnetismUtil.getReefMagnetizedChassisSpeeds(teleopSpeeds, drivetrainState.Pose)
            : teleopSpeeds;
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
      case INTAKE_ASSIST_CORAL_TELEOP -> {
        drivetrain.setControl(
            drive
                .withVelocityX(
                    teleopSpeeds.vxMetersPerSecond + coralAssistSpeedsOffset.vxMetersPerSecond)
                .withVelocityY(
                    teleopSpeeds.vyMetersPerSecond + coralAssistSpeedsOffset.vyMetersPerSecond)
                .withRotationalRate(
                    teleopSpeeds.omegaRadiansPerSecond
                        + coralAssistSpeedsOffset.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
      case REEF_ALIGN_TELEOP -> {
        var alignSpeeds = getScoringAlignChassisSpeeds();
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond + alignSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond + alignSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

        } else {

          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond + alignSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond + alignSpeeds.vyMetersPerSecond)
                  .withRotationalRate(
                      teleopSpeeds.omegaRadiansPerSecond + alignSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case REEF_ALIGN_AUTO -> {
        var wantedSpeeds = getScoringAlignChassisSpeeds();
        //  var wantedSpeeds = alignSpeeds.plus(autoSpeeds);
        var currentTimestamp = Timer.getFPGATimestamp();
        if (previousTimestamp == 0.0) {
          previousTimestamp = currentTimestamp - 0.02;
        }
        var constrainedWantedSpeeds =
            AutoConstraintCalculator.constrainVelocityGoal(
                wantedSpeeds,
                previousSpeeds,
                currentTimestamp - previousTimestamp,
                AutoConstraintCalculator.getLastUsedConstraints());

        if (constrainedWantedSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(constrainedWantedSpeeds.vxMetersPerSecond)
                  .withVelocityY(constrainedWantedSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.Velocity));
        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(constrainedWantedSpeeds.vxMetersPerSecond)
                  .withVelocityY(constrainedWantedSpeeds.vyMetersPerSecond)
                  .withRotationalRate(constrainedWantedSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity));
        }

        previousSpeeds = getFieldRelativeSpeeds();
        previousTimestamp = currentTimestamp;
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

  public void enableScoringAlignment() {
    if (MAGNETISM_ENABLED || PURPLE_ALIGN_ENABLED) {
      if (DriverStation.isAutonomous()) {
        // No magnetism in auto, use regular snaps
        setStateFromRequest(SwerveState.REEF_ALIGN_AUTO);

      } else {
        setStateFromRequest(SwerveState.REEF_ALIGN_TELEOP);
      }
    }
  }

  public ChassisSpeeds getScoringAlignChassisSpeeds() {
    if (MAGNETISM_ENABLED) {
      // TODO: Magnetism should be a no-op in auto >:(
      return magnetizedSpeeds;
    } else if (PURPLE_ALIGN_ENABLED) {
      return purpleSpeeds;
    }

    return new ChassisSpeeds();
  }

  public void enableCoralIntakeAssist() {
    if (DriverStation.isTeleop()) {
      if (INTAKE_ASSIST_CORAL_ENABLED) {
        setStateFromRequest(SwerveState.INTAKE_ASSIST_CORAL_TELEOP);
      } else {
        setStateFromRequest(SwerveState.TELEOP);
      }
    } else {
      setStateFromRequest(SwerveState.AUTO);
    }
  }

  public void setSnapsEnabled(boolean newValue) {
    switch (getState()) {
      case TELEOP,
              TELEOP_SNAPS,
              INTAKE_ASSIST_CORAL_TELEOP,
              REEF_ALIGN_TELEOP,
              INTAKE_ASSIST_ALGAE_TELEOP ->
          setStateFromRequest(newValue ? SwerveState.TELEOP_SNAPS : SwerveState.TELEOP);
      case AUTO, AUTO_SNAPS, REEF_ALIGN_AUTO ->
          setStateFromRequest(newValue ? SwerveState.AUTO_SNAPS : SwerveState.AUTO);
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Swerve/SnapAngle", goalSnapAngle);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
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

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }
}
