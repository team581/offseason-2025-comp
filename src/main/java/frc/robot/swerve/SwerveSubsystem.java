package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
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
import frc.robot.util.MathHelpers;
import frc.robot.util.ProfiledPhoenixPIDController;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Map;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  private static final ProfiledPhoenixPIDController SNAP_CONTROLLER =
      RobotConfig.get().swerve().snapController();

  // TODO: Remove this once magnetism is stable, with current way robot manager is, having both of
  // these enabled doesn't work
  private static final boolean MAGNETISM_ENABLED = false;
  private static final boolean AUTO_ALIGN_ENABLED = true;

  private static final boolean INTAKE_ASSIST_CORAL_ENABLED = true;

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.15;
  private static final double leftYDeadband = 0.05;

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      RobotConfig.get().swerve().snapController();

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(40.0, 1.0), Map.entry(40.1, 0.4));

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
          .withDeadband(MaxSpeed * 0.01)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

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

  private ChassisSpeeds autoAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds autoAlignAutoSpeeds = new ChassisSpeeds();

  private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private double previousTimestamp = 0.0;
  private double teleopSlowModePercent = 0.0;

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
    driveToAngle.HeadingController = SNAP_CONTROLLER;
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

  public void setAutoAlignAutoSpeeds(ChassisSpeeds speeds) {
    autoAlignAutoSpeeds = speeds;
    if (AUTO_ALIGN_ENABLED) {
      sendSwerveRequest();
    }
  }

  public void setAutoAlignSpeeds(ChassisSpeeds speeds) {
    autoAlignSpeeds = speeds;
    if (AUTO_ALIGN_ENABLED) {
      sendSwerveRequest();
    }
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
      case CLIMBING -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.CLIMBING;
    };
  }

  public void driveTeleop(double x, double y, double theta) {
    double leftY =
        -1.0
            * MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(y, leftYDeadband), 2.0);
    double leftX =
        MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(x, leftXDeadband), 2.0);
    double rightX =
        MathHelpers.signedExp(ControllerHelpers.deadbandJoystickValue(theta, rightXDeadband), 2.0);

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

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * mappedY * MaxSpeed * teleopSlowModePercent,
            mappedX * MaxSpeed * teleopSlowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);

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
    teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
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
          SNAP_CONTROLLER.setMaxOutput(
              TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);
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
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          SNAP_CONTROLLER.setMaxOutput(
              TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(autoAlignSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoAlignSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

        } else {

          drivetrain.setControl(
              drive
                  .withVelocityX(autoAlignSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoAlignSpeeds.vyMetersPerSecond)
                  .withRotationalRate(autoAlignSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case REEF_ALIGN_AUTO -> {
        SNAP_CONTROLLER.setMaxOutput(Double.POSITIVE_INFINITY);
        var wantedSpeeds = getAutoAlignAutoChassisSpeeds();
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
                // Our acceleration limit math is slightly wonky
                // So we turn it off here since velocity limiting is the good enough
                AutoConstraintCalculator.getLastUsedConstraints()
                    .withMaxAngularAcceleration(0)
                    .withMaxLinearAcceleration(0));

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
      case AUTO_SNAPS -> {
        SNAP_CONTROLLER.setMaxOutput(Double.POSITIVE_INFINITY);
        drivetrain.setControl(
            driveToAngle
                .withVelocityX(autoSpeeds.vxMetersPerSecond)
                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                .withDriveRequestType(DriveRequestType.Velocity));
      }
      case CLIMBING -> {
        SNAP_CONTROLLER.setMaxOutput(TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond / 2)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond / 2)
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

  public void setState(SwerveState newState) {
    setStateFromRequest(newState);
  }

  public void enableScoringAlignment() {
    if (MAGNETISM_ENABLED || AUTO_ALIGN_ENABLED) {
      if (DriverStation.isAutonomous()) {
        // No magnetism in auto, use regular snaps
        setStateFromRequest(SwerveState.REEF_ALIGN_AUTO);

      } else {
        setStateFromRequest(SwerveState.REEF_ALIGN_TELEOP);
      }
    }
  }

  public ChassisSpeeds getAutoAlignAutoChassisSpeeds() {
    if (MAGNETISM_ENABLED) {
      // TODO: Magnetism should be a no-op in auto >:(
      return magnetizedSpeeds;
    } else if (AUTO_ALIGN_ENABLED) {
      return autoAlignAutoSpeeds;
    }

    return new ChassisSpeeds();
  }

  public ChassisSpeeds getAutoAlignChassisSpeeds() {
    if (MAGNETISM_ENABLED) {
      // TODO: Magnetism should be a no-op in auto >:(
      return magnetizedSpeeds;
    } else if (AUTO_ALIGN_ENABLED) {
      return autoAlignSpeeds;
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
              INTAKE_ASSIST_ALGAE_TELEOP,
              CLIMBING ->
          setStateFromRequest(newValue ? SwerveState.TELEOP_SNAPS : SwerveState.TELEOP);
      case AUTO, AUTO_SNAPS, REEF_ALIGN_AUTO ->
          setStateFromRequest(newValue ? SwerveState.AUTO_SNAPS : SwerveState.AUTO);
    }
  }

  public void climbRequest() {
    setSnapToAngle(SnapUtil.getCageAngle());
    setState(SwerveState.CLIMBING);
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
