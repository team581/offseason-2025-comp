package frc.robot.auto_align.tag_align;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.google.common.collect.ImmutableList;
import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Comparator;
import java.util.Optional;

public class TagAlign {
  public static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());
  public static final ImmutableList<ReefSide> ALL_REEF_SIDES =
      ImmutableList.copyOf(ReefSide.values());
  public static final double L1_TRACKING_TIMEOUT = 15.0;

  private static final ProfiledPIDController TRAPEZOIDAL_ROTATION_CONTROLLER =
      new ProfiledPIDController(
          5.0,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

  private static final ProfiledPIDController TRAPEZOIDAL_TRANSLATION_CONTROLLER =
      new ProfiledPIDController(4.0, 0.0, 0.0, new Constraints(4.0, 4.0));

  private static final PhoenixPIDController ROTATION_CONTROLLER =
      new PhoenixPIDController(5.0, 0.0, 0.0);
  private static final PIDController VELOCITY_CONTROLLER = new PIDController(3.7, 0.0, 0.0);

  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", Units.inchesToMeters(1.0));
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 3.0);

  private static final DoubleSubscriber NEAR_ROTATION_GOAL =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 10.0);

  private static final DoubleSubscriber NEED_TO_MOVE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveTranslation", Units.inchesToMeters(1.5));
  private static final DoubleSubscriber NEED_TO_MOVE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveRotation", 3.0);

  private static final DoubleSubscriber IN_RANGE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeTranslation", Units.inchesToMeters(1.5));
  private static final DoubleSubscriber IN_RANGE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeRotation", 3.0);

  private static final double PIPE_SWITCH_TIMEOUT = 0.5;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private final ReefState reefState = new ReefState();

  private ReefPipeLevel pipeLevel = ReefPipeLevel.RAISING;
  private ReefPipeLevel preferedScoringLevel = ReefPipeLevel.L4;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private double lastPipeSwitchTimestamp = 0.0;
  private boolean aligned = false;
  private boolean resetReefPipeNextLoop = false;
  private boolean pipeSwitchActive = false;

  private static final DoubleSubscriber TRANSLATION_FEED_FORWARD =
      DogLog.tunable("AutoAlign/TranslationFeedForward", 0.0);
  private static final DoubleSubscriber ROTATION_FEED_FORWARD =
      DogLog.tunable("AutoAlign/RotationFeedForward", 0.0);

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
    TRAPEZOIDAL_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setLevel(ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
    this.robotScoringSide = side;
    alignmentCostUtil.setSide(robotScoringSide);
    this.pipeLevel = level;
    this.preferedScoringLevel = preferredLevel;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setControllerValues(double controllerXValue, double controllerYValue) {
    rawControllerXValue = controllerXValue;
    rawControllerYValue = controllerYValue;
    checkControllerForSwitch();
  }

  private void checkControllerForSwitch() {
    if (!DriverStation.isTeleop()) {
      return;
    }
    if (pipeSwitchActive
        && (Timer.getFPGATimestamp() > (lastPipeSwitchTimestamp + PIPE_SWITCH_TIMEOUT))
        && rawControllerXValue == 0.0) {
      pipeSwitchActive = false;
    }
    if (pipeSwitchActive) {
      return;
    }
    if ((Math.hypot(rawControllerXValue, rawControllerYValue) > 0.5)) {
      var storedPipe = getBestPipe();
      pipeSwitchActive = true;
      lastPipeSwitchTimestamp = Timer.getFPGATimestamp();

      var inputVector = new Translation2d(rawControllerXValue, -rawControllerYValue);
      var viewOffset = 0;
      if (FmsUtil.isRedAlliance()) {
        viewOffset = 180;
      }

      var sideAngle = ReefSide.fromPipe(storedPipe).getPose(FmsUtil.isRedAlliance());

      var rotatedVector =
          inputVector.rotateBy(
              Rotation2d.fromDegrees((viewOffset - sideAngle.getRotation().getDegrees())));
      var rotatedVectorLeft = rotatedVector.getX() < 0;
      ReefPipe leftPipe =
          switch (storedPipe) {
            case PIPE_A, PIPE_B -> ReefPipe.PIPE_A;
            case PIPE_C, PIPE_D -> ReefPipe.PIPE_C;
            case PIPE_E, PIPE_F -> ReefPipe.PIPE_E;
            case PIPE_G, PIPE_H -> ReefPipe.PIPE_G;
            case PIPE_I, PIPE_J -> ReefPipe.PIPE_I;
            case PIPE_K, PIPE_L -> ReefPipe.PIPE_K;
          };
      ReefPipe rightPipe =
          switch (storedPipe) {
            case PIPE_A, PIPE_B -> ReefPipe.PIPE_B;
            case PIPE_C, PIPE_D -> ReefPipe.PIPE_D;
            case PIPE_E, PIPE_F -> ReefPipe.PIPE_F;
            case PIPE_G, PIPE_H -> ReefPipe.PIPE_H;
            case PIPE_I, PIPE_J -> ReefPipe.PIPE_J;
            case PIPE_K, PIPE_L -> ReefPipe.PIPE_L;
          };
      var partnerPipe = ReefPipe.PIPE_A;
      if (rotatedVectorLeft) {
        DogLog.timestamp("AutoAlign/PipeSwitch/Left");
        partnerPipe = leftPipe;
      } else {
        DogLog.timestamp("AutoAlign/PipeSwitch/Right");
        partnerPipe = rightPipe;
      }
      reefState.removeCoral(partnerPipe, preferedScoringLevel);
      setPipeOveride(partnerPipe);
    }
  }

  public boolean isAligned(ReefPipe pipe) {
    if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return false;
    }
    var robotPose = localization.getPose();
    var targetPose = getUsedScoringPose(pipe);

    var translationGood =
        (robotPose.getTranslation().getDistance(targetPose.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            targetPose.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);

    return translationGood && rotationGood;
  }

  public boolean isNearRotationGoal(ReefPipe pipe) {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var rotationGood =
        MathUtil.isNear(
            scoringPoseFieldRelative.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            NEAR_ROTATION_GOAL.get(),
            -180.0,
            180.0);
    return rotationGood;
  }

  public boolean needToMove(Pose2d goal) {
    var robotPose = localization.getPose();
    var translationBad =
        (robotPose.getTranslation().getDistance(goal.getTranslation())
            > NEED_TO_MOVE_TRANSLATION_THRESHOLD.get());
    var rotationBad =
        !MathUtil.isNear(
            goal.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            NEED_TO_MOVE_ROTATION_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationBad || rotationBad;
  }

  public boolean inRange(Pose2d goal) {
    var robotPose = localization.getPose();
    var translationGood =
        (robotPose.getTranslation().getDistance(goal.getTranslation())
            <= IN_RANGE_TRANSLATION_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            goal.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            IN_RANGE_ROTATION_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationGood && rotationGood;
  }

  public void markScored(ReefPipe pipe) {
    reefState.markCoralScored(pipe, preferedScoringLevel);
  }

  public void markAlgaeRemoved(ReefSide side) {
    reefState.markAlgaeRemoved(side);
  }

  public boolean isAlgaeRemoved(ReefSide side) {
    return reefState.isAlgaeRemoved(side);
  }

  public void reset() {
    resetReefPipeNextLoop = true;
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return getUsedScoringPose(pipe, robotScoringSide);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, RobotScoringSide side) {
    return pipe.getPose(pipeLevel, side, localization.getPose());
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if ((DriverStation.isAutonomous() || pipeSwitchActive) && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }
    var level = pipeLevel;

    if (pipeLevel.equals(ReefPipeLevel.BACK_AWAY)
        || preferedScoringLevel.equals(ReefPipeLevel.L1)) {
      return getClosestPipe();
    }
    if (pipeLevel.equals(ReefPipeLevel.RAISING)) {
      level = preferedScoringLevel;
    }
    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public ReefPipe getClosestPipe() {
    var robotPose = localization.getPose();
    return ALL_REEF_PIPES.stream()
        .min(
            Comparator.comparingDouble(
                pipe ->
                    robotPose
                        .getTranslation()
                        .getDistance(
                            pipe.getPose(pipeLevel, robotScoringSide, robotPose).getTranslation())))
        .orElseThrow();
  }

  public ReefSide getBestAlgaeSide() {
    return ALL_REEF_SIDES.stream().min(alignmentCostUtil.getAlgaeComparator()).orElseThrow();
  }

  public PolarChassisSpeeds getPoseAlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds) {

    if (FeatureFlags.AUTO_ALIGN_DEADBAND.getAsBoolean()) {
      if (aligned || inRange(targetPose)) {
        if (needToMove(targetPose)) {
          aligned = false;
        } else {
          aligned = true;
          return new PolarChassisSpeeds();
        }
      }
    }

    // Calculate x and y velocities
    double distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    if (resetReefPipeNextLoop) {
      DogLog.timestamp("AutoAlign/ResetControllers");
      TRAPEZOIDAL_ROTATION_CONTROLLER.reset(
          currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
      TRAPEZOIDAL_ROTATION_CONTROLLER.setGoal(targetPose.getRotation().getRadians());
      var linearFieldVelocity =
          new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
      TRAPEZOIDAL_TRANSLATION_CONTROLLER.reset(
          distanceToGoalMeters,
          Math.min(
              0.0,
              -linearFieldVelocity
                  .rotateBy(
                      targetPose
                          .getTranslation()
                          .minus(currentPose.getTranslation())
                          .getAngle()
                          .unaryMinus())
                  .getX()));
      TRAPEZOIDAL_TRANSLATION_CONTROLLER.setGoal(0);
    }
    resetReefPipeNextLoop = false;

    var driveVelocityMagnitude =
        TRAPEZOIDAL_TRANSLATION_CONTROLLER.calculate(
            distanceToGoalMeters,
            new State(0, 0),
            new Constraints(constraints.maxLinearVelocity(), constraints.maxLinearAcceleration()));

    if (MathUtil.isNear(
        targetPose.getRotation().getRadians(),
        TRAPEZOIDAL_ROTATION_CONTROLLER.getSetpoint().position,
        1e-6)) {}

    ;

    var rotationSpeed =
        TRAPEZOIDAL_ROTATION_CONTROLLER.calculate(
            currentPose.getRotation().getRadians(),
            new State(targetPose.getRotation().getRadians(), 0));

    if (!FeatureFlags.AUTO_ALIGN_TRAPEZOIDAL.getAsBoolean()) {
      driveVelocityMagnitude = VELOCITY_CONTROLLER.calculate(distanceToGoalMeters);
      rotationSpeed =
          ROTATION_CONTROLLER.calculate(
              currentPose.getRotation().getRadians(),
              targetPose.getRotation().getRadians(),
              Timer.getFPGATimestamp());
    }

    if (Math.abs(distanceToGoalMeters) > Units.inchesToMeters(1.0)) {
      driveVelocityMagnitude +=
          Math.copySign(TRANSLATION_FEED_FORWARD.get(), driveVelocityMagnitude);
    }

    if (!MathUtil.isNear(
        targetPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), 1.0)) {
      rotationSpeed +=
          Math.copySign(Units.rotationsToRadians(ROTATION_FEED_FORWARD.get()), rotationSpeed);
    }

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    var speeds = new PolarChassisSpeeds(driveVelocityMagnitude, driveDirection, rotationSpeed);
    DogLog.log("AutoAlign/TargetPose", targetPose);
    DogLog.log("AutoAlign/Speeds", speeds);

    return speeds;
  }
}
