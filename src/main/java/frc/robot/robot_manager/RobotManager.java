package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldUtil;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraStatus;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  private static final double MINIMUM_SHOT_TIME = 0.5;
  public final ArmSubsystem arm;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ImuSubsystem imu;
  public final IntakeSubsystem intake;
  public final QueuerSubsystem queuer;
  public final SwerveSubsystem swerve;
  private final Timer shotTimer = new Timer();

  private DistanceAngle fieldRelativeDistanceAngleToSpeaker = new DistanceAngle(0, 0, false);
  private DistanceAngle fieldRelativeDistanceAngleToFeedSpot = new DistanceAngle(0, 0, false);
  private boolean facingSpeakerAngle = false;
  private boolean facingFeedSpotAngle = false;

  private boolean confirmShotActive = false;

  public RobotManager(
      ArmSubsystem arm,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ImuSubsystem imu,
      IntakeSubsystem intake,
      QueuerSubsystem queuer,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.arm = arm;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.imu = imu;
    this.intake = intake;
    this.queuer = queuer;
    this.swerve = swerve;
  }

  @Override
  protected void collectInputs() {
    fieldRelativeDistanceAngleToSpeaker =
        localization.getFieldRelativeDistanceAngleToPose(FieldUtil.getSpeakerPose());
    fieldRelativeDistanceAngleToFeedSpot =
        localization.getFieldRelativeDistanceAngleToPose(FieldUtil.getFeedSpotPose());

    var robotHeading = imu.getRobotHeading();

    facingSpeakerAngle =
        MathUtil.isNear(fieldRelativeDistanceAngleToSpeaker.targetAngle(), robotHeading, 3);
    facingFeedSpotAngle =
        MathUtil.isNear(fieldRelativeDistanceAngleToFeedSpot.targetAngle(), robotHeading, 3);
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case IDLE_NO_GP,
              IDLE_WITH_GP,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING,
              OUTTAKING,
              AMP_SCORING,
              SPEAKER_WAITING,
              AMP_WAITING,
              SUBWOOFER_WAITING,
              FEEDING_WAITING,
              PODIUM_WAITING ->
          currentState;

      case FEEDING_SHOOTING, PASS_SHOOTING, SUBWOOFER_SCORING, PODIUM_SCORING ->
          queuer.hasNote() ? currentState : RobotState.IDLE_NO_GP;
      case SPEAKER_SCORING ->
          queuer.hasNote() || !shotTimer.hasElapsed(MINIMUM_SHOT_TIME)
              ? currentState
              : RobotState.IDLE_NO_GP;

      case SPEAKER_PREPARE_TO_SCORE ->
          (shooter.atGoal() && arm.atGoal())
                  && (DriverStation.isAutonomous() || confirmShotActive)
                  && swerve.isSlowEnoughToShoot()
                  && ((DriverStation.isAutonomous()
                          && vision.getVisionState() == CameraStatus.OFFLINE)
                      || vision.getVisionState() == CameraStatus.SEES_TAGS)
                  && facingSpeakerAngle
              ? RobotState.SPEAKER_SCORING
              : currentState;

      case AMP_PREPARE_TO_SCORE ->
          shooter.atGoal() && arm.atGoal() ? RobotState.AMP_SCORING : currentState;

      case FEEDING_PREPARE_TO_SHOOT ->
          shooter.atGoal() && arm.atGoal() && facingFeedSpotAngle && swerve.isSlowEnoughToFeed()
              ? RobotState.FEEDING_SHOOTING
              : currentState;
      case PASS_PREPARE_TO_SHOOT ->
          shooter.atGoal() && arm.atGoal() ? RobotState.PASS_SHOOTING : currentState;
      case SUBWOOFER_PREPARE_TO_SCORE ->
          shooter.atGoal() && arm.atGoal() ? RobotState.SUBWOOFER_SCORING : currentState;
      case PODIUM_PREPARE_TO_SCORE ->
          shooter.atGoal() && arm.atGoal() ? RobotState.PODIUM_SCORING : currentState;

      case UNJAM -> currentState;
      case BEFORE_INTAKING -> arm.atGoal() ? RobotState.INTAKING : currentState;
      case BEFORE_INTAKING_ASSIST -> arm.atGoal() ? RobotState.INTAKE_ASSIST : currentState;

      case INTAKING, INTAKE_ASSIST -> {
        if (!queuer.hasNote()) {
          yield currentState;
        }

        if (confirmShotActive) {
          yield RobotState.SPEAKER_PREPARE_TO_SCORE;
        }

        yield RobotState.IDLE_WITH_GP;
      }
        // case INTAKING_BACK -> !queuer.hasNote() ? RobotState.INTAKING_FORWARD_PUSH :
        // currentState;
        // case INTAKING_FORWARD_PUSH -> {
        //   if (!queuer.atGoal()) {
        //     yield currentState;
        //   }

        //   if (confirmShotActive) {
        //     yield RobotState.SPEAKER_PREPARE_TO_SCORE;
        //   }

        //   yield RobotState.IDLE_WITH_GP;
        // }
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case SUBWOOFER_PREPARE_TO_SCORE, SUBWOOFER_WAITING -> {
        arm.setState(ArmState.SUBWOOFER_SHOT);
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getSubwooferAngle());
      }
      case PODIUM_PREPARE_TO_SCORE, PODIUM_WAITING -> {
        arm.setState(ArmState.PODIUM_SHOT);
        shooter.setState(ShooterState.PODIUM_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getPodiumAngle());
      }
      case PODIUM_SCORING -> {
        arm.setState(ArmState.PODIUM_SHOT);
        shooter.setState(ShooterState.PODIUM_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getPodiumAngle());
      }
      case SUBWOOFER_SCORING -> {
        arm.setState(ArmState.SUBWOOFER_SHOT);
        shooter.setState(ShooterState.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getSubwooferAngle());
      }
      case SPEAKER_PREPARE_TO_SCORE -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToSpeaker.targetAngle());
        shotTimer.stop();
        shotTimer.reset();
      }
      case SPEAKER_WAITING -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToSpeaker.targetAngle());
      }
      case SPEAKER_SCORING -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToSpeaker.targetAngle());
        shotTimer.start();
      }
      case AMP_PREPARE_TO_SCORE, AMP_WAITING -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.IDLE_WARMUP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getAmpAngle());
      }
      case AMP_SCORING -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.IDLE_WARMUP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.AMPING);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getAmpAngle());
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_WAITING -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToFeedSpot.targetAngle());
      }
      case FEEDING_SHOOTING -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToFeedSpot.targetAngle());
      }
      case PASS_PREPARE_TO_SHOOT -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case PASS_SHOOTING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case UNJAM -> {
        arm.setState(ArmState.UNJAM);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.OUTTAKING);
        queuer.setState(QueuerState.SHOOTING);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case INTAKING -> {
        arm.setState(ArmState.HOLD_FOR_INTAKE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.INTAKING);
        queuer.setState(QueuerState.INTAKING);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case BEFORE_INTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case BEFORE_INTAKING_ASSIST -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.INTAKE_ASSIST_TELEOP);
        } else {
          swerve.setState(SwerveState.INTAKE_ASSIST_AUTO);
        }
      }
        // case INTAKING_BACK -> {
        //   arm.setState(ArmState.IDLE);
        //   shooter.setState(ShooterState.IDLE_STOPPED);
        //   intake.setState(IntakeState.INTAKING_BACK);
        //   queuer.setState(QueuerState.INTAKING_BACK);
        //   swerve.setSnapsEnabled(false);
        //   swerve.setSnapToAngle(0);
        // }
        // case INTAKING_FORWARD_PUSH -> {
        //   arm.setState(ArmState.IDLE);
        //   shooter.setState(ShooterState.IDLE_STOPPED);
        //   intake.setState(IntakeState.INTAKING_FORWARD_PUSH);
        //   queuer.setState(QueuerState.INTAKING_FORWARD_PUSH);
        //   swerve.setSnapsEnabled(false);
        //   swerve.setSnapToAngle(0);
        // }
      case INTAKE_ASSIST -> {
        arm.setState(ArmState.HOLD_FOR_INTAKE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.INTAKING);
        queuer.setState(QueuerState.INTAKING);
        // We don't use the setSnaps here, since intake assist is a separate state
        if (DriverStation.isTeleop()) {
          swerve.setState(SwerveState.INTAKE_ASSIST_TELEOP);
        } else {
          swerve.setState(SwerveState.INTAKE_ASSIST_AUTO);
        }
      }
      case OUTTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.OUTTAKING);
        queuer.setState(QueuerState.OUTTAKING);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case CLIMBING_1_LINEUP -> {
        arm.setState(ArmState.CLIMBING_1_LINEUP);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(true);
        swerve.setSnapToAngle(SnapUtil.getClimbingAngle(imu));
      }
      case CLIMBING_2_HANGING -> {
        arm.setState(ArmState.CLIMBING_2_HANGING);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case IDLE_NO_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
      case IDLE_WITH_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_WARMUP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE);
        swerve.setSnapsEnabled(false);
        swerve.setSnapToAngle(0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Continuous state actions
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE, SPEAKER_SCORING, SPEAKER_WAITING -> {
        shooter.setDistanceToSpeaker(fieldRelativeDistanceAngleToSpeaker.distance());
        arm.setDistanceToSpeaker(fieldRelativeDistanceAngleToSpeaker.distance());
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToSpeaker.targetAngle());
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_SHOOTING, FEEDING_WAITING -> {
        shooter.setDistanceToFeedSpot(fieldRelativeDistanceAngleToFeedSpot.distance());
        swerve.setSnapToAngle(fieldRelativeDistanceAngleToFeedSpot.targetAngle());
        arm.setDistanceToFeedSpot(fieldRelativeDistanceAngleToFeedSpot.distance());
      }
      case INTAKING, INTAKE_ASSIST -> {
        if (arm.atGoal()) {
          intake.setState(IntakeState.INTAKING);
        } else {
          intake.setState(IntakeState.IDLE);
        }
      }
        // case INTAKING_BACK -> {
        //   if (arm.atGoal()) {
        //     intake.setState(IntakeState.INTAKING_BACK);
        //   } else {
        //     intake.setState(IntakeState.IDLE);
        //   }
        // }
        // case INTAKING_FORWARD_PUSH -> {
        //   if (arm.atGoal()) {
        //     intake.setState(IntakeState.INTAKING_FORWARD_PUSH);
        //   } else {
        //     intake.setState(IntakeState.IDLE);
        //   }
        // }
      default -> {}
    }
    DogLog.log("RobotManager/shooteratGoal", shooter.atGoal());
    DogLog.log("RobotManager/armAtGoal", arm.atGoal());
    DogLog.log("RobotManager/confirmShotActive", confirmShotActive);
    DogLog.log("RobotManager/isSlowEnoughToShoot", swerve.isSlowEnoughToShoot());
    DogLog.log("RobotManager/visionState", vision.getVisionState());
    DogLog.log("RobotManager/facingspekaer", facingSpeakerAngle);
    DogLog.log("RobotManager/facingFeedSpotAngle", facingFeedSpotAngle);
    DogLog.log(
        "RobotManager/goalAngleforspeaker", fieldRelativeDistanceAngleToSpeaker.targetAngle());
    DogLog.log("RobotManager/distancetoFeedSpot", fieldRelativeDistanceAngleToFeedSpot.distance());

    DogLog.log("RobotManager/minimunShotTimer", shotTimer.get());
  }

  public void setConfirmShotActive(boolean newValue) {
    confirmShotActive = newValue;
  }

  public void confirmShotRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST
      // INTAKING_BACK,
      // INTAKING_FORWARD_PUSH
      -> {}

      case AMP_WAITING -> setStateFromRequest(RobotState.AMP_PREPARE_TO_SCORE);
      case SPEAKER_WAITING -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
      case FEEDING_WAITING -> setStateFromRequest(RobotState.FEEDING_PREPARE_TO_SHOOT);
      case SUBWOOFER_WAITING -> setStateFromRequest(RobotState.SUBWOOFER_PREPARE_TO_SCORE);
      case PODIUM_WAITING -> setStateFromRequest(RobotState.PODIUM_PREPARE_TO_SCORE);
      default -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
    }
  }

  public void waitAmpRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          AMP_SCORING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.AMP_WAITING);
    }
  }

  public void waitSubwooferRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          SPEAKER_SCORING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.SUBWOOFER_WAITING);
    }
  }

  public void waitPodiumRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          SPEAKER_SCORING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.PODIUM_WAITING);
    }
  }

  public void waitSpeakerRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {
        if (DriverStation.isAutonomous()) {
          // Bypass intake checks if we're in auto
          setStateFromRequest(RobotState.SPEAKER_WAITING);
        }
      }
      default -> setStateFromRequest(RobotState.SPEAKER_WAITING);
    }
  }

  public void unjamRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void intakeRequest() {
    if (!queuer.hasNote()) {
      switch (getState()) {
        case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
        default -> setStateFromRequest(RobotState.BEFORE_INTAKING);
      }
    }
  }

  public void intakeAssistRequest() {
    if (!queuer.hasNote()) {
      switch (getState()) {
        case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
        default -> setStateFromRequest(RobotState.BEFORE_INTAKING_ASSIST);
      }
    }
  }

  public void outtakeRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.OUTTAKING);
    }
  }

  public void idleWithGpRequest() {
    setStateFromRequest(RobotState.IDLE_WITH_GP);
  }

  public void stowRequest() {
    switch (getState()) {
        // TODO: Intaking and unjam should not be IDLE_WITH_GP
      case INTAKING,
              INTAKE_ASSIST,
              AMP_PREPARE_TO_SCORE,
              SPEAKER_PREPARE_TO_SCORE,
              FEEDING_PREPARE_TO_SHOOT,
              PASS_PREPARE_TO_SHOOT,
              AMP_WAITING,
              SPEAKER_WAITING,
              FEEDING_WAITING,
              AMP_SCORING,
              SPEAKER_SCORING,
              FEEDING_SHOOTING,
              PASS_SHOOTING,
              IDLE_WITH_GP,
              UNJAM
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH
          ->
          setStateFromRequest(RobotState.IDLE_WITH_GP);
      default -> setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void preparePassRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.PASS_PREPARE_TO_SHOOT);
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_WITH_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void prepareSpeakerRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {
        if (DriverStation.isAutonomous()) {
          // Bypass intake checks if we're in auto
          setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
        }
      }
      default -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
    }
  }

  public void prepareAmpRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.AMP_PREPARE_TO_SCORE);
    }
  }

  public void prepareFeedRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.FEEDING_PREPARE_TO_SHOOT);
    }
  }

  public void waitFeedRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.FEEDING_WAITING);
    }
  }

  public void stopShootingRequest() {
    // If we are actively taking a shot, ignore the request to avoid messing up shooting
    switch (getState()) {
      case SPEAKER_SCORING,
          SUBWOOFER_SCORING,
          AMP_SCORING,
          AMP_WAITING,
          FEEDING_SHOOTING,
          PASS_SHOOTING -> {}

      default -> setStateFromRequest(RobotState.IDLE_WITH_GP);
    }
  }

  public void prepareSubwooferRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.SUBWOOFER_PREPARE_TO_SCORE);
    }
  }

  public void preparePodiumRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          // INTAKING_BACK,
          // INTAKING_FORWARD_PUSH,
          INTAKING,
          INTAKE_ASSIST,
          BEFORE_INTAKING,
          BEFORE_INTAKING_ASSIST -> {}
      default -> setStateFromRequest(RobotState.PODIUM_PREPARE_TO_SCORE);
    }
  }
}
