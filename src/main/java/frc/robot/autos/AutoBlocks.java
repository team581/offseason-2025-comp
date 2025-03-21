package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
  /**
   * The tolerance used to determine when to end the little "backup & stow" motion we do after
   * scoring L4.
   */
  private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.3, 10);

  /**
   * The offset used to calculate the position to go to before PIDing to the lineup pose. Ensures we
   * don't strafe into the scoring position, just a straight-on movement.
   */
  private static final Transform2d PIPE_LINEUP_OFFSET = new Transform2d(-0.6, 0, Rotation2d.kZero);

  private static final Transform2d PIPE_APPROACH_OFFSET =
      new Transform2d(-1.3, 0, Rotation2d.kZero);

  public static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(4.7, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }
}
