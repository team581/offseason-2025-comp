package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopLeftL4Auto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopLeftL4Auto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece1IKLAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece2ILKAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece3GHJ;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  THREE_PIECE_2ILK(RedThreePiece2ILKAuto::new, RedThreePiece2ILKAuto::new),
  LOLLIPOP_LEFT_L4(RedLollipopLeftL4Auto::new, BlueLollipopLeftL4Auto::new),
  LOLLIPOP_LEFT_AB(RedLollipopLeftABAuto::new, BlueLollipopLeftABAuto::new),

  THREE_PIECE_3GHJ(RedThreePiece3GHJ::new, RedThreePiece3GHJ::new),
  GROUND_THREE_PIECE_1IKL(RedThreePiece1IKLAuto::new, RedThreePiece1IKLAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
