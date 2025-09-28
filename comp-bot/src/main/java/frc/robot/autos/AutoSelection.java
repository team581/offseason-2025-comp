package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipop4PieceR1AB;
import frc.robot.autos.auto_path_commands.blue.BlueLollipop4PieceR6AB;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipop4PieceR1AB;
import frc.robot.autos.auto_path_commands.red.RedLollipop4PieceR6AB;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  LOLLIPOP_AB_LEFT(RedLollipop4PieceR1AB::new, BlueLollipop4PieceR1AB::new),
  LOLLIPOP_AB_RIGHT(RedLollipop4PieceR6AB::new, BlueLollipop4PieceR6AB::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
