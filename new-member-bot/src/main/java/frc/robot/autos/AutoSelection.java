package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueMiddle2AlgaeAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedMiddle2AlgaeAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  R3_ONE_CORAL_TWO_ALGAE(RedMiddle2AlgaeAuto::new, BlueMiddle2AlgaeAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseCommandAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseCommandAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseCommandAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseCommandAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
