package frc.robot.autos;

import frc.robot.autos.auto_path_commands.DoNothingAuto;
import frc.robot.autos.auto_path_commands.PushPartnerAuto;
import frc.robot.autos.auto_path_commands.ThreePiece1DEC;
import frc.robot.autos.auto_path_commands.ThreePiece2IJKAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),
  THREE_PIECE_2IJK(ThreePiece2IJKAuto::new),
  PUSH_PARTNER(PushPartnerAuto::new),
  THREE_PIECE_1DEC(ThreePiece1DEC::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
