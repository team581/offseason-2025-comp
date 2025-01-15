package frc.robot.autos;

import frc.robot.autos.auto_commands.DoNothingAuto;
import frc.robot.autos.auto_commands.PushPartnerAuto;
import frc.robot.autos.auto_commands.Three3Piece2IJKAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),
  THREE_PIECE_2IJK(Three3Piece2IJKAuto::new),
  PUSH_PARTNER(PushPartnerAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
