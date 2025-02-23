package frc.robot.autos;

import frc.robot.autos.auto_path_commands.DoNothingAuto;
import frc.robot.autos.auto_path_commands.FourPiece2IJKLAuto;
import frc.robot.autos.auto_path_commands.FourPiece5FEDC;
import frc.robot.autos.auto_path_commands.PushPartnerAuto;
import frc.robot.autos.auto_path_commands.StraightLineAuto;
import frc.robot.autos.auto_path_commands.TestAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),
  FOUR_PIECE_2IJK(FourPiece2IJKLAuto::new),
  PUSH_PARTNER(PushPartnerAuto::new),
  FOUR_PIECE_5FEDC(FourPiece5FEDC::new),
  STRAIGHT_LINE(StraightLineAuto::new),
  TEST_AUTO(TestAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
