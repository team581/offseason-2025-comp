package frc.robot.autos;

import frc.robot.autos.auto_path_commands.DoNothingAuto;
import frc.robot.autos.auto_path_commands.FourPiece1EDC;
import frc.robot.autos.auto_path_commands.FourPiece2IJKAuto;
import frc.robot.autos.auto_path_commands.PushPartnerAuto;
import frc.robot.autos.auto_path_commands.StraightLineAuto;
import frc.robot.autos.auto_path_commands.TestAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),
  FOUR_PIECE_2IJK(FourPiece2IJKAuto::new),
  PUSH_PARTNER(PushPartnerAuto::new),
  FOUR_PIECE_1EDC(FourPiece1EDC::new),
  STRAIGHT_LINE(StraightLineAuto::new),
  TEST_AUTO(TestAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
