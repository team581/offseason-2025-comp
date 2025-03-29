package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopAuto;
import frc.robot.autos.auto_path_commands.red.RedPushPartnerAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece2IKLAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  THREE_PIECE_2IKL(RedThreePiece2IKLAuto::new, RedThreePiece2IKLAuto::new),
  LOLLIPOP(RedLollipopAuto::new, RedLollipopAuto::new),
  PUSH_PARTNER(RedPushPartnerAuto::new, RedPushPartnerAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
