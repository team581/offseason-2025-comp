package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueFourPiece2IJKAGroundAuto;
import frc.robot.autos.auto_path_commands.blue.BlueFourPiece5EDCBGround;
import frc.robot.autos.auto_path_commands.blue.BlueOldPushPartnerAuto;
import frc.robot.autos.auto_path_commands.blue.BlueThreePiece3JKLAuto;
import frc.robot.autos.auto_path_commands.blue.BlueThreePiece4EDCAuto;
import frc.robot.autos.auto_path_commands.red.RedBackThreePiece3JKLAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedFourPiece2IJKAGroundAuto;
import frc.robot.autos.auto_path_commands.red.RedFourPiece5EDCBGround;
import frc.robot.autos.auto_path_commands.red.RedOldPushPartnerAuto;
import frc.robot.autos.auto_path_commands.red.RedOldThreePiece2IKLAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece3JKLAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece4EDCAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),

  PUSH_PARTNER(RedOldPushPartnerAuto::new, BlueOldPushPartnerAuto::new),
  FOUR_PIECE_2IJKA_GROUND(RedFourPiece2IJKAGroundAuto::new, BlueFourPiece2IJKAGroundAuto::new),
  FOUR_PIECE_5EDCB_GROUND(RedFourPiece5EDCBGround::new, BlueFourPiece5EDCBGround::new),

  // STRAIGHT_LINE(RedStraightLineAuto::new, BlueDoNothingAuto::new),

  THREE_PIECE_3JKL(RedThreePiece3JKLAuto::new, BlueThreePiece3JKLAuto::new),
  THREE_PIECE_4EDC(RedThreePiece4EDCAuto::new, BlueThreePiece4EDCAuto::new),
 // THREE_PIECE_2IKL(RedThreePiece2IKLAuto::new, RedThreePiece2IKLAuto::new),
  BACK_THREE_PIECE_3JKL(RedBackThreePiece3JKLAuto::new, RedBackThreePiece3JKLAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
