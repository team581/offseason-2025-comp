package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueFrontThreePiece2IKLAuto;
import frc.robot.autos.auto_path_commands.blue.BluePushPartnerAuto;
import frc.robot.autos.auto_path_commands.blue.BlueThreePiece2IKLAuto;
import frc.robot.autos.auto_path_commands.blue.BlueThreePiece5FDC;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedFrontThreePiece2IKLAuto;
import frc.robot.autos.auto_path_commands.red.RedPushPartnerAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece2IKLAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece5FDC;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  // FOUR_PIECE_2IJK(RedFourPiece2IJKLAuto::new, BlueFourPiece2IJKLAuto::new),
  PUSH_PARTNER(RedPushPartnerAuto::new, BluePushPartnerAuto::new),
  // FOUR_PIECE_5FEDC(RedFourPiece5FEDC::new, BlueFourPiece5FEDC::new),
  THREE_PIECE_2IKL(RedThreePiece2IKLAuto::new, BlueThreePiece2IKLAuto::new),
  FRONT_THREE_PIECE_2IKL(RedFrontThreePiece2IKLAuto::new, BlueFrontThreePiece2IKLAuto::new),
  THREE_PIECE_5FDC(RedThreePiece5FDC::new, BlueThreePiece5FDC::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
