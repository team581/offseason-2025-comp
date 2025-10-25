package frc.robot.config;

import frc.robot.vision.limelight.LimelightModel;

public record CameraConfig(LimelightModel model, boolean useMtp1, double forward, double right, double up, double pitch, double yaw, double roll) {

}
