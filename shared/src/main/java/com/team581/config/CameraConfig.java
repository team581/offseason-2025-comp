package com.team581.config;

public record CameraConfig(
    LimelightModel model,
    boolean useMegatag1RotationWhenClose,
    double forward,
    double right,
    double up,
    double pitch,
    double yaw,
    double roll) {}
