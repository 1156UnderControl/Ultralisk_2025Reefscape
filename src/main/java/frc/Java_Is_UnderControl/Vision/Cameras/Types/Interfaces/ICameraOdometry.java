package frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces;

import java.util.Optional;

import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public interface ICameraOdometry {

  public Optional<AprilTagData> getAprilTagData();

  public void updateLogsAprilTag();

  public Optional<PoseEstimation> getRobotPose();
}
