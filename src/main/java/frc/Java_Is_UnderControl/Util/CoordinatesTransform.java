package frc.Java_Is_UnderControl.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class CoordinatesTransform {
  public static Pose2d fromRobotRelativeToF(Pose2d robotPose, Pose2d poseForConversion) {
    Pose2d poseInTheField = robotPose
        .plus(new Transform2d(poseForConversion.getTranslation(), poseForConversion.getRotation()));
    return poseInTheField;
  }
}
