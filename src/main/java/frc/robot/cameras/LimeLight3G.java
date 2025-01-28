package frc.robot.cameras;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight.LimelightCamera;

public class LimeLight3G {
  ICameraOdometry camera = new LimelightCamera("limelight-reef", new Translation3d(0.826772, 13.971834, 12.43898),
      new Rotation3d(Units.degreesToRadians(4), Units.degreesToRadians(-23), 0), false);

  public AprilTagData getAprilTagData() {
    return camera.getAprilTagData();
  }
}
