package frc.Java_Is_UnderControl.Vision.Cameras.Types;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Vision.Cameras.TargetData;

public class PhotonCameras {

  private static PhotonCameras instance;
  private PhotonCamera camera;
  private double cameraLensHeightMeters;
  private double goalHeightMeters;
  private double cameraMountAngleDEG;
  private Map<Integer, TargetData> aprilTagData = new HashMap<>();
  private Rotation2d robotRotation;
  private Transform3d camToRobot;

  public static PhotonCameras getInstance(String cameraName, boolean isTag) {
    if (instance == null) {
      instance = new PhotonCameras(cameraName, isTag);
      return instance;
    } else {
      return instance;
    }
  }

  private PhotonCameras(String cameraName, boolean isTag) {
    this.camera = new PhotonCamera(cameraName);
  }

  public void setCamToRobot(Translation3d translation, Rotation3d rotation) {
    this.camToRobot = new Transform3d(translation, rotation);
  }

  public Transform3d getCamToRobot() {
    return this.camToRobot;
  }

  public double getDistanceTarget() {
    var results = camera.getAllUnreadResults();
    double distance = 0;
    if (!results.isEmpty()) {
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var targets : result.getTargets()) {
          distance = PhotonUtils.calculateDistanceToTargetMeters(cameraLensHeightMeters, goalHeightMeters,
              Units.degreesToRadians(cameraMountAngleDEG), Units.degreesToRadians(targets.getPitch()));
        }
      }
      return distance;
    }
    return distance;
  }

  public void setTargetData() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        TargetData data = new TargetData(
            target.getYaw(),
            target.getPitch(),
            target.getSkew(),
            target.getArea(),
            this.getDistanceTarget());
        aprilTagData.put(target.fiducialId, data);
      }
    }
  }

  public void setRobotRotation(Rotation2d rotation) {
    this.robotRotation = rotation;
  }

  public Rotation2d getRobotRotation() {
    return this.robotRotation;
  }

  public Map<Integer, TargetData> getTargetData() {
    return this.aprilTagData;
  }

  public int getNumberOfTargetsDetected() {
    var result = camera.getLatestResult();
    return result.getTargets().size();
  }
}
