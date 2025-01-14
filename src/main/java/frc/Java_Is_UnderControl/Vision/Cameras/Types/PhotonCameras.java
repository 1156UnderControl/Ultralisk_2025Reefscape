package frc.Java_Is_UnderControl.Vision.Cameras.Types;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Vision.Cameras.TargetData;

public class PhotonCameras{

    private String cameraName;
    private static PhotonCameras instance;
    private PhotonCamera camera;
    private double cameraLensHeightMeters;
    private double goalHeightMeters;
    private double cameraMountAngleDEG;
    private Rotation2d robotRotation;
    private Transform3d camToRobot;
    private TargetData targetData;
    private boolean isTag;
    
    public PhotonCameras getInstance(String cameraName, boolean isTag) {
        this.cameraName = cameraName;
        if (instance == null) {
          instance = new PhotonCameras(cameraName, isTag);
          return instance;
        } else {
          return instance;
        }
      }
    
    private PhotonCameras(String cameraName, boolean isTag){
        this.camera = new PhotonCamera(cameraName);
    }

    public void setCamToRobot(Translation3d translation, Rotation3d rotation){
        this.camToRobot = new Transform3d(translation, rotation);
    }

    private Transform3d getCamToRobot(){
        return this.camToRobot;
    }

    public double getDistanceTarget(){
        var result = camera.getLatestResult();
        double distance = 0;
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            distance = PhotonUtils.calculateDistanceToTargetMeters(cameraLensHeightMeters, goalHeightMeters,
            Units.degreesToRadians(cameraMountAngleDEG), Units.degreesToRadians(target.getPitch()));
            return distance;
        }
        return distance;
    }

    public void setTargetData(){
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if(isTag){
                    this.targetData = new TargetData(
                        cameraName,
                        target.getFiducialId(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getSkew(),
                        target.getArea(),
                        this.getDistanceTarget(),
                        target.getBestCameraToTarget(),
                        this.getCamToRobot());
                } else {
                    this.targetData = new TargetData(cameraName, target.getYaw(),
                    target.getPitch(),
                    target.getSkew(),
                    target.getArea(),
                    this.getDistanceTarget(),
                    this.getCamToRobot());
                }
            }
        }
    }

    public TargetData getTargetData(){
        return this.targetData;
    }

    public void setRobotRotation(Rotation2d rotation){
        this.robotRotation = rotation;
    }

    public Rotation2d getRobotRotation(){
        return this.robotRotation;
    }

    public int getNumberOfTargetsDetected(){
        var result = camera.getLatestResult();
        return result.getTargets().size();
    }
}
