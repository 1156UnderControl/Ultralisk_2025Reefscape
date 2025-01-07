package frc.Java_Is_UnderControl.Vision.Cameras.Types;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Vision.TargetData;
import frc.Java_Is_UnderControl.Vision.Cameras.ICamera;

public class PhotonCameras implements ICamera{

    private static PhotonCameras instance;
    private PhotonCamera camera;
    private AprilTagFieldLayout fieldTags = AprilTagFieldLayout.loadField(null);
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult result;
    private Transform3d cameraPos;
    public TargetData targetData;
    private double cameraLensHeightMeters;
    private double goalHeightMeters;
    private double cameraMountAngleDEG;
    
    public static PhotonCameras getInstance(String cameraName, boolean isTag) {
        if (instance == null) {
          instance = new PhotonCameras(cameraName, isTag);
          return instance;
        } else {
          return instance;
        }
      }
    
    private PhotonCameras(String cameraName, boolean isTag){
        this.targetType(isTag);
        this.camera = new PhotonCamera(cameraName);
        this.photonPoseEstimator = new PhotonPoseEstimator(fieldTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cameraPos);
        this.result = new PhotonPipelineResult(null, null, null);
    }

    private void targetType(boolean isTag){
        if(isTag){
            
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
        photonPoseEstimator.setReferencePose(estimatedRobotPose);
        return photonPoseEstimator.update(result);
    }

    public void setTargetData(){
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                this.targetData = new TargetData(target.objDetectId, target.getYaw(), target.getPitch(), target.getSkew(), target.getArea(), this.getDistanceTarget());
            }
        }
    }    

    @Override
    public void camToRobot(double cameraLensHeightMeters, double goalHeightMeters, double cameraMountAngleDEG){}

    @Override
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

    @Override
    public boolean hasTarget(){
        var result = camera.getLatestResult();
        return result.hasTargets(); 
    }

    @Override
    public void setPipeline(int pipeline){
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public int getNumberOfTargetsDetected(){
        var result = camera.getLatestResult();
        return result.getTargets().size();
    }
}
