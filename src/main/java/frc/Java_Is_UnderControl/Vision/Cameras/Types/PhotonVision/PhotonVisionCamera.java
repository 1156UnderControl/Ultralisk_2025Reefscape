package frc.Java_Is_UnderControl.Vision.Cameras.Types.PhotonVision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.AprilTagData;
import frc.Java_Is_UnderControl.Vision.Cameras.Data.ObjectData;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraObject;
import frc.Java_Is_UnderControl.Vision.Cameras.Types.Interfaces.ICameraOdometry;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetection;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectDetectionCamera;
import frc.Java_Is_UnderControl.Vision.Object_Detection.ObjectPoseEstimationRobotOriented;

public class PhotonVisionCamera implements ICameraOdometry, ICameraObject{

    private String cameraName;
    private String objectName;
    private PhotonCamera camera;
    private Rotation2d robotRotation;
    private Transform3d camToRobot;

    private AprilTagData aprilData;

    private ObjectData objectData;
    private ObjectDetection objectDetection;
    private ObjectDetectionCamera objectDetectionCamera;
    private ObjectPoseEstimationRobotOriented objectPose;
    private AprilTagFieldLayout aprilTagFieldLayout;
    
    public PhotonVisionCamera(String cameraName, Translation3d translation, Rotation3d rotation){
        this.camera = new PhotonCamera(cameraName);
        this.aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        this.setCamToRobot(translation, rotation);
    }

    private void setCamToRobot(Translation3d translation, Rotation3d rotation){
        this.camToRobot = new Transform3d(translation, rotation);
    }

    public String getCameraName(){
        return this.cameraName;
    }

    public PhotonPipelineResult getLatestResult(){
        return camera.getLatestResult();
    }

    private double getDistanceTarget(PhotonTrackedTarget photonTrackedTarget){
        var result = camera.getLatestResult();
        double distance = 0;
        if (result.hasTargets()) {
            PhotonTrackedTarget target = photonTrackedTarget;
            distance = PhotonUtils.calculateDistanceToTargetMeters(this.camToRobot.getY(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), 
            Units.degreesToRadians(this.camToRobot.getX()), Units.degreesToRadians(target.getPitch()));
            return distance;
        }
        return distance;
    }

    private void setAprilTagData(){
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                this.aprilData = new AprilTagData(cameraName, target.getFiducialId(), target.getYaw(), target.getPitch(), target.getSkew(), target.getArea(), this.getDistanceTarget(target), this.camToRobot, this.getNumberOfTargetsDetected());
            }
        }
    }

    @Override
    public Optional<Pose3d> getRobotPose(){
        PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobot);
        var robotPose = poseEstimator.update(getLatestResult());
        if(robotPose.isPresent()){
            return Optional.of(robotPose.get().estimatedPose);
        }else{
            return Optional.empty();
        }
    }

    @Override
    public AprilTagData getAprilTagData(){
        this.setAprilTagData();
        return this.aprilData;
    }

    @Override
    public void updateLogsAprilTag(){
        this.setAprilTagData();
        this.aprilData.updateLogs();
    }

    private void setObjectName(String objectName){
        this.objectName = objectName;
    }

    private void setObjectPose(double maxTy, double maxTx){
        this.setObjectData();
        this.objectDetectionCamera = new ObjectDetectionCamera(this.camToRobot, maxTy, maxTx);
        this.objectPose = new ObjectPoseEstimationRobotOriented(this.objectDetection, this.objectDetectionCamera);
    }

    private void setObjectData(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            for (PhotonTrackedTarget target : result.getTargets()){
                this.objectDetection = new ObjectDetection(target.getYaw(), target.getPitch(), (this.objectName != null ? this.objectName : "object"));
                this.objectData = new ObjectData(this.cameraName, objectDetection, this.getDistanceTarget(target), target.getBestCameraToTarget(), camToRobot, null);
            }
        }
    }

    @Override
    public Pose2d getObjectPose(String objectName, double maxTy, double maxTx, Pose2d robotPose){
        this.setObjectName(objectName);
        this.setObjectData();
        this.setObjectPose(maxTy, maxTx);
        Pose2d objectPose2d = this.objectPose.getObjectPoseFromRobotCenter();
        return objectPose.getObjectPoseInField(robotPose, objectPose2d);
    }

    @Override
    public ObjectData getObjectData(){
        this.setObjectData();
        return this.objectData;
    }

    @Override
    public void updateLogsObject(){
        this.setObjectData();
        this.aprilData.updateLogs();
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
