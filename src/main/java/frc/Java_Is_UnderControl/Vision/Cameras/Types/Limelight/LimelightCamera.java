package frc.Java_Is_UnderControl.Vision.Cameras.Types.Limelight;

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

import java.util.Optional;

public class LimelightCamera implements ICameraOdometry, ICameraObject {

    private String cameraName;
    private String objectName;
    private Rotation2d robotRotation;
    private Transform3d camToRobot;

    private AprilTagData aprilData;
    private ObjectData objectData;
    private ObjectDetection objectDetection;
    private ObjectDetectionCamera objectDetectionCamera;
    private ObjectPoseEstimationRobotOriented objectPose;

    public LimelightCamera(String cameraName, Translation3d translation, Rotation3d rotation) {
        this.cameraName = cameraName;
        this.setCamToRobot(translation, rotation);
    }

    private void setCamToRobot(Translation3d translation, Rotation3d rotation) {
        this.camToRobot = new Transform3d(translation, rotation);
    }

    public String getCameraName() {
        return this.cameraName;
    }

    private double getDistanceTarget(int targetID) {
      Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName);
      if (targetPose != null) {
          double x = targetPose.getTranslation().getX();
          double y = targetPose.getTranslation().getY();
          return Math.sqrt(x * x + y * y);
      }
      return 0;
  }

  private void setAprilTagData() {
    var results = LimelightHelpers.getLatestResults(cameraName);
    
    if (results.targets_Fiducials.length > 0) {
        // Atualiza os dados da única instância com os valores da primeira AprilTag detectada
        for (var target : results.targets_Fiducials) { // Itera sobre todas as tags detectadas
            this.aprilData = new AprilTagData(
                cameraName,
                (int) target.fiducialID,
                target.tx,
                target.ty,
                target.ta,
                this.getDistanceTarget((int) target.fiducialID),
                this.camToRobot,
                results.targets_Fiducials.length
            );
        }
    }
}

    @Override
    public Optional<Pose3d> getRobotPose() {
        double[] poseArray = LimelightHelpers.getBotPose(cameraName);
        if (poseArray.length == 6) {
            Pose3d pose = new Pose3d(
                new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
                new Rotation3d(
                    Units.degreesToRadians(poseArray[3]),
                    Units.degreesToRadians(poseArray[4]),
                    Units.degreesToRadians(poseArray[5])
                )
            );
            return Optional.of(pose);
        }
        return Optional.empty();
    }

    @Override
    public AprilTagData getAprilTagData() {
        this.setAprilTagData();
        return this.aprilData;
    }

    @Override
    public void updateLogsAprilTag() {
        this.setAprilTagData();
        if (this.aprilData != null) {
            this.aprilData.updateLogs();
        }
    }

    private void setObjectData() {
        var results = LimelightHelpers.getLatestResults(cameraName);
        if (results.targets_Retro.length > 0) {
            var target = results.targets_Retro[0]; // Using the first retro-reflective target
            this.objectDetection = new ObjectDetection(
                target.tx,
                target.ty,
                (this.objectName != null ? this.objectName : "object")
            );
            this.objectData = new ObjectData(
                this.cameraName,
                objectDetection,
                this.getDistanceTarget((int) target.tx),
                this.camToRobot,
                camToRobot
            );
        }
    }

    private void setObjectPose(double maxTy, double maxTx) {
        this.setObjectData();
        this.objectDetectionCamera = new ObjectDetectionCamera(this.camToRobot, maxTy, maxTx);
        this.objectPose = new ObjectPoseEstimationRobotOriented(this.objectDetection, this.objectDetectionCamera);
    }

    @Override
    public Pose2d getObjectPose(String objectName, double maxTy, double maxTx, Pose2d robotPose) {
        this.objectName = objectName;
        this.setObjectPose(maxTy, maxTx);
        Pose2d objectPose2d = this.objectPose.getObjectPoseFromRobotCenter();
        return this.objectPose.getObjectPoseInField(robotPose, objectPose2d);
    }

    @Override
    public ObjectData getObjectData() {
        this.setObjectData();
        return this.objectData;
    }

    @Override
    public void updateLogsObject() {
        this.setObjectData();
        if (this.objectData != null) {
            this.objectData.updateLogs();
        }
    }

    public void setRobotRotation(Rotation2d rotation) {
        this.robotRotation = rotation;
    }

    public Rotation2d getRobotRotation() {
        return this.robotRotation;
    }

    public int getNumberOfTargetsDetected() {
        return LimelightHelpers.getTargetCount(cameraName);
    }
}