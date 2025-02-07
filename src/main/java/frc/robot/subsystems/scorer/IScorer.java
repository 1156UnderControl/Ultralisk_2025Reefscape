package frc.robot.subsystems.scorer;

import edu.wpi.first.math.geometry.Pose3d;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  void intakeFromHP();

  void prepareToPlaceCoralOnBranch(Pose3d branchPose);

  void removeAlgaeFromBranch(Pose3d reefFaceToRemove);

  void placeCoral();

  boolean isSecuredToPlaceCoral();

  boolean hasPlaced();

}
