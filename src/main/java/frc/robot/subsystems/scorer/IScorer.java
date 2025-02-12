package frc.robot.subsystems.scorer;

import edu.wpi.first.math.geometry.Pose3d;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  void intakeFromHP();

  void prepareToPlaceCoralOnBranch(Pose3d branchPose);

  void removeAlgaeFromBranch(Pose3d reefFaceToRemove);

  void placeCoral();

  void homeElevator();

  void setElevatorTestPosition(double testPosition);

  void setPivotTestPosition(double testPosition);

  boolean isSecuredToPlaceCoral();

  boolean hasPlaced();

}
