package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightScore;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;

public interface IScorer {

  void periodic();

  boolean hasCoral();

  boolean hasAlgae();

  boolean readyToScoreProcessor();

  void intakeFromHP();

  void stopIntakeFromHP();

  void placeCoral();

  void placeAlgae();

  void prepareToScoreAlgae();

  void prepareToPlaceCoralOnBranch();

  void prepareToPlaceCoralOnBranch(Supplier<Double> distanceToTargetPoseProvider);

  void setAutoAlgaeLevel(TargetBranch targetBranch);

  void collectAlgae();

  void setAlgaeManualControl(boolean isManualControl);

  boolean isAlgaeManualControl();

  void stopEndEffector();

  void moveScorerToDefaultPosition();

  void setElevatorTestPosition(double testPosition);

  void setPivotTestPosition(double testPosition);

  void setElevatorDutyCycle(double dutyCycle);

  void setElevatorVoltage(double voltage);

  void setPivotDutyCycle(double dutyCycle);

  void setEndEffectorDutyCycle(double dutyCycle);

  boolean isAtCollectAlgaePosition();

  boolean isAtCollectCoralPosition();

  boolean isAtDefaultPosition();

  boolean isScorerAtPosition();

  boolean isSecuredToPlaceCoral();

  boolean isSecuredToScoreOnNet();

  boolean isElevatorInHighPosition();

  ReefLevel getTargetReefLevel();

  AlgaeHeightReef getTargetReefLevelAlgae();

  void setTargetBranchLevel(ReefLevel reefHeight);

  void setTargetAlgaeHeight(AlgaeHeightReef algaeHeight);

  void setTargetAlgaeScoreHeight(AlgaeHeightScore algaeHeightScore);

  void setCoastScorer();

  void setBrakeScorer();

  double getElevatorPosition();
}
