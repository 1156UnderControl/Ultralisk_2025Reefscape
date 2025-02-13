package frc.robot.subsystems.scorer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(PivotConstants.ID_pivotMotor, true, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(EndEffectorConstants.ID_endEffectorMotor, "END_EFFECTOR");

  private boolean hasCoral = false;
  private double previousVelocity = 0;
  private boolean elevatorHasHomed = false;
  private double goalElevator = 0;
  private double goalPivot = 0;

  @Logged(name = "State", importance = Importance.INFO)
  private String state = "START";

  @Logged(name = "Target Branch Height", importance = Importance.INFO)
  private String branchHeightTarget = "NONE";

  @Logged(name = "Target Reef Face To Remove Algae", importance = Importance.INFO)
  private String reefFaceTarget = "NONE";

  public static ScorerSubsystem getInstance() {
    if (instance == null) {
      instance = new ScorerSubsystem();
    }
    return instance;
  }

  private ScorerSubsystem() {
    setConfigsElevator();
    // setConfigsPivot();
    // setConfigsEndEffector();
  }

  private void setConfigsElevator() {
    elevatorMotorLeader.setMotorBrake(true);
    elevatorMotorFollower.setMotorBrake(true);
    elevatorMotorLeader.setLoopRampRate(0.5);
    elevatorMotorFollower.setLoopRampRate(0.5);
    elevatorMotorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
    elevatorMotorLeader.setPositionFactor(ElevatorConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
    elevatorMotorLeader.configureMotionProfiling(
        ElevatorConstants.tunning_values_elevator.PID.P,
        ElevatorConstants.tunning_values_elevator.PID.I,
        ElevatorConstants.tunning_values_elevator.PID.D,
        ElevatorConstants.tunning_values_elevator.PID.arbFF,
        ElevatorConstants.tunning_values_elevator.MAX_VELOCITY,
        ElevatorConstants.tunning_values_elevator.MAX_ACCELERATION,
        ElevatorConstants.tunning_values_elevator.POSITION_ERROR_ALLOWED);
    elevatorMotorFollower.burnFlash();
  }

  private void setConfigsPivot() {
    pivotMotor.setMotorBrake(true);
    pivotMotor.setLoopRampRate(0.2);
    pivotMotor.setPositionFactor(PivotConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_DEGREES);
    pivotMotor.configureMotionProfiling(
        PivotConstants.tunning_values_pivot.PID.P,
        PivotConstants.tunning_values_pivot.PID.I,
        PivotConstants.tunning_values_pivot.PID.D,
        PivotConstants.tunning_values_pivot.PID.arbFF,
        PivotConstants.tunning_values_pivot.MAX_VELOCITY,
        PivotConstants.tunning_values_pivot.MAX_ACCELERATION,
        PivotConstants.tunning_values_pivot.POSITION_ERROR_ALLOWED);
    pivotMotor.burnFlash();
  }

  private void setConfigsEndEffector() {
    endEffectorMotor.setMotorBrake(true);
    endEffectorMotor.setLoopRampRate(0.1);
    endEffectorMotor.setVelocityFactor(EndEffectorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM);
    endEffectorMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // elevatorMotorLeader.setPositionReferenceMotionProfiling(limitGoalElevator(goalElevator),
    // ElevatorConstants.tunning_values_elevator.PID.arbFF);
    // pivotMotor.setPositionReferenceMotionProfiling(limitGoalPivot(goalElevator),
    // PivotConstants.tunning_values_pivot.PID.arbFF);
    elevatorMotorLeader.updateLogs();
    elevatorMotorFollower.updateLogs();
    // pivotMotor.updateLogs();
    SmartDashboard.putString("state", state);
  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  public void runCoralIntakeDetection() {
    if (endEffectorMotor.getVelocity() < previousVelocity
        - EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION) {
      hasCoral = true;
    }
    previousVelocity = endEffectorMotor.getVelocity();
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void intakeFromHP() {

    state = "INTAKING_FROM_HP";
  }

  @Override
  public void prepareToPlaceCoralOnBranch(Pose3d branchPose) {
    ReefHeight selectedLevel = getReefHeightFromPose(branchPose);
    if (selectedLevel != null) {
      assignSetpointsForLevel(selectedLevel);
      state = "PREPARE_TO_PLACE_CORAL";
      branchHeightTarget = selectedLevel.name();
    } else {

    }
  }

  private ReefHeight getReefHeightFromPose(Pose3d pose) {
    double targetHeight = pose.getZ();
    final double tolerance = 0.001;
    for (ReefHeight level : ReefHeight.values()) {
      if (Math.abs(targetHeight - level.height) < tolerance) {
        return level;
      }
    }
    return null;
  }

  private void assignSetpointsForLevel(ReefHeight level) {
    switch (level) {
      case L1:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L1_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L1_ANGLE;
        break;
      case L2:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L2_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L2_ANGLE;
        break;
      case L3:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L3_ANGLE;
        break;
      case L4:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.L4_HEIGHT;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.L4_ANGLE;
        break;
      default:
        break;
    }
  }

  @Override
  public void removeAlgaeFromBranch(Pose3d reefFaceToRemove) {
    int faceIndex = getReefFaceIndexFromPose(reefFaceToRemove);
    if (faceIndex >= 0) {
      assignAlgaeRemovalSetpointsForFace(faceIndex);
      state = "REMOVING_ALGAE_FROM_REEF";
      reefFaceTarget = "/Starting facing the driver station in clockwise order: " + Integer.toString(faceIndex);
    } else {
      reefFaceTarget = "NO_MATCHING_REEF_FACE" + reefFaceToRemove;
    }
  }

  private int getReefFaceIndexFromPose(Pose3d pose) {
    double minDistance = Double.MAX_VALUE;
    int bestIndex = -1;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      var facePose = FieldConstants.Reef.centerFaces[i];
      double dx = pose.getX() - facePose.getTranslation().getX();
      double dy = pose.getY() - facePose.getTranslation().getY();
      double distance = Math.hypot(dx, dy);
      if (distance < minDistance) {
        minDistance = distance;
        bestIndex = i;
      }
    }
    return bestIndex;
  }

  private void assignAlgaeRemovalSetpointsForFace(int faceIndex) {
    switch (faceIndex) {
      case 0:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE0_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE0_ALGAE_REMOVAL;
        break;
      case 1:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE1_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE1_ALGAE_REMOVAL;
        break;
      case 2:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE2_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE2_ALGAE_REMOVAL;
        break;
      case 3:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE3_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE3_ALGAE_REMOVAL;
        break;
      case 4:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE4_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE4_ALGAE_REMOVAL;
        break;
      case 5:
        goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.FACE5_ALGAE_REMOVAL;
        goalPivot = PivotConstants.tunning_values_pivot.setpoints.FACE5_ALGAE_REMOVAL;
        break;
      default:
        break;
    }
  }

  @Override
  public void moveScorerToDefaultPosition() {
    goalElevator = ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    goalPivot = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
    state = "DEFAULT";
  }

  @Override
  public void homeElevator() {
    if (!elevatorHasHomed) {
      elevatorMotorLeader.set(0.1);
      this.state = "HOMING_ELEVATOR";
    } else if (Math.abs(elevatorMotorLeader.getVelocity()) < 0.1) {
      this.state = "ELEVATOR_HOMED";
      elevatorMotorLeader.set(0);
      elevatorMotorLeader.setPosition(0);
      elevatorHasHomed = true;
    }
  }

  @Override
  public void placeCoral() {
    hasCoral = false;
    elevatorHasHomed = false;
    this.state = "PLACING_CORAL";
  }

  @Override
  public boolean isSecuredToPlaceCoral() {
    return false;
  }

  @Override
  public boolean hasPlaced() {
    return false;
  }

  private double limitGoalElevator(double goal) {
    if (goal > ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT;
    } else if (goal < ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT) {
      return ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
    } else {
      return goal;
    }
  }

  private double limitGoalPivot(double goal) {
    if (goal > PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE;
    } else if (goal < PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  @Override
  public void setElevatorTestPosition(double testPosition) {
    goalElevator = testPosition;
    this.state = "ELEVATOR_TEST_POSITION" + testPosition;
  }

  @Override
  public void setPivotTestPosition(double testPosition) {
    goalPivot = testPosition;
    this.state = "PIVOT_TEST_POSITION" + testPosition;
  }

  @Override
  public void setElevatorDutyCycle(double dutyCycle) {
    if (elevatorMotorLeader.getPosition() > ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT
        && elevatorMotorLeader.getPosition() < ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT) {
      elevatorMotorLeader.set(dutyCycle);
    } else {
      elevatorMotorLeader.set(0);
    }
  }
}
