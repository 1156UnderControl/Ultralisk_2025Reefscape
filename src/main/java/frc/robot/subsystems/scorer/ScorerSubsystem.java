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

  public static ScorerSubsystem getInstance() {
    if (instance == null) {
      instance = new ScorerSubsystem();
    }
    return instance;
  }

  private ScorerSubsystem() {
    setConfigsElevator();
    setConfigsPivot();
    setConfigsEndEffector();
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
    elevatorMotorLeader.setPositionReferenceMotionProfiling(limitGoalElevator(goalElevator),
        ElevatorConstants.tunning_values_elevator.PID.arbFF);
    pivotMotor.setPositionReferenceMotionProfiling(limitGoalPivot(goalElevator),
        PivotConstants.tunning_values_pivot.PID.arbFF);

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
  }

  @Override
  public void prepareToPlaceCoralOnBranch(Pose3d branchPose) {
    ReefHeight selectedLevel = getReefHeightFromPose(branchPose);
    if (selectedLevel != null) {
      assignSetpointsForLevel(selectedLevel);
      state = "Setting coral to Reef Level " + selectedLevel.name();
    } else {
      state = "No matching ReefHeight for Pose z=" + branchPose.getZ();
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
  }

  @Override
  public void homeElevator() {
    if (!elevatorHasHomed) {
      elevatorMotorLeader.set(0.1);
    } else if (Math.abs(elevatorMotorLeader.getVelocity()) < 0.1) {
      elevatorMotorLeader.set(0);
      elevatorMotorLeader.setPosition(0);
      elevatorHasHomed = true;
    }
  }

  @Override
  public void placeCoral() {
    hasCoral = false;
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

}
