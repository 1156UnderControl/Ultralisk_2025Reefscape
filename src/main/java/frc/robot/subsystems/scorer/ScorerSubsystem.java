package frc.robot.subsystems.scorer;

import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem implements IScorer {

  private static ScorerSubsystem instance;
  private IMotor elevatorMotorLeader = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, "ELEVATOR_MASTER");
  private IMotor elevatorMotorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor,
      "ELEVATOR_FOLLOWER");

  private final IMotor pivotMotor = new SparkMAXMotor(PivotConstants.ID_pivotMotor, true, "PIVOT");
  private final IMotor endEffectorMotor = new SparkMAXMotor(EndEffectorConstants.ID_endEffectorMotor, "END_EFFECTOR");

  private boolean hasCoral = false;
  private double previousVelocity;

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
        ElevatorConstants.tunning_values_elevator.PID.F,
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
        PivotConstants.tunning_values_pivot.PID.F,
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

  }

  public boolean isRobotAbleToScore() {
    return false;
  }

  public void runCoralIntakeDetection() {
    if (endEffectorMotor.getVelocity() < previousVelocity
        - EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION) {
      hasCoral = true;
    } else {
      hasCoral = false;
    }
    previousVelocity = endEffectorMotor.getVelocity();
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void intake() {

  }

  @Override
  public void intakeFromHP() {
  }

  @Override
  public void prepareToPlaceCoral() {
  }

  @Override
  public void place() {
  }

  @Override
  public void expell() {
  }

  @Override
  public boolean isSecuredToPlace() {
    return false;
  }

  @Override
  public boolean hasPlaced() {
    return false;
  }

  @Override
  public void stopIntaking() {
  }

  @Override
  public boolean isAtSetPoint() {
    return false;
  }
}
