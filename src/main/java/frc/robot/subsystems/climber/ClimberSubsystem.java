package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.NoMotor;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.PivotConstants;

public class ClimberSubsystem implements IClimber {
  private static ClimberSubsystem instance;
  private IMotor cageIntakeMotor = new NoMotor();
  private IMotor climberArmMotor = new NoMotor();

  private double previousVelocity = 0;
  private boolean isCageCollected = false;

  boolean cageIntakeAccelerated = false;

  @Logged(name = "State", importance = Importance.CRITICAL)
  private String state = "START";

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  private ClimberSubsystem() {
    configureClimberMotor();
  }

  private void configureClimberMotor() {
    climberArmMotor.setPosition(0);
    climberArmMotor.configureMotionProfiling(
        ClimberConstants.tunning_values_arm.PID.P,
        ClimberConstants.tunning_values_arm.PID.I,
        ClimberConstants.tunning_values_arm.PID.D,
        ClimberConstants.tunning_values_arm.KS,
        ClimberConstants.tunning_values_arm.KV,
        ClimberConstants.tunning_values_arm.KA,
        ClimberConstants.tunning_values_arm.MAX_VELOCITY,
        ClimberConstants.tunning_values_arm.MAX_ACCELERATION,
        ClimberConstants.tunning_values_arm.JERK);
    cageIntakeMotor.burnFlash();
    climberArmMotor.setMotorBrake(true);
  }

  @Override
  public void climb() {

  }

  public void moveArmsToPosition(double position, double arbFF) {
    double goal = limitGoalArm(position);
    climberArmMotor.setPositionReferenceMotionProfiling(goal, arbFF);
  }

  private double limitGoalArm(double goal) {
    if (goal >= PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE;
    } else if (goal <= PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  public void periodic() {
    climberArmMotor.updateLogs();
  }

  @Override
  public boolean isAtSetPoint() {
    // return Util.atSetpoint(this.climberArmMotor.getPosition(), ,
    // previousVelocity);
    return false;
  }

  @Override
  public void raiseClimber() {
    isCageCollected = false;
  }

  @Override
  public void intakeCage() {
    runCageIntakeDetection();
    if (!isCageCollected) {
      cageIntakeMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
    } else {
      cageIntakeMotor.set(0);
    }
    state = "INTAKING_CAGE";
  }

  public void runCageIntakeDetection() {
    if (this.cageIntakeMotor.getVelocity() >= 3000) {
      this.cageIntakeAccelerated = true;
    }
    if (cageIntakeMotor
        .getVelocity() < EndEffectorConstants.tunning_values_endeffector.VELOCITY_FALL_FOR_INTAKE_DETECTION
        && cageIntakeAccelerated) {
      isCageCollected = true;
      cageIntakeAccelerated = false;
    }
  }

  @Override
  public boolean isCageCollected() {
    return isCageCollected;
  }

  @Override
  public void stopIntakingCage() {
    cageIntakeMotor.set(0);
  }

  @Override
  public void setCoastClimber() {
    cageIntakeMotor.setMotorBrake(false);
    climberArmMotor.setMotorBrake(false);
  }

  @Override
  public void setBrakeClimber() {
    cageIntakeMotor.setMotorBrake(true);
    climberArmMotor.setMotorBrake(true);
  }

  @Override
  public void setArmDutyCycle(double dutyCycle) {
    System.out.println(dutyCycle);
    if (climberArmMotor.getPosition() >= ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE
        && dutyCycle < 0) {
      climberArmMotor.set(dutyCycle);
      System.out.println("Indo para Baixo");
    } else if (climberArmMotor.getPosition() <= ClimberConstants.tunning_values_arm.setpoints.MAX_ANGLE
        && dutyCycle > 0) {
      climberArmMotor.set(dutyCycle);
      System.out.println("Indo para Cima");
    } else {
      climberArmMotor.set(0);
    }
  }
}
