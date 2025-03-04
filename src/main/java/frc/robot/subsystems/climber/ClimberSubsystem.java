package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.Servo;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.PivotConstants;

public class ClimberSubsystem implements IClimber {
  private static ClimberSubsystem instance;
  private IMotor cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, "Climber Intake Motor");
  private IMotor climberArmMotor = new TalonFXMotor(ClimberConstants.ID_climberArmMotor, GravityTypeValue.Arm_Cosine,
      "Climber Arm Motor");
  private Servo climberServoMotor = new Servo(0);

  private double previousVelocity = 0;
  private boolean isCageCollected = false;

  boolean cageIntakeAccelerated = false;

  double armGoal = ClimberConstants.tunning_values_arm.setpoints.STOW_ANGLE;

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
    climberArmMotor.setInverted(false);
    climberArmMotor.setPositionFactor(250);
    climberArmMotor.configurePIDF(
        ClimberConstants.tunning_values_arm.PID.P,
        ClimberConstants.tunning_values_arm.PID.I,
        ClimberConstants.tunning_values_arm.PID.D, 0);
    cageIntakeMotor.burnFlash();
    climberArmMotor.setMotorBrake(true);
    climberArmMotor.setPosition(0);
  }

  @Override
  public void climb() {
    this.armGoal = ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE;
  }

  public void moveArmToPosition(double position, double arbFF) {
    double goal = limitGoalArm(position);
    if (climberServoMotor.get() > 0.4 && goal > climberArmMotor.getPosition()) {
      climberArmMotor.set(0);
      return;
    }
    climberArmMotor.setPositionReference(goal);
  }

  private double limitGoalArm(double goal) {
    if (goal >= ClimberConstants.tunning_values_arm.setpoints.MAX_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MAX_ANGLE;
    } else if (goal <= ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE) {
      return PivotConstants.tunning_values_pivot.setpoints.MIN_ANGLE;
    } else {
      return goal;
    }
  }

  public void periodic() {
    moveArmToPosition(this.armGoal, 0);
    climberArmMotor.updateLogs();
  }

  @Override
  public boolean isAtIntakeCagePosition() {
    return Util.atSetpoint(this.climberArmMotor.getPosition(),
        ClimberConstants.tunning_values_arm.setpoints.INTAKE_CAGE_ANGLE, 0.01);
  }

  @Override
  public boolean isAtClimbPosition() {
    return Util.atSetpoint(this.climberArmMotor.getPosition(),
        ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE, 0.01);
  }

  @Override
  public void goToIntakeCagePosition() {
    this.armGoal = ClimberConstants.tunning_values_arm.setpoints.INTAKE_CAGE_ANGLE;
  }

  @Override
  public void intakeCage() {
    cageIntakeMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE);
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
  public void lockClimber() {
    climberServoMotor.set(1);
  }

  @Override
  public void unlockClimber() {
    climberServoMotor.set(0);
  }
}
