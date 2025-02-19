package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem implements IClimber {
  private static ClimberSubsystem instance;
  private IMotor cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, "CAGE_INTAKE");
  private IMotor climberArmMotor = new TalonFXMotor(ClimberConstants.ID_climberArmMotor,
      GravityTypeValue.Arm_Cosine,
      "CLIMBER_ARM");

  private double previousVelocity = 0;
  private boolean isCageCollected = false;

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
    climberArmMotor.setMotorBrake(false);
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
  }

  @Override
  public void climb() {
  }

  private void activateRollers(double speed) {
    cageIntakeMotor.set(speed);
  }

  private void moveArmsToPosition(double position) {
    climberArmMotor.set(position);
  }

  public void periodic() {
  }

  @Override
  public void isAtSetPoint() {
  }

  @Override
  public void raiseClimber() {
    isCageCollected = false;
  }

  @Override
  public void intakeCage() {
    runCageIntakeDetection();
    cageIntakeMotor.set(ClimberConstants.tunning_values_intake.setpoints.DUTY_CYCLE_INTAKE);
  }

  public void runCageIntakeDetection() {
    if (cageIntakeMotor.getVelocity() < previousVelocity
        - ClimberConstants.tunning_values_intake.VELOCITY_FALL_FOR_CAGE_INTAKE_DETECTION) {
      isCageCollected = true;
    }
    previousVelocity = cageIntakeMotor.getVelocity();
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
    if (climberArmMotor.getPosition() <= ClimberConstants.tunning_values_arm.setpoints.MIN_ANGLE
        && dutyCycle > 0) {
      climberArmMotor.set(dutyCycle);
    } else if (climberArmMotor.getPosition() >= ClimberConstants.tunning_values_arm.setpoints.MAX_ANGLE
        && dutyCycle < 0) {
      climberArmMotor.set(dutyCycle);
    } else {
      climberArmMotor.set(0);
    }
  }
}
