package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private static final int IR = 0;
  private IMotor intakeMotor = new SparkMAXMotor(IntakeConstants.ID_intakeMotor, "INTAKE");;
  private DigitalInput infraredSensor;
  private boolean isCoralDetected;

  public IntakeSubsystem() {
    intakeMotor.setMotorBrake(false);
    intakeMotor.burnFlash();
    this.infraredSensor = new DigitalInput(IR);
  }

  @Override
  public void intake() {
    if (infraredSensor.get()) {
      this.isCoralDetected = true;
    }
    this.intakeMotor.set(0);
  }

  public boolean isCoralDetected() {
    return !infraredSensor.get();

  }

  private void detectthecollect() {
    if (infraredSensor.get()) {
      this.isCoralDetected = true;
    }
  }

  private void stopIntake() {
    this.intakeMotor.set(0);
  }

  @Override
  public void expell() {
  }

  @Override
  public void goToIntakePosition() {
  }

  @Override
  public void goToSecuredPosition() {
  }
}
