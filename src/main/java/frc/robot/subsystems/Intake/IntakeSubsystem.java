package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private IMotor intakeMotor = new SparkMAXMotor(IntakeConstants.ID_intakeMotor, "INTAKE");;

  public IntakeSubsystem() {
    intakeMotor.setMotorBrake(false);
    intakeMotor.burnFlash();
  }

  @Override
  public void intake() {
  }

  private void activateCoralIntake() {
    intakeMotor.setMotorBrake(false);
  }

  public void controlIntake(String itemType) {
    if (itemType.equals("coral")) {
      activateCoralIntake();
    } else {
      intakeMotor.setMotorBrake(false);
    }
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

  @Override
  public boolean isAtSetPoint() {
    return false;
  }
}
