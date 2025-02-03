package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private static IntakeSubsystem instance;
  private static final int IR = 0;
  private IMotor intakeMotor = new SparkMAXMotor(IntakeConstants.ID_intakeMotor, "INTAKE");;
  private DigitalInput infraredSensor;
  private boolean isCoralDetected;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private IntakeSubsystem() {
    intakeMotor.setMotorBrake(false);
    intakeMotor.burnFlash();
    this.infraredSensor = new DigitalInput(IR);
  }

  @Override
  public void intake() {
    runCoralDetection();
    this.intakeMotor.set(0);
  }

  public boolean isCoralDetected() {
    return this.isCoralDetected;

  }

  private void runCoralDetection() {
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
