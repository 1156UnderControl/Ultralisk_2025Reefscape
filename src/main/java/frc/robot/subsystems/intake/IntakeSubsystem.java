package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import frc.Java_Is_UnderControl.Motors.IMotor;
import frc.Java_Is_UnderControl.Motors.NoMotor;
import frc.Java_Is_UnderControl.Sensors.DigitalInput;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem implements IIntake {
  private static IntakeSubsystem instance;

  private IMotor intakeMotor = new NoMotor();

  private DigitalInput beamBreakSensor;

  @Logged(name = "isCoralDetected", importance = Importance.CRITICAL)
  private boolean isCoralDetected;

  @Logged(name = "State", importance = Importance.INFO)
  private String state = "START";

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private IntakeSubsystem() {
    intakeMotor.setMotorBrake(false);
    intakeMotor.setInverted(true);
    intakeMotor.burnFlash();
    this.beamBreakSensor = new InfraRed(IntakeConstants.port_IR, false);
  }

  @Override
  public void intake() {
    runCoralDetection();
    this.intakeMotor.set(IntakeConstants.tunning_values_intake.setpoints.SPEED_INTAKE);
    this.state = "INTAKING";
  }

  public boolean isCoralDetected() {
    return this.isCoralDetected;
  }

  private void runCoralDetection() {
    if (beamBreakSensor.getBoolean()) {
      this.isCoralDetected = true;
    } else {
      this.isCoralDetected = false;
    }
  }

  @Override
  public void stopIntake() {
    this.intakeMotor.set(0);
    this.state = "STOPPED";
  }

  @Override
  public void expell() {
    this.intakeMotor.set(IntakeConstants.tunning_values_intake.setpoints.SPEED_EXPELL);
    this.state = "EXPELLING";
  }

  @Override
  public void periodic() {
    this.intakeMotor.updateLogs();
  }
}
