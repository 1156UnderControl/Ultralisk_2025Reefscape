package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.Java_Is_UnderControl.Control.PIDConfig;

public class MoveToPosePIDConfig {

  public PIDConfig pidX;

  public Constraints constraintsX;

  public PIDConfig pidY;

  public Constraints constraintsY;

  public MoveToPosePIDConfig(PIDConfig pidX, Constraints constraintsX, PIDConfig pidY, Constraints constraintsY) {
    this.pidX = pidX;
    this.constraintsX = constraintsX;
    this.pidY = pidY;
    this.constraintsY = constraintsY;
  }

  public MoveToPosePIDConfig(PIDConfig pid, Constraints constraints) {
    this(pid, constraints, pid, constraints);
  }

  public ProfiledPIDController getProfiledPIDX() {
    return new ProfiledPIDController(pidX.kP, pidX.kI, pidX.kD, constraintsX);
  }

  public ProfiledPIDController getProfiledPIDY() {
    return new ProfiledPIDController(pidY.kP, pidY.kI, pidY.kD, constraintsY);
  }

}
