package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.Java_Is_UnderControl.Control.PIDConfig;

public class SwerveConstants {
  public static final PIDConfig MOVE_TO_POSE_X_PID = new PIDConfig(0.5, 0, 0);
  public static final PIDConfig MOVE_TO_POSE_Y_PID = new PIDConfig(0.5, 0, 0);

  public static final Constraints MOVE_TO_POSE_X_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_Y_CONSTRAINTS = new Constraints(4, 3);
}
