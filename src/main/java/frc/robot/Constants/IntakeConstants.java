package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final Pose2d intakeRollerPositionRelativeToRobotCenter = new Pose2d(0.46, 0, new Rotation2d());
  public static final int ID_intakeMotor = 0;
  public static final int port_IR = 0;

  public class tunning_values_intake {
    public class setpoints {
      public static final double SPEED_INTAKE = 0;
      public static final double SPEED_EXPELL = 0;
    }
  }
}
