package frc.robot.constants;

public class EndEffectorConstants {
  public static final int ID_endEffectorMotor = 18;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM = 0;

  public class tunning_values_endeffector {
    public static final double VELOCITY_FALL_FOR_INTAKE_DETECTION = 100;

    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE = 1.0;
      public static final double DUTY_CYCLE_EXPELL = -1.0;
    }
  }
}
