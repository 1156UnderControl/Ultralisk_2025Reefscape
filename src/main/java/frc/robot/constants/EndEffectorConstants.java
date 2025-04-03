package frc.robot.constants;

public class EndEffectorConstants {
  public static final int ID_endEffectorMotor = 4;
  public static final int Port_coralInfraRed = 9;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM = 1;

  public class tunning_values_endeffector {
    public static final double VELOCITY_FALL_FOR_INTAKE_DETECTION = 700;

    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE = 1.0;
      public static final double DUTY_CYCLE_EXPELL = -0.8;
      public static final double DUTY_CYCLE_EXPELL_L1 = -0.275;
      public static final double DUTY_CYCLE_HOLDING_ALGAE = 0.3;
    }
  }
}
