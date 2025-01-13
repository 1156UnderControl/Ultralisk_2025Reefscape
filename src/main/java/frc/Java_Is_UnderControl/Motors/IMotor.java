package frc.Java_Is_UnderControl.Motors;

import edu.wpi.first.units.measure.Voltage;

public interface IMotor {
    final int maximumRetries = 5;

    void factoryDefault();

    void clearStickyFaults();

    void configureFeedForward(double Kg, double Ks, double Kv);

    void configurePIDF(double P, double I, double D, double F, double Izone);

    void configurePIDF(double P, double I, double D, double F);

    void configurePIDWrapping(double minInput, double maxInput);

    void setMotorBrake(boolean isBrakeMode);

    void setInverted(boolean inverted);

    void setInvertedEncoder(boolean inverted);

    void burnFlash();

    void set(double percentOutput);

    void set(Voltage percentOutput);

    void setPositionReference(double position);

    void setPositionReferenceMotionProfiling(double position, double velocity, double feedforward);

    double getVoltage();

    double getDutyCycleSetpoint();

    void setVoltage(double voltage);

    double getAppliedOutput();

    double getVelocity();

    double getPosition();

    void setPositionFactor(double factor);

    void setVelocityFactor(double factor);

    void setPosition(double position);

    void setVoltageCompensation(double nominalVoltage);

    void setCurrentLimit(int currentLimit);

    void setLoopRampRate(double rampRate);

    Object getMotor();

    void updateLogs();

}