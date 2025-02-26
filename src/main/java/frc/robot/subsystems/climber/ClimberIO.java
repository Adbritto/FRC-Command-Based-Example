package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public boolean motorConnected = true;
    public double velocityInchPerSec = 0.0;
    public double positionInch = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Angle positionDegrees) {}
}
