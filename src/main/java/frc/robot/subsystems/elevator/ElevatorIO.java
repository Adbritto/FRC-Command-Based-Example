package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean motorConnected = true;
    public double velocityInchPerSec = 0.0;
    public double positionInch = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Distance positionInches) {}

  default void reset() {}
}
