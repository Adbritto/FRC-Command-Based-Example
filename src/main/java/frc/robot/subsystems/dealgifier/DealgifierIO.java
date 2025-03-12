package frc.robot.subsystems.dealgifier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public interface DealgifierIO {
  @AutoLog
  class DealgifierIOInputs {
    public boolean motorConnected = true;
    public double velocityDegsPerSec = 0.0;
    public double positionDegs = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(DealgifierIO.DealgifierIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Angle position) {}

  default void reset() {}
}
