package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @AutoLogOutput
  public Command runElevatorToPos(Distance target) {
    return startEnd(() -> io.runPosition(target), () -> io.stop());
  }

  @AutoLogOutput
  public Command runElevatorVolts(double volts) {
    return startEnd(() -> io.runVolts(volts), () -> io.stop());
  }

  @AutoLogOutput
  public Command runElevatorOpenLoop(double output) {
    return startEnd(() -> io.runOpenLoop(output), () -> io.stop());
  }
}
