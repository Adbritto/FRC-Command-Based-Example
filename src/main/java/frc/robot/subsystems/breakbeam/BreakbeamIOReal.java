package frc.robot.subsystems.breakbeam;

import edu.wpi.first.wpilibj.DigitalInput;

public class BreakbeamIOReal implements BreakbeamIO {
  final DigitalInput breakbeam;
  final boolean invert;

  public BreakbeamIOReal(int port, boolean invert) {
    breakbeam = new DigitalInput(port);
    this.invert = invert;
  }

  @Override
  public void updateInputs(BreakbeamIOInputsAutoLogged inputs) {
    inputs.get = invert != breakbeam.get();
  }
}
