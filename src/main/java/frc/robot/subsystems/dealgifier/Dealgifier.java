package frc.robot.subsystems.dealgifier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

public class Dealgifier extends SubsystemBase {
	private final DealgifierIO io;
	private final DealgifierIOInputsAutoLogged inputs = new DealgifierIOInputsAutoLogged();

	public Dealgifier(DealgifierIO io) {
		this.io = io;
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Dealgifier", inputs);
	}

	@AutoLogOutput
	public Command runArmToPos(Angle target) {
		return startEnd(() -> io.runPosition(target), io::stop);
	}

	@AutoLogOutput
	public Command runArmVolts(double volts) {
		return startEnd(() -> io.runVolts(volts), io::stop);
	}

	@AutoLogOutput
	public Command runArmOpenLoop(double output) {
		return startEnd(() -> io.runOpenLoop(output), io::stop);
	}

	public boolean inRange(double expected) {
		return MathUtil.isNear(
				expected, inputs.positionDegs, 0.1
		);
	}
}
