package frc.robot.subsystems.dealgifier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Degrees;

public class DealgifierIOTalonFX implements DealgifierIO{

	// Hardware
	private final TalonFX armMotor = new TalonFX(13);

	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Angle> position;
	private final StatusSignal<Current> torqueCurrent;
	private final StatusSignal<Voltage> appliedVolts;
	private final StatusSignal<Temperature> temp;
	private final StatusSignal<Current> supplyCurrent;

	private final TrapezoidProfile armProfile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(
					Units.degreesToRadians(10),
					Units.degreesToRadians(100)
			));
	private TrapezoidProfile.State lastState = new TrapezoidProfile.State();

	private final LinearSystem<N2, N1, N2> armSystem =
			LinearSystemId.createSingleJointedArmSystem(
					DCMotor.getKrakenX60(1),
					100,
					50
			);

	private final KalmanFilter<N2, N1, N1> armObserver =
			new KalmanFilter<>(
					Nat.N2(),
					Nat.N1(),
					(LinearSystem<N2, N1, N1>) armSystem.slice(0),
					VecBuilder.fill(0.015, 0.17),
					VecBuilder.fill(0.01),
					0.020
			);

	private final LinearQuadraticRegulator<N2, N1, N1> armController =
			new LinearQuadraticRegulator<>(
					(LinearSystem<N2, N1, N1>) armSystem.slice(0),
					VecBuilder.fill(1.0, 10.0),
					VecBuilder.fill(12.0),
					0.020
			);

	private final LinearSystemLoop<N2, N1, N1> armLoop =
			new LinearSystemLoop<>(
					(LinearSystem<N2, N1, N1>) armSystem.slice(0),
					armController,
					armObserver,
					12.0,
					0.020
			);

	public DealgifierIOTalonFX() {
		// Config Motor
		// Config
		var config = new TalonFXConfiguration();
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		config.Feedback.SensorToMechanismRatio = 50.0 / 360.0;
		config.CurrentLimits.StatorCurrentLimit = 80.0;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.SoftwareLimitSwitch =
				new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withReverseSoftLimitEnable(true)
						.withReverseSoftLimitThreshold(0);
		armMotor.getConfigurator().apply(config);

		position = armMotor.getPosition();
		velocity = armMotor.getVelocity();
		appliedVolts = armMotor.getMotorVoltage();
		torqueCurrent = armMotor.getTorqueCurrent();
		supplyCurrent = armMotor.getSupplyCurrent();
		temp = armMotor.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, temp);
		torqueCurrent.setUpdateFrequency(1000);
		ParentDevice.optimizeBusUtilizationForAll(armMotor);
	}

	@Override
	public void updateInputs(DealgifierIOInputs inputs) {
		boolean connected =
				BaseStatusSignal.refreshAll(
								position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
						.isOK();

		inputs.motorConnected = connected;
		inputs.positionDegs = position.getValueAsDouble();
		inputs.velocityDegsPerSec = velocity.getValueAsDouble();
		inputs.appliedVolts = appliedVolts.getValueAsDouble();
		inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
		inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
		inputs.tempCelsius = temp.getValueAsDouble();
	}

	@Override
	public void runOpenLoop(double output) {
		armMotor.set(output);
	}

	@Override
	public void runVolts(double volts) {
		armMotor.setVoltage(volts);
	}

	@Override
	public void stop() {
		armMotor.stopMotor();
	}

	@Override
	public void runPosition(Angle positionDegs) {
		TrapezoidProfile.State goal;
		goal = new TrapezoidProfile.State(positionDegs.in(Degrees), 0.0);
		lastState = armProfile.calculate(0.020, lastState, goal);
		armLoop.setNextR(lastState.position, lastState.velocity);
		armLoop.correct(VecBuilder.fill(armMotor.getPosition().getValueAsDouble()));
		armLoop.predict(0.020);
		double nextVoltage = armLoop.getU(0);
		armMotor.setVoltage(nextVoltage);
	}

	@Override
	public void reset() {
		armLoop.reset(
				VecBuilder.fill(
						armMotor.getPosition().getValueAsDouble(),
						armMotor.getVelocity().getValueAsDouble()));

		lastState = new TrapezoidProfile.State(
				armMotor.getPosition().getValueAsDouble(),
				armMotor.getVelocity().getValueAsDouble());
	}
}
