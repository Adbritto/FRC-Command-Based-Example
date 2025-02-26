package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

public class ClimberIOTalonFX implements ClimberIO {
  // Hardware
  private final TalonFX climberMotor;

  private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(Units.Degrees.of(0));

  // Status Signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> supplyCurrent;

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(20);

    // Config
    var config =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(125))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0)
                    .withKG(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Units.DegreesPerSecond.of(0))
                    .withMotionMagicAcceleration(Units.DegreesPerSecondPerSecond.of(0)));
    climberMotor.getConfigurator().apply(config);

    position = climberMotor.getPosition();
    velocity = climberMotor.getVelocity();
    appliedVolts = climberMotor.getMotorVoltage();
    torqueCurrent = climberMotor.getTorqueCurrent();
    supplyCurrent = climberMotor.getSupplyCurrent();
    temp = climberMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, temp);
    torqueCurrent.setUpdateFrequency(1000);
    ParentDevice.optimizeBusUtilizationForAll(climberMotor);
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
            .isOK();

    inputs.motorConnected = connected;
    inputs.positionInch = position.getValueAsDouble();
    inputs.velocityInchPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    climberMotor.set(output);
  }

  @Override
  public void runVolts(double volts) {
    climberMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    climberMotor.stopMotor();
  }

  @Override
  public void runPosition(Angle positionDegrees) {
    climberMotor.setControl(motionRequest.withPosition(positionDegrees));
  }
}
