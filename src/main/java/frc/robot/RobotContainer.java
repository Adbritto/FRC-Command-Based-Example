// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.dealgifier.Dealgifier;
import frc.robot.subsystems.dealgifier.DealgifierIO;
import frc.robot.subsystems.dealgifier.DealgifierIOTalonFX;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;
	private final Elevator elevator;
	private final Dealgifier dealgifier;

	private SwerveDriveSimulation driveSimulation = null;

	// Controller
	private final CommandXboxController driveController = new CommandXboxController(0);
	private final CommandXboxController mechController = new CommandXboxController(1);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL:
				// Real robot, instantiate hardware IO implementations
				drive =
						new Drive(
								new GyroIOPigeon2(),
								new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
								new ModuleIOTalonFXReal(TunerConstants.FrontRight),
								new ModuleIOTalonFXReal(TunerConstants.BackLeft),
								new ModuleIOTalonFXReal(TunerConstants.BackRight),
								(pose) -> {});

				elevator = new Elevator(new ElevatorIOTalonFX());
				dealgifier = new Dealgifier(new DealgifierIOTalonFX());
				break;

			case SIM:
				// Sim robot, instantiate physics sim IO implementations
				driveSimulation =
						new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

				drive =
						new Drive(
								new GyroIOSim(driveSimulation.getGyroSimulation()),
								new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
								new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
								new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
								new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
								driveSimulation::setSimulationWorldPose);

				elevator = new Elevator(new ElevatorIO() {});
				dealgifier = new Dealgifier(new DealgifierIO() {});
				break;

			default:
				// Replayed robot, disable IO implementations
				drive =
						new Drive(
								new GyroIO() {},
								new ModuleIO() {},
								new ModuleIO() {},
								new ModuleIO() {},
								new ModuleIO() {},
								(pose) -> {});

				elevator = new Elevator(new ElevatorIO() {});
				dealgifier = new Dealgifier(new DealgifierIO() {});
				break;
		}

		// Set up auto routines
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		autoChooser.addOption(
				"Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption(
				"Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Default command, normal field-relative drive
		drive.setDefaultCommand(
				DriveCommands.joystickDrive(
						drive,
						() -> -driveController.getLeftY(),
						() -> -driveController.getLeftX(),
						() -> -driveController.getRightX()));

		// Lock to 0° when A button is held
		driveController
				.a()
				.whileTrue(
						DriveCommands.joystickDriveAtAngle(
								drive,
								() -> -driveController.getLeftY(),
								() -> -driveController.getLeftX(),
								() -> new Rotation2d()));

		// Switch to X pattern when X button is pressed
		driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

		// Reset gyro to 0° when B button is pressed
		driveController
				.b()
				.onTrue(
						Commands.runOnce(
										() ->
												drive.setPose(
														new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
										drive)
								.ignoringDisable(true));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
