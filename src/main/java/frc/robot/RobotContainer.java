package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.TeleopDriveCommand;

public class RobotContainer {
	private CommandXboxController xbox;
	private SwerveDrive swerve;
	private MessagingSystem messaging;
	private Command autoCommand;
	private SendableChooser<Command> autonChooser;

	private final int DRIVER_PORT = 2;

	public RobotContainer() {
		swerve = SwerveDrive.getInstance();
		messaging = MessagingSystem.getInstance();
		setupAuto();
		setupDriveController();
	}

	public void setupAuto() {
		autonChooser = new SendableChooser<Command>();
		autonChooser.setDefaultOption("No Auto", null);
		Shuffleboard.getTab("Display").add("Auto Route", autonChooser);
	}

	public void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		TeleopDriveCommand swerveCommand = new TeleopDriveCommand(xbox);
		swerve.setDefaultCommand(swerveCommand);

		Trigger switchDriveModeButton = xbox.x();
		Trigger resetGyroButton = xbox.a();
		Trigger alignToTargetButton = xbox.rightBumper();
		Trigger slowModeButton = xbox.leftBumper();
		Trigger cancelationButton = xbox.start();

		switchDriveModeButton.toggleOnTrue(swerveCommand.toggleRobotCentricCommand());
		resetGyroButton.onTrue(swerveCommand.resetGyroCommand());
		slowModeButton.whileTrue(swerveCommand.toggleSlowModeCommand());
		alignToTargetButton.whileTrue(swerveCommand.toggleAlignToAngleCommand());
		cancelationButton.onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
	}


	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}

	public void autonomousInit() {
		messaging.setMessagingState(true);
		messaging.addMessage("Auto Started");
		autoCommand = autonChooser.getSelected();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.setMessagingState(true);
		messaging.addMessage("Teleop Started");
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}
