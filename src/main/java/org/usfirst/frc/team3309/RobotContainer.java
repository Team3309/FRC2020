package org.usfirst.frc.team3309;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team3309.commands.drive.DriveManual;
import org.usfirst.frc.team3309.commands.drive.DriveSimpleTest;
import org.usfirst.frc.team3309.subsystems.*;

import javax.naming.ldap.Control;

public class RobotContainer {

    //Constructs instances of our subsystems.
    private final ClimberSubsystem Climber = new ClimberSubsystem();
    private final CtrlPanelSubsystem Turner = new CtrlPanelSubsystem();
    private final DriveSubsystem Drive = new DriveSubsystem();
    private final IndexerSubsystem Indexer = new IndexerSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem();
    private final ShooterSubsystem Shooter = new ShooterSubsystem();
    private final VisionSubsystem Vision = new VisionSubsystem();

    public RobotContainer() {

        ConfigureButtonBindings();
        SetDefaultCommands();

    }


    private void SetDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(Drive, new DriveManual());
    }


    private void ConfigureButtonBindings() {
        new JoystickButton(Robot.oi.operatorController, XboxController.Button.kA.value)
                .whileHeld(new DriveSimpleTest(0.1, Drive));

        Button button = new JoystickButton(Robot.oi.operatorController, XboxController.Button.kB.value)
                .whileHeld(new RunCommand(() -> Drive.SetLeftRight(ControlMode.PercentOutput, 0.2, 0.2), Drive));
    }
}
