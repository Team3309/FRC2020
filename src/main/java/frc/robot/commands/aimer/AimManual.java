package frc.robot.commands.aimer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AimManual extends CommandBase {

    private ArmSubsystem Arm;
    private XboxController Controller;

    public AimManual(ArmSubsystem arm, XboxController controller) {
        Arm = arm;
        Controller = controller;
        addRequirements(arm);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Arm.MoveArmManually(Controller.getY(GenericHID.Hand.kRight)*Config.EncoderCountsPerDegree);
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
