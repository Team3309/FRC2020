package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class LiftRobot extends CommandBase {

    ClimberSubsystem climber;
    XboxController controller;

    public LiftRobot (ClimberSubsystem climber, XboxController controller) {
        this.climber = climber;
        this.controller = controller;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(RobotContainer.getRobotState() == RobotContainer.RobotState.CLIMBING) {
            double yRaw = Math.max(controller.getY(GenericHID.Hand.kRight), 0);

            if (yRaw < Config.operatorControllerDeadzoneRightStick) {
                yRaw = 0;
            }
            climber.moveWinch(yRaw * Config.climberMaxPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.moveWinch(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
