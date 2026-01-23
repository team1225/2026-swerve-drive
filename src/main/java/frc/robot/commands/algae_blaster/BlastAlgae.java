package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeBlaster;

public class BlastAlgae extends Command {
    
    private AlgaeBlaster algaeBlaster;

    public BlastAlgae(AlgaeBlaster algaeBlaster) {    
        this.algaeBlaster = algaeBlaster;
        addRequirements(algaeBlaster);
    }

    @Override
    public void initialize() {
        algaeBlaster.Eject();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeBlaster.Stop();
    }
}
