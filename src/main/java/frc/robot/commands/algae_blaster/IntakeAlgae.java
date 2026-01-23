package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeBlaster;

public class IntakeAlgae extends Command {
	private AlgaeBlaster algaeBlaster;

	public IntakeAlgae(AlgaeBlaster algaeBlaster) {	
		this.algaeBlaster = algaeBlaster;
		addRequirements(algaeBlaster);
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		algaeBlaster.Intake();
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

