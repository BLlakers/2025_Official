package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.AlgaeMechanism;

public class AlgaePID extends Command {
  private AlgaeMechanism m_algaeMech;
  private double position;

  public AlgaePID(AlgaeMechanism algaeMech, Double pos) {
    position = pos;
    m_algaeMech = algaeMech;
    addRequirements(m_algaeMech);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeMech.initAlgaePID(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_algaeMech.setAlgaePIDPosition(position);
  }
  
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeMech.AlgaeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_algaeMech.CheckAlgaePID();
  }
}
