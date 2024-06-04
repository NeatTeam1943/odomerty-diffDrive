package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;


public class RobotContainer {
  private final DriveTrain m_drive;
  private final CommandXboxController m_joystick;
  
  public RobotContainer() {
    m_joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_drive = new DriveTrain();
   
    m_drive.setDefaultCommand(m_drive.drive(m_joystick));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
