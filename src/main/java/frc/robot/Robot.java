package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.support.Telemetry;
import frc.robot.support.limelight.LimelightUtil;
// some imports no longer needed but leaving them here untill final version

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = new RobotContainer();
    String codeVersion = "0.0";
    private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);
    public static boolean navxCalibrated = false;

    @Override
    public void close() {
        super.close();
        LimelightUtil.stopPortForwarding();

        Telemetry.shutdown();
    }
    @Override
    public void robotInit() {
        m_robotContainer.m_LimelightBack.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrl.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrr.SetTagIDToTrack(-1);
        m_robotContainer.mLedStrand.changeLed(128, 0, 0);
        try {
            var cam = CameraServer.startAutomaticCapture();
            cam.setResolution(100, 100);
            cam.setFPS(60);
        } catch (Exception e) {
        }

        SmartDashboard.putString("Code Version", codeVersion);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putData(PDH);
        CommandScheduler.getInstance().run();
 
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        AlgaeMechanism.AUTORunning = true;
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = false;
        System.out.println(m_robotContainer.m_DriveTrain.m_FieldRelativeEnable);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        AlgaeMechanism.AUTORunning = true;
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {  
        // m_robotContainer.m_DriveTrain.ZeroGyro().schedule();
        AlgaeMechanism.AUTORunning = false;
        m_robotContainer.m_DriveTrain.m_FieldRelativeEnable = true;
        m_robotContainer.m_LimelightBack.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrl.SetTagIDToTrack(-1);
        m_robotContainer.m_LimelightFrr.SetTagIDToTrack(-1);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
