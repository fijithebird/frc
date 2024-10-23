package frc.robot.subsystems.Extension;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import frc.robot.subsystems.Extension.ExtensionIO;
//import frc.robot.subsystems.Extension.ExtensionIO.ExtensionIOStats;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Extension extends SubsystemBase implements ExtensionIO {

  private final ExtensionIO io;
  private final ExtensionIOStats stats = new ExtensionIOStats();
  private final ShuffleboardTab extensionShuffleboard;

 private GenericEntry appliedVolts;
  private GenericEntry velocityRpm;
  private GenericEntry position;
  private GenericEntry supplyCurrent;
  private GenericEntry statorCurrent;
  private GenericEntry temp;



    public enum state{
  
        position()
    }
    public Extension(ExtensionIO io) {
    this.io = io;
    this.extensionShuffleboard = Shuffleboard.getTab("Extension");
    position = this.extensionShuffleboard.add("extension Position", 0.0).getEntry();
    appliedVolts = this.extensionShuffleboard.add("extension Volts", 0.0).getEntry();
    velocityRpm = this.extensionShuffleboard.add("extension RPM ", 0.0).getEntry();
    supplyCurrent = this.extensionShuffleboard.add("extension Supply Current", 0.0).getEntry();
    statorCurrent = this.extensionShuffleboard.add("extension Stator Current", 0.0).getEntry();
    
  }


public void periodic(){
    io.updateStats(stats);
    UpdateTelemetry();
  }
  private void UpdateTelemetry() {
    appliedVolts.setDouble(stats.AppliedVolts);
    velocityRpm.setDouble(stats.VelocityRpm);
    position.setDouble(stats.PositionRads);
    supplyCurrent.setDouble(stats.SupplyCurrentAmps);
    statorCurrent.setDouble(stats.TorqueCurrentAmps);
    temp.setDouble(stats.TempCelsius);

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

