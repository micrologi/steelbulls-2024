package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.DriveConstants;

import frc.utils.Dashboard;

public class VisionSubsystem extends SubsystemBase { 
  private final DriveSubsystem m_robot;
  private final Dashboard dash = new Dashboard();

  private double tx, ty, tz; 
  private boolean flagCenterTag = false;
  private int cont = 0;

  public VisionSubsystem(DriveSubsystem m_robotDrive) {
    m_robot = m_robotDrive;

    for (int port = 5800; port <= 5807; port++) {
        PortForwarder.add(port, "limelight.local", port);
    }

  }

  @Override
  public void periodic() {
    /* 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose_robotspace");

    double[] values = targetpose_robotspace.getDoubleArray(new double[] { 0, 0, 0 });

    tx = (values[0] * 100);
    ty = (values[1] * 100);
    tz = (values[2] * 100);

    if (flagCenterTag) {
      if (centerTag(tx)) {
        cont++;

        if (cont > 2) {  
          flagCenterTag = false;
        }

      } else {
        cont = 0;
      }
    } else {
      cont = 0;
    }

    dash.PV("VAL-X",tx);
    dash.PV("VAL-Y",ty);
    dash.PV("VAL-Z",tz);
        */
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTZ() {
    return tz;
  }

  public void changeFlagCenterTag() {
    flagCenterTag = !flagCenterTag; 
  }

  public boolean centerTag(double txcenter) {
    double offset = 0;

    double kp = 0.1;
    double proportional = 0;
    double limit = 0.1;

    proportional = (kp * txcenter);
    proportional = Math.abs(proportional) > limit ? limit * (proportional / Math.abs(proportional)) : proportional; 

    dash.PV("PROPOR",proportional);
    dash.PV("RANDOM-PROPOR",Math.random());

    m_robot.drive(0,
    proportional,
    0,
    0,
    DriveConstants.KFieldRelative, 
    DriveConstants.kRateLimit);      

    if (Math.abs(tx) < 3) {
        return true;
    } else {
        return false;
    }

  }

}
