package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import frc.utils.Dashboard;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Dashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final Dashboard dash = new Dashboard();
    private final CANSparkMax m_intake;
    private final Ultrasonic sensorIntake = new Ultrasonic(4, 5);

    public IntakeSubsystem() {    
        m_intake = new CANSparkMax(IntakeConstants.intakeCAN, MotorType.kBrushed);  
        sensorIntake.setAutomaticMode(true);    
    } 

    @Override
    public void periodic() {
        
        dash.PV("S-GARRA",sensorIntake.getRangeMM() / 10);

        if ((sensorIntake.getRangeMM() / 2) < 2) {
          //m_intake.set(0);
        }
    }  

    public void inTake() {      
        m_intake.set(IntakeConstants.inVelocity * 1);
    }
    
    public void outTake() {      
        m_intake.set(IntakeConstants.inVelocity * -1);
    }

    public void zeroTake() {      
        m_intake.set(0);
    }

  }
