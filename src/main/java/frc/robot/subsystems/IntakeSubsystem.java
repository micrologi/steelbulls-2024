/**Classe IntakeSubsystem - Mais uma implementação genial da Intake
 * @author Marlon Andrei (The Master of Java)
 * @version 1.00 
 */
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Dashboard;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeSubsystem extends SubsystemBase {
    private final Dashboard dash = new Dashboard();
    
    private final CANSparkMax m_intakeDown;
    private final CANSparkMax m_intakeUp;
    
    private final Ultrasonic sensorIntake = new Ultrasonic(4, 5);
    private boolean flagIntake = false;

    public IntakeSubsystem() {    
        
      m_intakeDown = new CANSparkMax(IntakeConstants.intakeCANidDOWN, MotorType.kBrushed);          
      m_intakeUp = new CANSparkMax(IntakeConstants.intakeCANidUP, MotorType.kBrushless);

      m_intakeDown.setSmartCurrentLimit(stabilizeCurrent(80));
      m_intakeUp.setSmartCurrentLimit(stabilizeCurrent(80));

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
        m_intakeDown.set(IntakeConstants.downVelocity * 1);
        m_intakeUp.set(IntakeConstants.downVelocity * 1);
    }

    public void outTake() {      
        m_intakeDown.set(IntakeConstants.downVelocity * -1);
        m_intakeUp.set(IntakeConstants.downVelocity * -1);
    }

    public void inTakeMiddle() {      
        m_intakeDown.set(IntakeConstants.downVelocity * 1);
        //m_intakeUp.set(IntakeConstants.downVelocity * 1);
    }


    public void axisTake(double laxis, double raxis) {
 
      if (laxis > 0.3) {
        inTake();
        flagIntake = true;
      } else if (raxis > 0.3) {
        outTake();
        flagIntake = true;
      } else if (flagIntake) {
        zeroTake();
        flagIntake = false;
      }

    }

    public int stabilizeCurrent(int current) {
      double batVoltage = RobotController.getBatteryVoltage(); 
      return (int)(current + (current - (current / Constants.IntakeConstants.currentMaxBattery) * batVoltage));
    }
    
    public void outTakeShooter() {      
        double startTime;
                
        int currentMin = stabilizeCurrent(30);
        int currentMax = stabilizeCurrent(80);

        m_intakeDown.setSmartCurrentLimit(currentMin);
        m_intakeUp.setSmartCurrentLimit(currentMin);

        m_intakeDown.set(IntakeConstants.downVelocity * 1);  
        m_intakeUp.set(IntakeConstants.downVelocity * 1);
              
        startTime = System.currentTimeMillis();
        while ( (System.currentTimeMillis() - startTime) < 125 ) {
        }

        m_intakeDown.set(0);  
        m_intakeUp.set(0);

        m_intakeDown.setSmartCurrentLimit(currentMax);
        m_intakeUp.setSmartCurrentLimit(currentMax);
        m_intakeDown.set(IntakeConstants.downVelocity * -1);

        startTime = System.currentTimeMillis();
        while ( (System.currentTimeMillis() - startTime) < 225) {
        }

        m_intakeUp.set(IntakeConstants.downVelocity * -1);

        startTime = System.currentTimeMillis();
        while ( (System.currentTimeMillis() - startTime) < 125) {
        }

    }

    public void zeroTake() {      
        m_intakeDown.set(0);
        m_intakeUp.set(0);
    }

  }
