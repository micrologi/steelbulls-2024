package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Dashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final Dashboard dash = new Dashboard();
    
    private final CANSparkMax m_intakeDown;
    private final CANSparkMax m_intakeUp;
    
    //private final RelativeEncoder m_encoderUp;
    //private final SparkPIDController m_pidUp;
    //private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, kAngle;

    private final Ultrasonic sensorIntake = new Ultrasonic(4, 5);

    private boolean flagIntake = false;

    public IntakeSubsystem() {    
        
      m_intakeDown = new CANSparkMax(IntakeConstants.intakeCANidDOWN, MotorType.kBrushed);          
      m_intakeUp = new CANSparkMax(IntakeConstants.intakeCANidUP, MotorType.kBrushless);

      m_intakeDown.setSmartCurrentLimit(80);
      m_intakeUp.setSmartCurrentLimit(80);

      /* 

      m_intakeUp.restoreFactoryDefaults();
    
      // PID Coeficientes
      kP = 0.1; //0.1
      kI = 0;
      kD = 0;
      kFF = 0;
      
      kMinOutput = -1;      
      kMaxOutput = 1;
      
      m_encoderUp = m_intakeUp.getEncoder();
      m_pidUp = m_intakeUp.getPIDController();
      m_pidUp.setFeedbackDevice(m_encoderUp);

      m_encoderUp.setPositionConversionFactor(1);
      m_encoderUp.setPosition(0);
  
      m_pidUp.setP(kP);
      m_pidUp.setI(kI);
      m_pidUp.setD(kD);
      m_pidUp.setFF(kFF);
      m_pidUp.setOutputRange(kMinOutput,kMaxOutput);

      m_intakeUp.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
      m_intakeUp.setSmartCurrentLimit(ArmConstants.armMotorCurrentLimit);

      m_intakeUp.burnFlash();
      */   

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
    
    public void outTakeShooter() {      
        double startTime;

        m_intakeDown.setSmartCurrentLimit(30);
        m_intakeUp.setSmartCurrentLimit(30);

        m_intakeDown.set(IntakeConstants.downVelocity * 1);  
        m_intakeUp.set(IntakeConstants.downVelocity * 1);
              
        startTime = System.currentTimeMillis();
        while ( (System.currentTimeMillis() - startTime) < 125 ) {
        }

        m_intakeDown.set(0);  
        m_intakeUp.set(0);

        m_intakeDown.setSmartCurrentLimit(80);
        m_intakeUp.setSmartCurrentLimit(80);
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
