package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.utils.Dashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;


public class ArmSubsystem extends SubsystemBase {
    // Declara SmartDashBoard
    private final Dashboard dash = new Dashboard();

    private final CANSparkMax m_controlSparkMaxLeft;
    private final RelativeEncoder m_controlEncoderLeft;
    private final SparkPIDController m_controlPIDControllerLeft;

    private final CANSparkMax m_controlSparkMaxRight;
    private final RelativeEncoder m_controlEncoderRight;
    private final SparkPIDController m_controlPIDControllerRight;

    private final Ultrasonic sensorEsquerdo = new Ultrasonic(8, 9);
    private final Ultrasonic sensorDireito = new Ultrasonic(6, 7);
    
    private final DigitalInput sensorDown = new DigitalInput(0);

    private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, kAngle;
    private double angleMotor;

    private boolean flagElevator;
    private double elevatorSetPoint;
//    private boolean flagDownArm = true;

    private DriveSubsystem m_robot;

    public ArmSubsystem() {            

      sensorEsquerdo.setAutomaticMode(true);
      sensorDireito.setAutomaticMode(true);

      // PID Coeficientes
      kP = 0.1; //0.1
      kI = 0;
      kD = 0;
      kFF = 0;
      
      kMinOutput = -0.3;      
      kMaxOutput = 0.3;

      angleMotor = 0;
      kAngle = 0;

      /* 
      int smartMotionSlot = 0;
      int minVel = 0;
      int maxVel = 400;
      int maxAcc = 100;
      */

      // MOTOR ESQUERDO
      m_controlSparkMaxLeft = new CANSparkMax(ArmConstants.armCANIdLeft, MotorType.kBrushless);
      m_controlSparkMaxLeft.restoreFactoryDefaults();
    
      m_controlEncoderLeft = m_controlSparkMaxLeft.getEncoder();
      m_controlPIDControllerLeft = m_controlSparkMaxLeft.getPIDController();
      m_controlPIDControllerLeft.setFeedbackDevice(m_controlEncoderLeft);

      m_controlEncoderLeft.setPositionConversionFactor(ArmConstants.armFactorConverion);
      m_controlEncoderLeft.setPosition(0);
  
      m_controlPIDControllerLeft.setP(kP);
      m_controlPIDControllerLeft.setI(kI);
      m_controlPIDControllerLeft.setD(kD);
      m_controlPIDControllerLeft.setFF(kFF);
      m_controlPIDControllerLeft.setOutputRange(kMinOutput,kMaxOutput);

      /* 
      m_controlPIDControllerLeft.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_controlPIDControllerLeft.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_controlPIDControllerLeft.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_controlPIDControllerLeft.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
      */

      m_controlSparkMaxLeft.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
      m_controlSparkMaxLeft.setSmartCurrentLimit(ArmConstants.armMotorCurrentLimit);

      //m_controlEncoder.setInverted(true);
      m_controlSparkMaxLeft.burnFlash();
      

      // MOTOR DIREITO
      m_controlSparkMaxRight = new CANSparkMax(ArmConstants.armCANIdRight, MotorType.kBrushless);
      m_controlSparkMaxRight.restoreFactoryDefaults();

      m_controlEncoderRight = m_controlSparkMaxRight.getEncoder();
      m_controlPIDControllerRight = m_controlSparkMaxRight.getPIDController();
      m_controlPIDControllerRight.setFeedbackDevice(m_controlEncoderRight);

      m_controlEncoderRight.setPositionConversionFactor(ArmConstants.armFactorConverion);
      m_controlEncoderRight.setPosition(0);
  
      m_controlPIDControllerRight.setP(kP);
      m_controlPIDControllerRight.setI(kI);
      m_controlPIDControllerRight.setD(kD);
      m_controlPIDControllerRight.setFF(kFF);
      m_controlPIDControllerRight.setOutputRange(kMinOutput,kMaxOutput);

      /* 
      m_controlPIDControllerRight.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_controlPIDControllerRight.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_controlPIDControllerRight.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_controlPIDControllerRight.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
      */
      
      m_controlSparkMaxRight.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
      m_controlSparkMaxRight.setSmartCurrentLimit(ArmConstants.armMotorCurrentLimit);

      m_controlSparkMaxRight.burnFlash();

      flagElevator = false;
      elevatorSetPoint = m_controlEncoderLeft.getPosition();
      //


      //kAngle = getRelationalTooth(90);
      //m_controlPIDControllerLeft.setReference(kAngle, CANSparkMax.ControlType.kPosition);
      //m_controlPIDControllerRight.setReference(-kAngle, CANSparkMax.ControlType.kPosition);
      
    } 

    private double getRelationalTooth(double angle) {
      double retAngle;

      double factorRelational = (ArmConstants.toothBig / ArmConstants.toothSmall);
      retAngle = -(Units.degreesToRadians(angle * factorRelational) * ArmConstants.armReduction);
    
      return retAngle;
    }

    public void angularArm(double angle) {
      //angle = angle + ArmConstants.angleAngularInit;

      angle = getRelationalTooth(angle);
      
      m_controlPIDControllerLeft.setReference(angle, CANSparkMax.ControlType.kPosition);
      m_controlPIDControllerRight.setReference(-angle, CANSparkMax.ControlType.kPosition);

      kAngle = angle;

    }

    public boolean fimDeCurso() {
      return !sensorDown.get();
    }

    @Override
    public void periodic() {

      dash.PV("S-ESQUERDO",sensorEsquerdo.getRangeMM() / 10);
      dash.PV("S-DIREITO",sensorDireito.getRangeMM() / 10);
      
      //Se fim de curso acionado, para e zera motores braço
      dash.PV("FIM-CURSO",fimDeCurso());
      
      dash.PV("LEFT-POSITION",m_controlEncoderLeft.getPosition());
      dash.PV("RIGHT-POSITION",m_controlEncoderRight.getPosition());

      dash.PV("flagElevator",flagElevator);
      
      /*  
      if (getSensorDown()) {
        flagElevator = false;
        elevatorSetPoint = 0;

        m_controlSparkMaxLeft.set(0);
        m_controlSparkMaxRight.set(0);      

        m_controlEncoderLeft.setPosition(-130);
        m_controlEncoderRight.setPosition(-130);  
      }
      */

    }
    

    public void elevatorMotor(double axis) {
      double erroElevator;
      double kp = 0.04;  //Aumentar força
      double proportional;
      double limit = 0.4;  //Limitar velocidade - Tremer

      //if (fimDeCurso() && (axis < 0)) {
      //  return;
      //}


      if (flagElevator) {

        if (Math.abs(axis) > 0.1) {
          m_controlSparkMaxLeft.set(axis);
          m_controlSparkMaxRight.set(-axis);      
          elevatorSetPoint = m_controlEncoderLeft.getPosition();
        } else {
          erroElevator = elevatorSetPoint - m_controlEncoderLeft.getPosition();

          proportional = Math.abs(kp * erroElevator) > limit ? limit * ((kp * erroElevator) / Math.abs(kp * erroElevator)) : kp * erroElevator; 

          m_controlSparkMaxLeft.set(proportional);
          m_controlSparkMaxRight.set(-proportional);        
        }

      }


      /* 
      if (Math.abs(axis) > 0.1) {
        m_controlSparkMaxLeft.set(axis);
        m_controlSparkMaxRight.set(-axis);        
      }
      */
    }

    public double getAngleMotor() {
      return angleMotor;
    }

    public void changeflagElevator() {
      flagElevator = !flagElevator;
      elevatorSetPoint = 0;
      //elevatorSetPoint = m_controlEncoderLeft.getPosition();

      m_controlEncoderLeft.setPosition(0);
      m_controlEncoderRight.setPosition(0);
      //m_controlSparkMaxLeft.set(0);
      //m_controlSparkMaxRight.set(0);
    }

    public void setPosition(double value) {
      m_controlEncoderLeft.setPosition(value);
      m_controlEncoderRight.setPosition(value);
    }

    public void alignRobot(DriveSubsystem m_robotDrive) {
      double distanceLeft = 0; 
      double distanceRight = 0;
      double angle;
      double rangeCm;
      double factor;
      double startTime;

      //IntakeSubsystem m_intakesystem

      for (int cont = 1; cont <= ArmConstants.counterCheckSensor; cont++) {
        distanceLeft += sensorEsquerdo.getRangeMM() / 10;
        distanceRight += sensorDireito.getRangeMM() / 10;
      }
      distanceLeft = distanceLeft / ArmConstants.counterCheckSensor;
      distanceRight = distanceRight / ArmConstants.counterCheckSensor;

      angle = Units.radiansToDegrees(Math.atan( (distanceLeft - distanceRight) / ArmConstants.sensorDistance));
      if ((Math.abs(angle) < ArmConstants.rangeAngle) && (distanceLeft < ArmConstants.rangeDistance)) {
        
        m_robotDrive.rotationRobot(m_robotDrive, angle);  
        m_robotDrive.setRotSetPoint();
        
        rangeCm = sensorEsquerdo.getRangeMM() / 10;
        startTime = System.currentTimeMillis();

        //while (true) {
        while ( (System.currentTimeMillis() - startTime) < 4000 ) {
          
          rangeCm = sensorEsquerdo.getRangeMM() / 10;          
          if (rangeCm > ArmConstants.rangeMaxDistance) {
            factor = -0.1;
          } else if (rangeCm < ArmConstants.rangeMinDistance) {
            factor = 0.1;
          } else {
            break;
          }

          m_robotDrive.drive(0,
          factor,
          0,
          0,
          DriveConstants.KFieldRelative, 
          DriveConstants.kRateLimit);      

        }          

      }

      //angularMotor(125);
      //m_intakesystem.inTake();

    }

    public double getDistanceUltrasonicCM(int iLeftRight) {
      if (iLeftRight == 1) {
        return sensorEsquerdo.getRangeMM() / 10;
      } else {
        return sensorDireito.getRangeMM() / 10;
      }
    }

    public void downArm() {

    }

}
