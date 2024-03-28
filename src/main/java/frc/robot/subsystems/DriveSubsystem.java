/**Classe DriveSubsystem - Essa classe que faz o Rôbo andar 
 * @author Não sei ao certo, admito que peguei da Net mas tive que arruma-lá! - Marlon Andrei (O Mito)
 * @version 2.00 (Como eu mechi, considero a minha versão a 2) * 
 */
package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;

import frc.utils.SwerveUtils;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.PigeonConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Dashboard;
import edu.wpi.first.wpilibj.Ultrasonic;


public class DriveSubsystem extends SubsystemBase {

  // Declara SmartDashBoard
  private final Dashboard log = new Dashboard();

  // Cria MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      false);
    
  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      false);

  // O sensor giroscópio
  private final Pigeon2 m_gyro = new Pigeon2(PigeonConstants.Pigeon2ID);

  // Variáveis de filtro de taxa de galho para controlar a aceleração lateral
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;


  
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private double rotSetPoint = 0;
  private boolean flagDrive = true;
  private boolean pidAtivo = true;

  // Classe de odometria para rastrear posição de robô
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics, // Configuração cinemática do drive
      Rotation2d.fromDegrees(m_gyro.getAngle()), // Orientação do robô obtida do giroscópio
      new SwerveModulePosition[] { // Posições das rodas
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });      

  /** Construtor */
  public DriveSubsystem() {

  }

  @Override
  /*
   * Esta anotação indica que o método seguinte (public void periodic() {) está 
   * substituindo um método da classe pai.
   */
  public void periodic() {

   log.PV("PID", pidAtivo);

    // Atualize a odometria no bloco periódico
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
    });

  }

  /**
   * Retorna a posição atualmente estimada do robô.
   *
   * @return The posiçao.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Redefina a odometria para a pose especificada.
   *
   * @param pose A pose para a qual definir a odometria.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Método para dirigir o robô usando informações do joystick.
   *
   * @param xSpeed        Velocidade do robô na direção X (para a frente).
   * @param ySpeed        Velocidade do robô na direção y (lateral).
   * @param rot           Taxa angular do robô.
   * @param fieldRelative Se as velocidades X e Y fornecidas são relativas ao
   *                      campo.
   * @param rateLimit     Se deve permitir a limitação da taxa para controle mais
   *                      suave.
   */
  public void drive(double xSpeed, 
                    double ySpeed, 
                    double rot, 
                    double velocity, 
                    boolean fieldRelative, 
                    boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;     
    double erro = 0;
    double kp = 0.03;  //Força da rotação
    boolean flagSetPoint = false;
    double rangeRotational = 0.3;  //Velocidade maxima da rotação

    //Captura(time,x,y,rot)
    //As diferencas de time definem quanto tempo fica um valor
    
    log.PV("RELACAO-ARENA",fieldRelative);
    log.PV("MODO-SUAVE",rateLimit);

    //Ajusta velocidade do robo
    if (velocity > 0.3) {
      highVelocity();
    } else {
      lowVelocity();
    }

    if (pidAtivo) {

      if ((xSpeed != 0) || (ySpeed != 0)) {
        if (flagDrive) {
          rotSetPoint = m_gyro.getRotation2d().getDegrees();
          flagDrive = false;
        }
      } else {
        flagDrive = true;
      }

      if (rot == 0) {
        erro = m_gyro.getRotation2d().getDegrees() - rotSetPoint;      
        rot = kp * erro;

        if (Math.abs(rot) > rangeRotational) {
          rot = rangeRotational * (rot / Math.abs(rot) );
        }

      } else {
        flagSetPoint = true;
      }
  
    }

    if (rateLimit) {

      double inputTranslationDir = Math.atan2(ySpeed, xSpeed); // Converter polaridade XY para limitar a taxa
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
      double directionSlewRate; // Calcule a taxa de direção com base em uma estimativa da aceleração lateral

      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // algum número alto que significa que a taxa de giro é efetivamente instantânea
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;

      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);

        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);

      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // algum número pequeno para evitar erros de ponto flutuante com
                                              // verificação de igualdade
                                              // Mantenha o CurrentTranslationdIr inalterado
          m_currentTranslationMag = m_magLimiter.calculate(0.0);

        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);

        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);

      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Converte as velocidades comandadas nas unidades corretas para o drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;

    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    log.PV("GYRO",-m_gyro.getRotation2d().getDegrees());

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, 
                                                    ySpeedDelivered, 
                                                    rotDelivered,
                                                    m_gyro.getRotation2d() )
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    if (pidAtivo) {
      if (flagSetPoint) {
        rotSetPoint = m_gyro.getRotation2d().getDegrees();
      }  
    }

  }

  /**
   * Coloca as rodas em uma formação diamante para impedir o movimento.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void axixVelocity(double velocity) {
    DriveConstants.kMaxSpeedMetersPerSecond = 2.5 + (velocity * 2);
  }

  public void setRotSetPoint() {
    rotSetPoint = m_gyro.getRotation2d().getDegrees();
  }

  public void highVelocity() {
    DriveConstants.kMaxSpeedMetersPerSecond = 4.8;
  }

  public void lowVelocity() {
    DriveConstants.kMaxSpeedMetersPerSecond = 3.5;
  }

  /**
   * Seleciona o swerve ModuleStates.
   *
   * @param desiredStates Os estados do SwerveModule.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Redefina os codificadores de unidade para ler atualmente uma posição de 0.
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zera a cabeça do robô. */
  public void zeroHeading() {
    m_gyro.reset();
    rotSetPoint = 0;
  }

  public void changeRotation(double rot) {
    //m_gyro.addYaw(180);
    m_gyro.setYaw(rot);
  } 

  //MAC - Troca a frente do robô
  public void changeFront() {
    m_gyro.setYaw(180);
  }

  //MAC - Essa função espetacular rotaciona o robô na direita ou esquerda automaticamente
  public void rotationRobot(DriveSubsystem m_robotDrive,double angulo) {
    double angleat = getHeading();

    double graus = angleat + angulo;
    double logica = -1;
    double kp = 0.02;  // Força
    double velocidade = 0.4; //Velocidade
    int qRotation = 10; 

    double startTime = System.currentTimeMillis();

    int cont = 0;

    while ((cont < qRotation) && ((System.currentTimeMillis() - startTime) < 3000)) {
      double erro = getHeading() - graus;
      double proporcional = kp * erro * logica; 

      if (Math.round(erro) == 0) {
        cont++;
      } else {
        cont = 0;
      }

      if (Math.abs(proporcional) > velocidade) {
        proporcional = velocidade * (proporcional / Math.abs(proporcional));
      }

      m_robotDrive.drive(0,
      0,
      proporcional,
      0,
      DriveConstants.KFieldRelative, 
      DriveConstants.kRateLimit);    
      
    }

      /* 
      while (angleat <= anglenew) {

        m_robotDrive.drive(0,
        0,
        1,
        DriveConstants.KFieldRelative, 
        DriveConstants.kRateLimit);

        angleat = getHeading();
      }
    
    } else {

      while (angleat >= anglenew) {
        m_robotDrive.drive(0,
        0,
        -1,
        DriveConstants.KFieldRelative, 
        DriveConstants.kRateLimit);

        angleat = getHeading();
      }

    }
    */

  }

  /**
   * Retorna o angulo da cabeça do robô.
   *
   * @return Posição da cabeça do robô em graus, de -180 a 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Retorna a taxa de virada do robô.
   *
   * @return A taxa de giro do robô, em graus por segundo
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Altera a condição do RateLimit (Direção mais suave ou não)
   */
  public void changeRateLimit() {
    DriveConstants.kRateLimit = !DriveConstants.kRateLimit;
  }

  public void changeFieldRelative() {
    DriveConstants.KFieldRelative = !DriveConstants.KFieldRelative;
  }

  public void setPidAtivo() {
    pidAtivo = !pidAtivo; 
  }

  public void setPidDesativo() {
    pidAtivo = false;
  }

  public void moveXY(double xCentimeters, double yCentimeters) { 
    double startTime;  
    double vRealMedido = 0.65;  //Valor medido por trena do meio da roda inicio ao fim
    double vEncoder = 0.692;  //Leitura do encoder final após 2 segundos de movimento
    double setpoint;
    double erro;
    double kp = 0.05; // Força - precisa de força para chegar (Altera over shoot)
    double controle = 0;
    double limit = 0.1;  // Velocidade / Potencia
    double velocidadeUltrasonico = 0.2;
    double tentativasAjuste = 50;
    int cont;

    zeroHeading();
    resetEncoders();

    if (xCentimeters != 0) {
      setpoint = Math.abs((vEncoder / vRealMedido) * (xCentimeters));
    } else {
      setpoint = Math.abs((vEncoder / vRealMedido) * (yCentimeters));
    }

    
 //   if (dUltrasonico != 0) {
/* 
      while (armsubsystem.getDistanceUltrasonicCM(1) > dUltrasonico) {
        
        drive(
          -velocidadeUltrasonico,
          0, 
          0, 
          true, 
          true
        );
      }
*/

  //  } else {

      cont = 1;
      while (cont <= tentativasAjuste) {

        log.PV("GETDRIVEPOSITION",m_frontLeft.getDrivePosition() * 100);

        erro = setpoint - Math.abs(m_frontLeft.getDrivePosition()) * 100;
        controle = kp * erro;

        if (Math.round(Math.abs(erro)) == 0) {
          cont++;
        } else {
          cont = 1;
        }

        if (Math.abs(controle) > limit) {
          controle = limit * (Math.abs(controle) / controle);
        }

        if (xCentimeters != 0) {
          
          if (xCentimeters > 0) {
            controle = -controle;
          }

          drive(
            controle,
            0, 
            0, 
            0,
            false, 
            true
          );
        } else {

          if (yCentimeters < 0) {
            controle = -controle;
          }

          drive(
            0, 
            controle,
            0, 
            0,
            false, 
            true
          );
        }
    
      }
   // }    

    log.PV("FIM",m_frontLeft.getDrivePosition() * 100);
  }


}
