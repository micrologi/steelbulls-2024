package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonomousLine;
import frc.robot.commands.AutonomousLeft;
import frc.robot.commands.AutonomousRight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * Esta classe é onde a maior parte do robô deve ser declarada.Já que baseado em comando é um
 * paradigma "declarativo", muito pouca lógica do robô deve realmente ser tratada no {@link robot}
 * Métodos periódicos (exceto o Scheduler chama).Em vez disso, a estrutura do robô
 * (incluindo subsistemas, comandos e mapeamentos de botões) devem ser declarados aqui.
 * 
 * @author 
 */
public class RobotContainer {

  // Os subsistemas do robô
  private final ArmSubsystem m_armsubsystem = new ArmSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakesystem = new IntakeSubsystem();

  // MAC - Variaveis do autonomo
  private final AutonomousLine autonomousLine = new AutonomousLine(m_robotDrive, m_armsubsystem);
  private final AutonomousLeft autonomousLeft = new AutonomousLeft(m_robotDrive, m_armsubsystem, m_intakesystem);
  private final AutonomousRight autonomousRight = new AutonomousRight(m_robotDrive, m_armsubsystem, m_intakesystem);


  // MAC - caixinha seletora na drive station
  private static final String kAutoLeft = "Autonomo Left";
  private static final String kAutoRight = "Autonomo Right";
  private static final String kAutoLine = "Autonomo Line";
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Double> seletorLimeLight = new SendableChooser<>();  

  // O Joystick do robô
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);  
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_controlController = new Joystick(OIConstants.kControlControllerPort);

  /**
   * O contêiner para o robô. Contém subsistemas, dispositivos de entrada/saida e comandos.
   */
  public RobotContainer() {
      
    // Configura os botão padrão
    configureButtonBindings();

    // MAC - Declaração do autonomo, cada autonomo criado deve ser declarado
    autonomousLine.addRequirements(m_robotDrive);
    autonomousLeft.addRequirements(m_robotDrive);
    autonomousRight.addRequirements(m_robotDrive);

    // MAC - Adiciona as opções na select box da FRC Drive station
    m_chooser.setDefaultOption(kAutoLeft, autonomousLeft);
    m_chooser.addOption(kAutoRight, autonomousRight);
    m_chooser.addOption(kAutoLine, autonomousLine);

    SmartDashboard.putData("AUTONOMO",m_chooser);

    m_robotDrive.zeroHeading();
    //m_robotDrive.changeRateLimit();

    // Configure comandos padrão do Driving
    m_robotDrive.setDefaultCommand(

      new RunCommand(
        () -> m_robotDrive.drive(
          MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), //LeftY
          MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband), //LeftX
          MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kTurningDeadband), //RightX
          MathUtil.applyDeadband(m_driverController.getRawAxis(3), OIConstants.kTurningDeadband), //Velocity
          DriveConstants.KFieldRelative, 
          DriveConstants.kRateLimit),  
        m_robotDrive)
        
    );

    //Configura modo turbo
    /* 
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.axixVelocity(
                MathUtil.applyDeadband(m_driverController.getRawAxis(2), OIConstants.kDriveDeadband)
              ),
              m_robotDrive 
      )
    );
    */

    //Configura elevador
    m_armsubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_armsubsystem.elevatorMotor(
                    m_controlController.getRawAxis(5)
             ),
             m_armsubsystem
        )
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /* 
     * JOYSTICK 0 - DRIVING
     */

    //Zera a posição de frente
     
    JoystickButton setZeroHeading = new JoystickButton(m_driverController, 3);
    setZeroHeading.onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading()));

/* 
    JoystickButton changeFlagCenterTag = new JoystickButton(m_driverController, 3);
    changeFlagCenterTag.onTrue(new InstantCommand(
            () -> m_vision.changeFlagCenterTag()));
*/

    //Coloca as rodas em posição diamante para brecar o robô
    /* 
    JoystickButton setXRB = new JoystickButton(m_driverController, 2);
    // RunCommand roda sempre
    setXRB.whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
                  m_robotDrive));
    */

    JoystickButton alignRobot = new JoystickButton(m_driverController,2);
    alignRobot.onTrue(new InstantCommand(
            () -> m_armsubsystem.alignRobot(m_robotDrive))
    );


    //Define se o movimento do robo será baseado pela arena (Field) ou pela frente do Robo
    JoystickButton changeFieldRelative = new JoystickButton(m_driverController, 1);
    changeFieldRelative.onTrue(new InstantCommand(
            () -> m_robotDrive.changeFieldRelative()));                  

    //Altera o modo de direção (Suave / Agressiva)
    JoystickButton changeRateLimit = new JoystickButton(m_driverController, 4);
    changeRateLimit.onTrue(new InstantCommand(
            () -> m_robotDrive.changeRateLimit()));                  

    //Modo Turbo
    //JoystickButton highVelocity = new JoystickButton(m_driverController, 5);
    //highVelocity.onTrue(new InstantCommand(
    //        () -> m_robotDrive.highVelocity()));

    //Rotaciona o Robô 90º 
    JoystickButton rotationRobotL = new JoystickButton(m_driverController, 5);
    rotationRobotL.onTrue(new InstantCommand(
            () -> m_robotDrive.rotationRobot(m_robotDrive,-45)));                  

    JoystickButton rotationRobotR = new JoystickButton(m_driverController, 6);
    rotationRobotR.onTrue(new InstantCommand(
            () -> m_robotDrive.rotationRobot(m_robotDrive,45)));                  

    JoystickButton rotationRobot180 = new JoystickButton(m_driverController, 10);
    rotationRobot180.onTrue(new InstantCommand(
            () -> m_robotDrive.rotationRobot(m_robotDrive,180)));                  
        
    JoystickButton pidAtivo = new JoystickButton(m_driverController, 9);
    pidAtivo.onTrue(new InstantCommand(
            () -> m_robotDrive.setPidAtivo()));                  
                
    /* 
    //Altera frente do robo
    JoystickButton changeFront = new JoystickButton(m_driverController, 6);
    changeFront.onTrue(new InstantCommand(
            () -> m_robotDrive.changeFront()));                  
    */

    JoystickButton lowVelocity = new JoystickButton(m_driverController, 5);
    // RunCommand roda sempre
    lowVelocity.onFalse(new InstantCommand(
            () -> m_robotDrive.lowVelocity()));
        
        
            
    /* 
     * JOYSTICK 1 - PITCHING / ELEVATOR
     */
    JoystickButton armDown = new JoystickButton(m_controlController, 1);
    armDown.onTrue(new InstantCommand(
            () -> m_armsubsystem.angularArm(-135))
    );

    JoystickButton armMiddle = new JoystickButton(m_controlController, 3);
    armMiddle.onTrue(new InstantCommand(
            () -> m_armsubsystem.angularArm(-95))
    );

    JoystickButton armUp = new JoystickButton(m_controlController, 4);
    armUp.onTrue(new InstantCommand(
            () -> m_armsubsystem.angularArm(25))   //160
    );

    JoystickButton armInit = new JoystickButton(m_controlController, 2);
    armInit.onTrue(new InstantCommand(
            () -> m_armsubsystem.angularArm(0))   
    );


    //
    JoystickButton inTake = new JoystickButton(m_controlController, 5);
    // RunCommand roda sempre
    inTake.onTrue(new InstantCommand(
            () -> m_intakesystem.inTake())
    );

    JoystickButton zeroTake1 = new JoystickButton(m_controlController,5);
    // RunCommand roda sempre
    zeroTake1.onFalse(new InstantCommand(
            () -> m_intakesystem.zeroTake())
    );

    JoystickButton outTake = new JoystickButton(m_controlController, 6);
    // RunCommand roda sempre
    outTake.onTrue(new InstantCommand(
            () -> m_intakesystem.outTake())
    );

    JoystickButton zeroTake2 = new JoystickButton(m_controlController,6);
    // RunCommand roda sempre
    zeroTake2.onFalse(new InstantCommand(
            () -> m_intakesystem.zeroTake())
    );


    JoystickButton changeflagElevator = new JoystickButton(m_controlController,9);
    // RunCommand roda sempre
    changeflagElevator.onTrue(new InstantCommand(
            () -> m_armsubsystem.changeflagElevator())
    );


  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }    

}
