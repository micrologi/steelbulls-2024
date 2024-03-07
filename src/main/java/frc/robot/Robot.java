package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A VM está configurada para executar automaticamente esta classe e para chamar as funções correspondentes a
 * Cada modo, conforme descrito na documentação do TimedRobot.Se você mudar o nome desta classe ou
 * O pacote Após criar este projeto, você também deve atualizar o arquivo Build.gradle no
 * projeto. 
 */
public class Robot extends TimedRobot { 

  // Declara SmartDashBoard
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  /**
   * Esta função é executada quando o robô é iniciado e deve ser usado para qualquer
   * Código de inicialização.   
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * Esta função é chamada de pacote de robô, independentemente do modo.Use isso para itens como
   * Diagnósticos que você deseja executar durante deficientes, autônomos, teleoperados e testados.
   *
   * Isso funciona após o modo funções periódicas específicas, mas antes de LiveWindow e
   * Atualização integrada do SmartDashboard.   
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * Este autônomo (junto com o código do Chooser acima) mostra como selecionar entre diferentes
   * Modos autônomos usando o painel.O código do escolhedor enviável funciona com o java
   * SmartDashboard.Se você preferir o painel Labview, remova todo o código do Chooser e
   * Uncomment a linha Getting para obter o nome automático da caixa de texto abaixo do giroscópio
   *
   * Você pode adicionar modos de automóveis adicionais adicionando comparações adicionais à estrutura do Switch
   * Abaixo com cordas adicionais.Se estiver usando o sendableChooser, adicione -os ao
   * Código de Chooser também acima.   
   */
  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** Esta função é chamada periodicamente durante autônoma. */
  @Override
  public void autonomousPeriodic() {
     
    /* 
    switch (m_autoSelected) {
      case kAutoRight:
        // Coloque o código automático personalizado aqui
        break;
      case kAutoLine:
        // Coloque o código automático personalizado aqui
        break;
      case kAutoLeft:
      default:
        // Coloque o código automático padrão aqui
        break;
    }
    */
    
  }

  /** Esta função é chamada uma vez quando o Teleop é ativado. */
  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** Esta função é chamada periodicamente durante o controle do operador. */
  @Override
  public void teleopPeriodic() {

  }

  /** Esta função é chamada uma vez quando o robô é desativado. */
  @Override
  public void disabledInit() {

  }

  /** Esta função é chamada periodicamente quando desativada. */
  @Override
  public void disabledPeriodic() {

  }

  /** Esta função é chamada uma vez quando o modo de teste é ativado. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Esta função é chamada periodicamente durante o modo de teste. */
  @Override
  public void testPeriodic() {

  }

  
}
