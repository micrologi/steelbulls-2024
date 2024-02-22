package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Não adicione variáveis estáticas a esta classe ou qualquer inicialização.A menos que você saiba o que
 * Você está fazendo, não modifique este arquivo, exceto para alterar a classe de parâmetros para o StartroBot
 * chamar. 
 */
public final class Main {
  private Main() {}

  /**
   * Função de inicialização principal.Não realize nenhuma inicialização aqui.
   * Se você alterar sua classe de robô principal, altere o tipo de parâmetro.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
