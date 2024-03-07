/*
 * Autonomo Left - Esse autonomo coloca a nota no amplificador da esquerda
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.Dashboard;

public class AutonomousRight extends SequentialCommandGroup {
  // Declara SmartDashBoard
  private final Dashboard log = new Dashboard();
  
  /** Creates a new Autonomous001. */
  public AutonomousRight(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armsubsystem,
                        IntakeSubsystem intakesubsystem) {

    // MAC - Verificar inversão da direção do autonomo
    addCommands(
        new InstantCommand(() -> execAutonomous(
          driveSubsystem,
          armsubsystem,
          intakesubsystem
        )
      )        
    );

  }

  
  private void execAutonomous(DriveSubsystem drivesubsystem, 
                              ArmSubsystem armsubsystem,
                              IntakeSubsystem intakesubsystem) {

    //Alterar apenas essas 2 variaveis com medicao da arena
    double distMedidaLateral = 0; //185; //Distancia da parede lateral até a ultima ponta do robo
    double distMedidaInicio = 0; //Distancia do inicio da arena ate a ponta da frente do robo

    
    double distBaseLateral = 0; //Distancia da parede lateral até a ultima ponta do robo
    double distBaseInicio = 0; //Distancia do inicio da arena ate a ponta da frente do robo
        
    double startTime;

    drivesubsystem.setPidDesativo();
    drivesubsystem.zeroHeading();
    drivesubsystem.moveXY(25,0);
    drivesubsystem.moveXY(0,-35);    
    armsubsystem.angularArm(30);

    startTime = System.currentTimeMillis();
    while ( (System.currentTimeMillis() - startTime) < 1500 ) {
    }
    startTime = System.currentTimeMillis();
    while ( (System.currentTimeMillis() - startTime) < 2000 ) {
      intakesubsystem.inTake();
    }
    intakesubsystem.zeroTake();

    armsubsystem.angularArm(-135);     

    //drivesubsystem.moveXY(20,0);
    drivesubsystem.moveXY(0,-95);    

    //drivesubsystem.setPidAtivo();

    //drivesubsystem.moveXY(-40,0);
    //intakesubsystem.zeroTake();

  }

}
