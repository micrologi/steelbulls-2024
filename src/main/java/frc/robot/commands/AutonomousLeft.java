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

public class AutonomousLeft extends SequentialCommandGroup {
  // Declara SmartDashBoard
  private final Dashboard log = new Dashboard();
  
  /** Creates a new Autonomous001. */
  public AutonomousLeft(DriveSubsystem driveSubsystem, 
                        ArmSubsystem armsubsystem,
                        IntakeSubsystem intakesubsystem) {
    
    /* 
    // Criar configuração para trajetória
    TrajectoryConfig config = new TrajectoryConfig(
        1.0,  //AutoConstants.kMaxSpeedMetersPerSecond
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Adicione a cinemática para garantir que a velocidade máxima seja realmente obedecida
        .setKinematics(DriveConstants.kDriveKinematics);

    // Um exemplo de trajetória a seguir.Todas as unidades em metros.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Comece na origem voltada para a direção +x
        new Pose2d(0, 0, new Rotation2d(0)),
        
        // Passe por esses dois waypoints interiores, fazendo um caminho de curva 's'         
        List.of(
                new Translation2d(1,0)
        ),
        
        // Termine 3 metros direto à frente de onde começamos, enfrentando
        new Pose2d(2, 0, new Rotation2d(0)),
        config
    );

    var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 
          0, 
          0, 
          AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        driveSubsystem::getPose, // Interface funcional para alimentar o fornecedor
        DriveConstants.kDriveKinematics,
        // Controladores de posição
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem
    );
    */

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

    double startTime;

    drivesubsystem.setPidDesativo();
    drivesubsystem.zeroHeading();
    drivesubsystem.moveXY(25,0);
    drivesubsystem.moveXY(0,35);    
    armsubsystem.angularArm(30);

    startTime = System.currentTimeMillis();
    while ( (System.currentTimeMillis() - startTime) < 1000 ) {
    }
    startTime = System.currentTimeMillis();
    while ( (System.currentTimeMillis() - startTime) < 1500 ) {
      intakesubsystem.inTake();
    }
    intakesubsystem.zeroTake();

    armsubsystem.angularArm(-135);


    //drivesubsystem.moveXY(20,0);
    drivesubsystem.moveXY(0,95);    

  }

}
