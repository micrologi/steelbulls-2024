package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousLine extends SequentialCommandGroup {
  // Declara SmartDashBoard
  //private final Dashboard dash = new Dashboard();
  
  /** Creates a new Autonomous001. */
  public AutonomousLine(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem) {
    
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
        new InstantCommand(() -> execAutonomous(driveSubsystem,
                                                armsubsystem)
      )        
    );

  }

  private void execAutonomous(DriveSubsystem driveSubsystem, ArmSubsystem armsubsystem) {
    
    driveSubsystem.moveXY(205,0);
    //driveSubsystem.moveXY(-150,0);

  }

}
