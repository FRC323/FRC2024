package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class FireNoteAuto extends SequentialCommandGroup{
    public FireNoteAuto(DriveSubsystem driveSubsystem,IntakeSubsystem intake, ArmSubsystem armSubsystem){
        addCommands(
            new ShootSpeaker(armSubsystem),
            new ParallelCommandGroup(
                new HandoffProc(intake, armSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    PathFollowerCommands.followPathFromFile(driveSubsystem, "Pick Note")
                )
            ),
            new ShootSpeaker(armSubsystem),
            new SetIntakeFolded(intake, armSubsystem)
        );
    }
}