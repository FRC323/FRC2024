package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GotoArmIntakeState extends Command{
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private double armTarget;
    private double intakeTarget;

    private enum SystemState{
        ARM_DOWN_INTAKE_IN,
        ARM_DOWN_INTAKE_OUT,
        ARM_DOWN_INTAKE_INTEFERE,
        ARM_UP_INTAKE_IN,
        ARM_UP_INTAKE_OUT,
        ARM_UP_INTAKE_INTERFERE,
        INVALID_STATE
    }

    private SystemState currentState;
    private SystemState targetState;

    public GotoArmIntakeState(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, double armTarget, double intakeTarget){
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.armTarget = armTarget;
        this.intakeTarget = intakeTarget;

        targetState = getStateFromAngles(armTarget, intakeTarget);

        addRequirements(armSubsystem,intakeSubsystem);
    }

    @Override
    public void execute(){
        currentState = getStateFromAngles(
            intakeSubsystem.getWristAngleRads(),
            armSubsystem.getArmAngleRads()
        );

        //Todo: Handle INVALID_STATE

        //If intake and arm are already in current zones
        if(currentState == targetState){
            armSubsystem.setTargetRads(armTarget);
            intakeSubsystem.setTargetRads(intakeTarget);
            return;
        }

        //Unlocks intake to go to other states
        if(currentState == SystemState.ARM_DOWN_INTAKE_INTEFERE){
            intakeSubsystem.setTargetRads(Intake.UNFOLDED_POSE);
            return;
        }





        /*
         * States that can move to certain state
         * ARM_DOWN_INTAKE_IN:
         *      ARM_DOWN_INTAKE_IN
         *      ARM_UP_INTAKE_IN
         * ARM_DOWN_INTAKE_OUT:
         *      ARM_DOWN_INTAKE_OUT
         *      ARM_DOWN_INTAKE_INTERFERE
         *      ARM_UP_INTAKE_OUT
         * ARM_DOWN_INTAKE_INTERFERE:
         *      ARM_DOWN_INTAKE_OUT
         *      ARM_DOWN_INTAKE_INTERFERE
         *      ARM_UP_INTAKE_OUT   
         * ARM_UP_INTAKE_IN:
         *      ARM_DOWN_INTAKE_IN
         *      ARM_UP_INTAKE_OUT
         *      ARM_UP_INTAKE_IN
         *      ARM_UP_INTAKE_INTERFERE
         * ARM_UP_INTAKE_OUT:
         *      ARM_DOWN_INTAKE_OUT
         *      ARM_UP_INTAKE_OUT
         *      ARM_UP_INTAKE_IN
         *      ARM_UP_INTAKE_INTERFERE
         * ARM_UP_INTAKE_INTERFERE:
         *      ARM_UP_INTAKE_OUT
         *      ARM_UP_INTAKE_IN
         *      ARM_UP_INTAKE_INTERFERE
         */

         //Generalization
         /*
          * If arm is up, intake can freely move
          * If arm is down, intake can only move between Out and Locked
          *
          * If intake is In, arm can freely move
          * If intake is Out, arm can freely move
          * If intake is Interfereing, arm can not move
          */

    }

    @Override
    public boolean isFinished(){
        return false;
        //Todo:
    }

    public SystemState getStateFromAngles(double armAngle, double intakeAngle){
        //Error State
        if(
            intakeAngle > Intake.SOFT_LIMIT_MAX
            || intakeAngle < Intake.SOFT_LIMIT_MIN
            || armAngle < Arm.SOFT_LIMIT_MAX
            || armAngle > Arm.SOFT_LIMIT_MIN
        ) return SystemState.INVALID_STATE;

        
        if(armAngle > Arm.ARM_INTAKE_UNFOLDING_POSE){
            if(intakeAngle < Intake.START_DANGER_ZONE) return SystemState.ARM_DOWN_INTAKE_IN;
            if(intakeAngle > Intake.END_DANGER_ZONE) return SystemState.ARM_DOWN_INTAKE_OUT;
            return SystemState.ARM_DOWN_INTAKE_INTEFERE;
        }else{
            if(intakeAngle < Intake.START_DANGER_ZONE) return SystemState.ARM_UP_INTAKE_IN;
            if(intakeAngle > Intake.END_DANGER_ZONE) return SystemState.ARM_UP_INTAKE_OUT;
            return SystemState.ARM_UP_INTAKE_INTERFERE;
        }
    }


}
