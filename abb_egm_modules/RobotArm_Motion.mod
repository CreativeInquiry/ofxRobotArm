MODULE RobotArm_Motion
    !***********************************************************
    !
    ! RAPID EGM Module
    !
    ! Copyright (c) 2018, ATONATON, LLC. All rights reserved. 
    !
    ! @authors ATONATON | Madeline Gannon-//-Kevyn McPhail | www.atonaton.com
    ! 08.26.2018
    !
    !***********************************************************

    PROC Main()
        !ROBOT 0 | 1200-339
        VAR num option;
        TPReadFK option,"What would you like to do?","Send Robot to Home Position","Run EGM",stEmpty,stEmpty,"Cancel";
        TEST option
        CASE 1:
            TPWrite "Sending Robot Home...";
            MoveAbsJ startPosition,v100,fine,tool0;
            
        CASE 2:
            MoveAbsJ startPosition,v100,fine,tool0;
            WHILE TRUE DO
                runEGM;
            ENDWHILE
        CASE 5:
            RETURN ;
        ENDTEST
    ENDPROC

    !EGM Procedure
    PROC runEGM()
        EGMReset RobotArm;
        EGMGetId RobotArm;
        state:=EGMGetState(RobotArm);
        !TPWrite "EGM state: "\Num:=egmSt1;
        IF state<=EGM_STATE_CONNECTED THEN
            TPWrite "EGM State: NOT CONNECTED... Setting up EGM.";
            !Setup EGM | Device: "EGMsensor" | Configuration: "default"
            EGMSetupUC ROB_1,RobotArm,"gains_5","EGMsensor:"\Joint\CommTimeout:=60;
            state:=EGMGetState(RobotArm);
            IF state = EGM_STATE_CONNECTED THEN
                TPWrite "EGM State: CONNECTED... Hold on to your butts!";
            ENDIF
        ENDIF

        ! Correction frame is the World coordinate system and the sensor measurements are relative
        ! to the tool frame of the used tool (tFroniusCMT)
        EGMActJoint RobotArm\J1:=jointLimit\J2:=jointLimit
                         \J3:=jointLimit\J4:=jointLimit
                         \J5:=jointLimit\J6:=jointLimit
                         \LpFilter:=3\SampleRate:=24\MaxSpeedDeviation:=100;

        ! Run: the convergence condition has to be fulfilled during 2 seconds before RAPID
        ! executeion continues to the next instruction
        EGMRunJoint RobotArm,EGM_STOP_HOLD\J1\J2\J3\J4\J5\J6\CondTime:=20\RampInTime:=2.0;
        state:=EGMGetState(RobotArm);

        IF state=EGM_STATE_CONNECTED THEN
            TPWrite "Convergence condition fulfilled.. Resetting EGM.";
            EGMReset RobotArm;
        ENDIF
    ENDPROC
    
ENDMODULE