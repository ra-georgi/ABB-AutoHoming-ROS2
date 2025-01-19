MODULE Module1
    
    VAR socketdev client_socket;
    VAR string receive_string;
    VAR rawbytes raw_data;
    
    VAR jointtarget joints;
    VAR jointtarget target;
    
    CONST jointtarget home_target:=[[-40,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]; 
    VAR num tolerance := 2;  ! Set a tolerance for comparison
    VAR bool is_reached;    
    
    PROC client_messaging()
        ! Create and connect the socket in error handlers
        joints := CJointT();    
        PackRawBytes joints.robax.rax_1, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.robax.rax_2, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.robax.rax_3, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.robax.rax_4, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.robax.rax_5, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.robax.rax_6, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        PackRawBytes joints.extax.eax_a, raw_data, (RawBytesLen(raw_data)+1) \Float4;
        client_recover;
        SocketSend client_socket \RawData := raw_data;
        !SocketReceive client_socket \Str := receive_string;
        WaitTime 1;
        SocketClose client_socket;
    ERROR
        IF ERRNO=ERR_SOCK_TIMEOUT THEN
        RETRY;
        ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
            client_recover;
        RETRY;
        ELSE
        ! No error recovery handling
        ENDIF
    ENDPROC
    
    PROC client_recover()
        SocketClose client_socket;
        SocketCreate client_socket;
        SocketConnect client_socket, "172.23.226.230", 1025 \Time:=5;
        !With Docker
        !SocketConnect client_socket, "127.0.0.1", 1025 \Time:=5;
        !SocketConnect client_socket, "10.103.142.146", 1025 \Time:=5;
    ERROR
        IF ERRNO=ERR_SOCK_TIMEOUT THEN
            RETRY;    
        ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
            RETURN;
        ELSE
            ! No error recovery handling
        ENDIF
    ENDPROC    
       
    PROC auto_home()
        VAR socketdev server_socket;
        VAR socketdev client_socket;
        VAR string receive_string;
        VAR string client_ip;
        
        VAR rawbytes raw_data_in;
        VAR jointtarget joints;
        
        VAR bool compare_J1:=FALSE;
        VAR bool compare_J2:=FALSE;
        VAR bool compare_J3:=FALSE;
        VAR bool compare_J4:=FALSE;
        VAR bool compare_J5:=FALSE;
        VAR bool compare_J6:=FALSE;
        VAR bool compare_J7:=FALSE;
        VAR num tolerance := 0.5;  ! Set a tolerance for comparison
        VAR bool DONE:=FALSE;
        
        SocketClose client_socket;
        SocketClose server_socket;
        SocketCreate server_socket;
        SocketBind server_socket, "10.103.141.81", 1025;
        SocketListen server_socket;
        SocketAccept server_socket, client_socket\ClientAddress:=client_ip;
        WHILE DONE = FALSE DO
            SocketReceive client_socket \RawData:=raw_data_in;
            ! Wait for client acknowledge
            UnpackRawBytes raw_data_in, 1, target.robax.rax_1 \Float4;
            UnpackRawBytes raw_data_in, 5, target.robax.rax_2 \Float4;
            UnpackRawBytes raw_data_in, 9, target.extax.eax_a \Float4;
            UnpackRawBytes raw_data_in, 13, target.robax.rax_3 \Float4;
            UnpackRawBytes raw_data_in, 17, target.robax.rax_4 \Float4;
            UnpackRawBytes raw_data_in, 21, target.robax.rax_5 \Float4;
            UnpackRawBytes raw_data_in, 25, target.robax.rax_6 \Float4;
            MoveAbsJ target, v200, z1, tool0;
            SocketSend client_socket \Str := "Target reached ";
            
            compare_J1 := Abs(target.robax.rax_1 - home_target.robax.rax_1) <= tolerance;
            compare_J2 := Abs(target.robax.rax_2 - home_target.robax.rax_1) <= tolerance;
            compare_J3 := Abs(target.robax.rax_3 - home_target.robax.rax_1) <= tolerance;
            compare_J4 := Abs(target.robax.rax_4 - home_target.robax.rax_1) <= tolerance;
            compare_J5 := Abs(target.robax.rax_5 - home_target.robax.rax_1) <= tolerance;
            compare_J6 := Abs(target.robax.rax_6 - home_target.robax.rax_1) <= tolerance;
            compare_J7 := Abs(target.robax.rax_1 - home_target.robax.rax_1) <= tolerance;
            IF (compare_J1) AND (compare_J2) AND (compare_J3) AND (compare_J4) AND (compare_J5) AND (compare_J6 ) AND (compare_J7) THEN
                DONE := TRUE;
            ENDIF            
            
        ENDWHILE
        SocketClose client_socket;
        SocketClose server_socket;
    ENDPROC    
        
    PROC main()
        !Add your code here
        client_messaging;
        auto_home;
    ENDPROC
    
ENDMODULE