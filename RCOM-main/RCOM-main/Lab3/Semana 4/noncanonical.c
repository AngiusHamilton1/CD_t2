/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
// Header Values
#define H_F 0x5c
#define H_A_T 0x01
#define H_A_R 0x03
#define H_C_I_0 0x00 // N(s) = 0, in second LSB
#define H_C_I_1 0x02 // N(s) = 1, in second LSB
#define H_C_SET 0x03
#define H_C_DISC 0x0B
#define H_C_UA 0x07
#define H_C_RR_0 0x01 // N(r) = 0 in third MSB
#define H_C_RR_1 0x21 // N(r) = 1 in third MSB
#define H_C_REJ_0 0x05 // N(r) = 0 in third MSB
#define H_C_REJ_1 0x25 // N(r) = 1 in third MSB
#define H_ESC 0x5d
#define H_ESC_BCC 0x20
// TimeOut
#define TIMEOUT 3
#define MAX_N_TIMEOUT 3
// States
#define S_Start 0
#define S_INI_F 1
#define S_A 2
#define S_C 3
#define S_BCC1 4
#define S_END_F 5
//#define S_BREAK 6
#define S_DISC_UA 99
// General
#define MAX_SIZE 2050

volatile int STOP=FALSE;

// Needs testing and not in use currently
int w_ua(int *fd, unsigned char *buf)
{
    // SET flags
    buf[0] = H_F;
    buf[1] = H_A_R;
    buf[2] = H_C_UA;
    buf[3] = buf[1]^buf[2];
    buf[4] = H_F;

    // Informative prints and write
    printf("\nSending UA: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    printf("%d bytes written\n\n", write(*fd, buf, 5));

    buf[0] = 0; // To not be reused
}

void stuffing(unsigned char *buf, int *length)
{
    for(int i = 4; i < (*length)-2; i++){
        if((buf[i] == H_F) || (buf[i] == H_ESC)){
            for(int j = (*length); j > i; j--){
                buf[j] = buf[j-1];
            }
            buf[i+1] = (buf[i]^H_ESC_BCC);
            buf[i] = H_ESC;

            (*length) = (*length) + 1;
            i++;
        }
    }
}

void de_stuffing(unsigned char *buf, int *length){
    for(int i = 4; i < (*length-3); i++){
        if( (buf[i] == H_ESC) && ( (buf[i+1] == (H_F^H_ESC_BCC)) || (buf[i+1] == (H_ESC^H_ESC_BCC)) ) ){
            if(buf[i+1] == (H_F^H_ESC_BCC)){
                buf[i] = H_F;
            } else{
                buf[i] = H_ESC;
            }
            for(int j = i+1; j < (*length); j++){
                buf[j] = buf[j+1];
            }
            (*length) = (*length) - 1;
        }
    }
}

unsigned char data_bcc(unsigned char *buf, int *length)
{
    unsigned char bcc = buf[4];
    for(int i = 5; i < (*length)-2; i++){
        bcc = (bcc^buf[i]);
    }
    return bcc;
}

int main(int argc, char** argv)
{
    int fd, c, res, state = 0, ns = 0, nr = 0, frameLength = 4;
    struct termios oldtio, newtio;
    unsigned char buf[MAX_SIZE], recv[MAX_SIZE];
    buf[0] = 0; recv[0] = 0;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) &&
          (strcmp("/dev/ttyS11", argv[1])!=0)  )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = (TIMEOUT/0.1);   /* inter-character timer unused */ // Value will be multiplied by 100ms
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    printf("\nWaiting for transmission...\n\n");

    while (state != S_END_F) {       /* loop for input */
        res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
        //printf("\n Direct Read: %02x\n",buf[0]);

        if(res == 0) continue;
        switch(state){
            case S_Start:
                printf("State %d. ", state);

                if(buf[0] == H_F){
                    recv[0] = buf[0];
                    state = S_INI_F;
                    printf("Flag received. State to %d\n",state);
                }
                else{
                    printf("Flag failed. State to %d\n", state);
                }
                break;
            case S_INI_F:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_A_T){
                    recv[state] = buf[0];
                    state = S_A;
                    printf("Address received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    printf("Address failed. Flag received. State to %d\n", state);
                }
                else{
                    state = S_Start;
                    printf("Address failed. State to %d.\n", state);              
                }
                break;
            case S_A:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_C_SET){
                    recv[state] = buf[0];
                    state = S_C;
                    printf("Control received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    state = S_INI_F;
                    printf("Control failed. Flag received. State to %d\n", state);
                }
                else{
                    state = S_Start;
                    printf("Control failed. State to %d.\n", state);
                }
                break;
            case S_C:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == (H_A_T^H_C_SET)){
                    recv[state] = buf[0];
                    state = S_BCC1;
                    printf("BCC received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    state = S_INI_F;
                    printf("BCC failed. Flag received. State to %d\n", state);
                }
                else{
                    state = S_Start;
                    printf("BCC failed. State to %d.\n", state);
                }
                break;
            case S_BCC1:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_F){
                    recv[state] = buf[0];
                    state = S_END_F;
                    printf("Flag received. End of transmission.\n");
                }
                else{
                    state = S_Start;
                    printf("Flag failed. State to %d.\n", state);
                }
                break;
            default:
                printf("ERROR: Default?!");
                state = S_Start;
                break;
        }
    }

    //res = write(fd,recv,255);
    //printf("%d bytes written.\nMessage sent: %s\n", res,recv);

    printf("Received: %02x %02x %02x %02x %02x\n\n", recv[0], recv[1], recv[2], recv[3], recv[4]);

    // Sends UA
    w_ua(&fd, buf);

    // Reseting state variable
    state = S_Start;
    frameLength = 4;

    while(state != S_DISC_UA){
        while(state != S_END_F){
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            //printf("\n Direct Read: %02x\n",buf[0]);

            if(res == 0) continue;
            switch(state){
                // Same
                case S_Start:
                    printf("State %d. ", state);

                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                        printf("Flag received. State to %d\n",state);
                    }
                    else{
                        printf("Flag failed. State to %d\n", state);
                    }
                    break;
                // Same
                case S_INI_F:
                    printf("State %d. ", state);

                    printf("Received buff: ");
                    for(int i = 0; i < state; i++){
                        printf("%02x ",recv[i]);                    
                    }
                    printf(". ");

                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                        printf("Address received. State to %d\n", state);
                    }
                    else if(buf[0] == H_F){
                        printf("Address failed. Flag received. State to %d\n", state);
                    }
                    else{
                        state = S_Start;
                        printf("Address failed. State to %d.\n", state);              
                    }
                    break;
                // Different
                case S_A:
                    printf("State %d. ", state);

                    printf("Received buff: ");
                    for(int i = 0; i < state; i++){
                        printf("%02x ",recv[i]);                    
                    }
                    printf(". ");

                    if(buf[0] == H_C_I_0 || buf[0] == H_C_I_1){
                        if(buf[0] == H_C_I_0){
                            ns = 0;
                        } else{
                            ns = 1;
                        }
                        recv[2] = buf[0];
                        state = S_C;
                        printf("Control received. State to %d\n", state);
                    }
                    else if(buf[0] == H_F){
                        state = S_INI_F;
                        printf("Control failed. Flag received. State to %d\n", state);
                    }
                    else{
                        state = S_Start;
                        printf("Control failed. State to %d.\n", state);
                    }
                    break;
                // Different
                case S_C:
                    printf("State %d. ", state);

                    printf("Received buff: ");
                    for(int i = 0; i < state; i++){
                        printf("%02x ",recv[i]);                    
                    }
                    printf(". ");

                    if( (buf[0] == (H_A_T^H_C_I_0) && recv[2] == H_C_I_0) || (buf[0] == (H_A_T^H_C_I_1) && recv[2] == H_C_I_1) ){
                        recv[3] = buf[0];
                        state = S_BCC1;
                        printf("BCC received. State to %d\n", state);
                    }
                    else if(buf[0] == H_F){
                        state = S_INI_F;
                        printf("BCC failed. Flag received. State to %d\n", state);
                    }
                    else{
                        state = S_Start;
                        printf("BCC failed. State to %d.\n", state);
                    }
                    break;
                // Different
                case S_BCC1:
                    printf("State %d. ", state);

                    printf("Received buff: ");
                    for(int i = 0; i < frameLength; i++){
                        printf("%02x ",recv[i]);                    
                    }
                    printf(". ");

                    recv[frameLength] = buf[0];
                    frameLength++;
                    if(buf[0] == H_F){
                        state = S_END_F;
                        printf("Flag received. End of transmission.\n");
                    }
                    else{
                        printf("Data received. State to %d.\n", state);
                    }
                    break;
                // Same
                default:
                    printf("ERROR: Default?!");
                    state = S_Start;
                    break;
            }
        }
        printf("\n\nStuffed : ");
        for(int i = 0; i < frameLength; i++){
            printf("%02x ", recv[i]);
        }
        printf("\n\n");

        // De-stuffing
        de_stuffing(recv, &frameLength);

        printf("\nOriginal : ");
        for(int i = 0; i < frameLength; i++){
            printf("%02x ", recv[i]);
        }
        printf("\n");

        // BCC2
        // Check for errors in the data field
        unsigned char bcc2 = data_bcc(recv, &frameLength);
        if(bcc2 != recv[frameLength-2]){
            // Check if duplicate
            if(ns == nr){
                // New frame -> REJ
                printf("Data bad: REJ\n");
            } else{
                // Duplicate -> send RR
                printf("Data bad: RR\n");
            }
        } else{
            // Send RR, even if duplicate
            printf("Data good: RR\n");
        }
        return 0;

    }

    // End
    printf("\nEOF\n");

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */

    sleep( 1 + TIMEOUT );

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
