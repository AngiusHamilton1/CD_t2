/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>


#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

// Header Values 
#define BUF_FLAG 0x5c // flag on buffer
#define TRANS_TO_REC 0x01 //transmitier to receiver
#define REC_TO_TRANS 0x03 //receiver to transmiter
#define INF_I_0 0x00 // N(s) = 0
#define INF_I_1 0x02 // N(s) = 1
#define CONTR_SET 0x03 //Control field SET on buffer (set up)
#define CONTR_DISC 0x0B //Control field DISC on buffer(disconnect)
#define CONTR_UA 0x07 //Control field UA on buffer (unnumbered acknowledgment)
#define SUPER_RR_0 0x01 // N(r) = 0 (receiver ready )
#define SUPER_RR_1 0x21 // N(r) = 1 (receiver reject)
#define SUPER_REJ_0 0x05 // N(r) = 0 
#define SUPE 0x25 // N(r) = 1 
#define STUFF_ESC 0x5d //escape byte for stuffing
#define STUFF_BCC_SUBSTITUTE 0x20 //substitute on stuffing

// TimeOut Values
#define TIMEOUT 3 
#define MAX_N_TIMEOUT 3 //number of timeout before a retransmission

// States of State Machine
#define STA_START 0 // inicial state
#define STA_FLAG 1 //flag state
#define STA_ADRESS 2 //adress state
#define STA_CONTROL 3 // control state
#define STA_BCC 4  // BCC state
#define STA_END 5  // final state (end of transmission)

//#define S_BREAK 6 

// General
#define MAX_SIZE 2050 //max length of buffer

volatile int STOP=FALSE;

int timeout_flag = FALSE, timeout_count = 0;
//supervision frame function (slide 27)
int w_supervision(int *fd, unsigned char controlType){
    unsigned char buf[5];
    int res;

    // SET flags
    buf[0] = BUF_FLAG;
    buf[1] = TRANS_TO_REC;
    buf[2] = controlType;
    buf[3] = buf[1]^buf[2];
    buf[4] = BUF_FLAG;

    res = write(*fd, buf, 5);

    // Show us what type of control the supervision frame uses
    printf("\nSending ");
    switch (controlType)
    {
    case CONTR_SET:
        printf("SET");
        break;
    case CONTR_UA:
        printf("UA");
        break;
    case CONTR_DISC:
        printf("DISC");
        break;
    case SUPER_RR_0:
    case SUPER_RR_1:
        printf("RR");
        break;
    case SUPER_REJ_0:
    case SUPE:
        printf("REJ");
        break;
    default:
        printf("ERROR");
        break;
    }
    //printf the supervision frame
    printf(": %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    //bytes written 
    printf("%d bytes written\n\n", res);
    //check out if the buffer was wrote
    if(res == 5){
        return 0;
    } else{
        return -1;
    }
}

//function where we Establish the data on serial portal driver
int w_set(int *fd, unsigned char *buf)
{
    // SET flags
    buf[0] = BUF_FLAG;
    buf[1] = TRANS_TO_REC;
    buf[2] = CONTR_SET;
    buf[3] = buf[1]^buf[2];
    buf[4] = BUF_FLAG;

    int res = write(*fd, buf, 5);
    // Informative prints
    printf("\nSending SET: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    printf("%d bytes written\n\n", res);

    buf[0] = 0; // To not be reused

    if(res == 5){
        return 0;
    } else{
        return -1;
    }
}
//test function (angius)
/*int w_set_test_timeout(int *fd, unsigned char *buf)
{
    // SET flags
    buf[0] = BUF_FLAG;
    buf[1] = TRANS_TO_REC;
    buf[2] = CONTR_SET;
    buf[3] = 0xff; // buf[1]^buf[2];
    buf[4] = BUF_FLAG;

    // Informative prints and write
    printf("\nSending SET: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    printf("%d bytes written\n\n", write(*fd, buf, 5));

    buf[0] = 0; // To not be reused
}
*/
//function where ocurres the timeout verification
void time_out(int rig)
{
    printf("\nTIMEOUT!\n");
    timeout_flag = TRUE;
    timeout_count++; 
}
//fucntion which disarm the alarm when we dont need it
void disarm()
{
    alarm(0);
    timeout_count = 0;
}
//function where stuffing ocures (search in the buffer for the octet format of a flag or escape)
void stuffing(unsigned char *buf, int *length)
{
    for(int i = 4; i < (*length)-2; i++){
        if((buf[i] == BUF_FLAG) || (buf[i] == STUFF_ESC)){
            for(int j = (*length); j > i; j--){
                buf[j] = buf[j-1];
            }
            buf[i+1] = (buf[i]^STUFF_BCC_SUBSTITUTE);
            buf[i] = STUFF_ESC; //The generation of BCC considers only the original octets (before stuffing),
                            // even if any octet must be replaced by the escape sequence
            (*length) = (*length) + 1;
            i++;
        }
    }
}
//fuction where destuffing occures (turns de stuff version of the buffer in the original one)
void destuffing(unsigned char *buf, int *length){
    for(int i = 4; i < (*length-3); i++){
        if( (buf[i] == STUFF_ESC) && ( (buf[i+1] == (BUF_FLAG^STUFF_BCC_SUBSTITUTE)) || (buf[i+1] == (STUFF_ESC^STUFF_BCC_SUBSTITUTE)) ) ){
            if(buf[i+1] == (BUF_FLAG^STUFF_BCC_SUBSTITUTE)){
                buf[i] = BUF_FLAG;
            } else{
                buf[i] = STUFF_ESC;
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

//function where all Data Link Protocol Phases occur
int main(int argc, char** argv)
{
    int fd, c, res, state = 0, ns = 0, nr = 0, LEG = 4;;
    struct termios oldtio, newtio;
    unsigned char buf[MAX_SIZE], recv[MAX_SIZE];
    int i, sum = 0, speed = 0;
    
    buf[0] = 0; //transmission buffer
    recv[0] = 0; //receiver buffer

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) &&
          (strcmp("/dev/ttyS10", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }

    // Setting alarm handler
    (void) signal(SIGALRM, time_out);


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
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

    // Establishment
    // Sending SET
    w_set(&fd, buf);
    //w_set_test_timeout(&fd, buf); //test (angius)
    alarm(TIMEOUT);

    printf("\nWaiting for transmission...\n\n");

    // Waiting for UA
    while (state != 5) {       /* loop for input */
        if( timeout_flag == TRUE ){
            if( timeout_count == MAX_N_TIMEOUT ){
                printf("Number of TIMEOUTS exceeded!\n");
                return -1;
            }
            printf("\nRe-sending SET.\n");
            state = STA_START; //estados de transmissao
            timeout_flag = FALSE;
            w_set(&fd, buf);
            alarm(TIMEOUT);

            printf("\nWaiting for transmission...\n\n");
        }

        res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
        //printf("\n Direct Read: %02x\n",buf[0]);

        if(res == 0) continue;
        //state machine (all routes)
        switch(state){
            case STA_START:
                printf("State %d. ", state);

                if(buf[0] == BUF_FLAG){
                    recv[state] = buf[0];
                    state = STA_FLAG;
                    printf("Flag received. State to %d\n",state);
                }
                else{
                    printf("Flag failed. State to %d\n", state);
                }
                break;
            case STA_FLAG:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == REC_TO_TRANS){
                    recv[state] = buf[0];
                    state = STA_ADRESS;
                    printf("Address received. State to %d\n", state);
                }
                else if(buf[0] == BUF_FLAG){
                    printf("Address failed. Flag received. State to %d\n", state);
                }
                else{
                    state = STA_START;
                    printf("Address failed. State to %d.\n", state);              
                }
                break;
            case 2:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == CONTR_UA){
                    recv[state] = buf[0];
                    state = STA_CONTROL;
                    printf("Control received. State to %d\n", state);
                }
                else if(buf[0] == BUF_FLAG){
                    state = STA_FLAG;
                    printf("Control failed. Flag received. State to %d\n", state);
                }
                else{
                    state = STA_START;
                    printf("Control failed. State to %d.\n", state);
                }
                break;
            case STA_CONTROL:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == (REC_TO_TRANS^CONTR_UA)){
                    recv[state] = buf[0];
                    state = STA_BCC;
                    printf("BCC received. State to %d\n", state);
                }
                else if(buf[0] == BUF_FLAG){
                    state = STA_FLAG;
                    printf("BCC failed. Flag received. State to %d\n", state);
                }
                else{
                    state = STA_START;
                    printf("BCC failed. State to %d.\n", state);
                }
                break;
            case 4:
                printf("State %d. ", state);

                printf("Received buff: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == BUF_FLAG){
                    disarm(); // Disarms the alarm because we dont need it anymore (end of transmission)
                    recv[state] = buf[0];
                    state = STA_END;
                    printf("Flag received. End of transmission.\n");
                }
                else{
                    state = STA_START;
                    printf("Flag failed. State to %d.\n", state);
                }
                break;
            
            default:
                printf("ERROR: Default?!");
                state = STA_START;
                break;
        }
    }

    printf("Received: %02x %02x %02x %02x %02x\n\n", recv[0], recv[1], recv[2], recv[3], recv[4]);

    // Starting the info sending

    // Test Data Frame
    buf[0] = BUF_FLAG;
    buf[1] = TRANS_TO_REC;
    buf[2] = INF_I_0;
    buf[3] = buf[1]^buf[2];
    buf[4] = 0xb3;
    buf[5] = 0xa4;
    buf[6] = 0xb9;
    buf[7] = 0xe7;
    buf[8] = BUF_FLAG;
    buf[9] = 0x34;
    buf[10] = STUFF_ESC;
    buf[11] = 0xa2;
    buf[12] = 0xff;
    buf[13] = BUF_FLAG;
    LEG = 14;

    //buf[LEG-2] = data_bcc(buf, &LEG);

    printf("\nOriginal : ");
    for(int i = 0; i < LEG; i++){
        printf("%02x ", buf[i]);
    }

    stuffing(buf, &LEG);
    printf("\n\nStuffed : ");
    for(int i = 0; i < LEG; i++){
        printf("%02x ", buf[i]);
    }
    printf("\n\n");
    write(fd, buf, LEG);

    // Testing Stuffing
    /*
    printf("\nInitial : ");
    for(int i = 0; i < LEG; i++){
        printf("%02x ", buf[i]);
    }

    stuffing(buf, &LEG);
    printf("\n\nStuffed : ");
    for(int i = 0; i < LEG; i++){
        printf("%02x ", buf[i]);
    }
    printf("\n\n");

    destuffing(buf, &LEG);
    printf("\n\nDe-Stuffed : ");
    for(int i = 0; i < LEG; i++){
        printf("%02x ", buf[i]);
    }
    printf("\n\n");
    */
    /*
    w_supervision(&fd, CONTR_SET);
    w_supervision(&fd, CONTR_UA);
    w_supervision(&fd, CONTR_DISC);
    w_supervision(&fd, SUPER_RR_0);
    w_supervision(&fd, SUPER_RR_1);
    w_supervision(&fd, SUPER_REJ_0);
    w_supervision(&fd, SUPE);
    */
    // End
    printf("\nEOF\n");

    sleep( 1 + TIMEOUT );

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
