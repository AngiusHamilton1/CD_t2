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
#define H_F 0x5c
#define H_A_T 0x01
#define H_A_R 0x03
#define H_C_SET 0x03
#define H_C_DISC 0x0B
#define H_C_UA 0x07
#define H_C_RR 0x01 // Needs N(r) in 3 MSB
#define H_C_REJ 0x05 // Needs N(r) in 3 MSB
#define TIMEOUT 3
#define MAX_N_TIMEOUT 3 

volatile int STOP=FALSE;

int flag = 0, tcount = 0; 

int w_set(int *fd, unsigned char *buf)
{
    // SET flags
    buf[0] = H_F;
    buf[1] = H_A_T;
    buf[2] = H_C_SET;
    buf[3] = buf[1]^buf[2];
    buf[4] = H_F;

    // Informative prints and write
    printf("\nSending SET: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    printf("%d bytes written\n\n", write(*fd, buf, 5));

    buf[0] = 0; // To not be reused
}

int w_set_test_timeout(int *fd, unsigned char *buf)
{
    // SET flags
    buf[0] = H_F;
    buf[1] = H_A_T;
    buf[2] = H_C_SET;
    buf[3] = 0xff; // buf[1]^buf[2];
    buf[4] = H_F;

    // Informative prints and write
    printf("\nSending SET: %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    printf("%d bytes written\n\n", write(*fd, buf, 5));

    buf[0] = 0; // To not be reused
}

void time_out(int rig)
{
    printf("\nTIMEOUT!\n");
    flag = 1;
    tcount++;
}

void disarm(){
    alarm(0);
    tcount = 0;
}

int main(int argc, char** argv)
{
    int fd, c, res, state = 0;
    struct termios oldtio, newtio;
    unsigned char buf[255], recv[255];
    int i, sum = 0, speed = 0;
    buf[0] = 0; recv[0] = 0;

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
    //w_set(&fd, buf);
    w_set_test_timeout(&fd, buf);
    alarm(TIMEOUT);

    printf("\nWaiting for transmission...\n\n");

    while (state != 5) {       /* loop for input */
        if( flag == 1 ){
            if( tcount == MAX_N_TIMEOUT ){
                printf("Number of TIMEOUTS exceeded!\n");
                break;
            }
            printf("\nRe-sending SET.\n");
            state = 0;
            flag = 0;
            w_set(&fd, buf);
            alarm(TIMEOUT);
        }

        res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
        //printf("\n Direct Read: %02x\n",buf[0]);

        if(res == 0) continue;
        switch(state){
            case 0:
                printf("State %d. ", state);

                if(buf[0] == H_F){
                    recv[state] = buf[0];
                    state = 1;
                    printf("Flag received. State to %d\n",state);
                }
                else{
                    printf("Flag failed. State to %d\n", state);
                }
                break;
            case 1:
                printf("State %d. ", state);

                printf("Received: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_A_R){
                    recv[state] = buf[0];
                    state = 2;
                    printf("Address received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    printf("Address failed. Flag received. State to %d\n", state);
                }
                else if(buf[0] != H_F){
                    state = 0;
                    printf("Address failed. State to %d.\n", state);              
                }
                break;
            case 2:
                printf("State %d. ", state);

                printf("Received: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_C_UA){
                    recv[state] = buf[0];
                    state = 3;
                    printf("Control received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    state = 1;
                    printf("Control failed. Flag received. State to %d\n", state);
                }
                else{
                    state = 0;
                    printf("Control failed. State to %d.\n", state);
                }
                break;
            case 3:
                printf("State %d. ", state);

                printf("Received: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == (H_A_R^H_C_UA)){
                    recv[state] = buf[0];
                    state = 4;
                    printf("BCC received. State to %d\n", state);
                }
                else if(buf[0] == H_F){
                    state = 1;
                    printf("BCC failed. Flag received. State to %d\n", state);
                }
                else{
                    state = 0;
                    printf("BCC failed. State to %d.\n", state);
                }
                break;
            case 4:
                printf("State %d. ", state);

                printf("Received: ");
                for(int i = 0; i < state; i++){
                    printf("%02x ",recv[i]);                    
                }
                printf(". ");

                if(buf[0] == H_F){
                    disarm(); // disarm alarm
                    recv[state] = buf[0];
                    state = 5;
                    printf("Flag received. End of transmission.\n");
                }
                else{
                    state = 0;
                    printf("Flag failed. State to %d.\n", state);
                }
                break;
            default:
                printf("ERROR: Default?!");
                state = 0;
                break;
        }
    }

    printf("Received: %02x %02x %02x %02x %02x\n\n", recv[0], recv[1], recv[2], recv[3], recv[4]);

    printf("\nEOF\n");

    sleep( 1 + TIMEOUT );

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
