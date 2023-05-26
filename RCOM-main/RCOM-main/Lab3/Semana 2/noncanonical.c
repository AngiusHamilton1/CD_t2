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
#define H_F 0x5c
#define H_A_T 0x01
#define H_A_R 0x03
#define H_C_SET 0x03
#define H_C_DISC 0x0B
#define H_C_UA 0x07
#define H_C_RR 0x
#define H_C_REJ 0x

volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255], recv[255];
    buf[0] = '\0';
    recv[0] = '\0';
    int state = 0;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
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

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

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

    while (state != 5) {       /* loop for input */
        res = read(fd,buf,1);   /* returns after 5 chars have been input */
        //buf[res]=0;               /* so we can printf... */
        /*printf(":%c:%d\n", buf[0], res);
        strcat(recv, buf);
        if (buf[0]=='\0') STOP=TRUE;*/

        switch(state){
            case 0:
                printf("State %d.", state);
                if(buf[0] == H_F){
                    state = 1;
                    printf("Flag received. State to %d\n",state);
                }
                else{
                    printf("Flag failed. State to %d\n", state);
                }
                break;
            case 1:
                printf("State %d.", state);
                if(buf[0] == H_A_T){
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
                printf("State %d.", state);
                if(buf[0] == H_C_SET){
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
                printf("State %d.", state);
                if(buf[0] == H_A_T^H_C_SET){
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
                printf("State %d.", state);
                if(buf[0] == H_F){
                    state = 5;
                    printf("Flag received. End of transmission.\n");
                }
                else{
                    state = 0;
                    printf("Flag failed. State to %d.\n", state);
                }
                break;
            default:
                printf("Default?!");
                state = 0;
                break;
        }
    }

    //res = write(fd,recv,255);
    //printf("%d bytes written.\nMessage sent: %s\n", res,recv);

    buf[0] = H_F;
    buf[1] = H_A_R;
    buf[2] = H_C_UA;
    buf[3] = buf[1]^buf[2];
    buf[4] = H_F;

    printf("Sending: %c %c %c %c %c\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

    res = write(fd,buf,5);
    printf("%d bytes written\n", res);

    printf("\nEOF\n");

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */

    sleep(1);

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
