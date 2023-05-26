#include "linklayer.h"
#include <time.h>

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
// States
#define S_Start 0
#define S_INI_F 1
#define S_A 2
#define S_C 3
#define S_BCC1 4
#define S_END_F 5
#define S_BCC1_OTHER 6
// General
#define MAX_SIZE (MAX_PAYLOAD_SIZE*2 + 10)
// showStatistics // To be used
int totTr = 0, totReTr = 0;
int dataDupl = 0, dataRej = 0, dataSuc = 0, dataTot = 0;
int dataBytesSend = 0, dataBytesRecv = 0;
int numTimeouts = 0; // RTT_n=0, dataThroughput_n=0;
time_t startTime, endTime;
/*clock_t tempStartTime, tempEndTime;
long double Avg_RTT = 0, Avg_dataThroughput = 0;*/

int fd, timeOutCount = 0, timeOutMax;
int numSend = 0, numRecv = 0;
struct termios oldtio, newtio;

int w_supervision(int *fd, unsigned char A, unsigned char C)
{
    unsigned char buf[5];
    int res;

    // SET flags
    buf[0] = H_F;
    buf[1] = A;
    buf[2] = C;
    buf[3] = buf[1]^buf[2];
    buf[4] = H_F;

    res = write(*fd, buf, 5);

    if(res == 5){
        return 0;
    } else{
        return -1;
    }
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

void de_stuffing(unsigned char *buf, int *length)
{
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

unsigned char data_bcc(unsigned char *buf, int length)
{
    unsigned char bcc = buf[4];
    for(int i = 5; i < length-2; i++){
        bcc = (bcc^buf[i]);
    }
    return bcc;
}

int llopen(linkLayer connectionParameters)
{
    // Statistics
    time(&startTime);

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );
    if(fd < 0){
        perror(connectionParameters.serialPort); 
        return -1;
    }

    if(tcgetattr(fd, &oldtio) == -1){ /* save current port settings */
        perror("tcgetattr");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = (connectionParameters.timeOut/0.1);   /* Value will be multiplied by 100ms */
    newtio.c_cc[VMIN]     = 0;   /* non blocking read */

    tcflush(fd, TCIOFLUSH);

    if(tcsetattr(fd, TCSANOW, &newtio) == -1){
        perror("tcsetattr");
        return -1;
    }

    // Starting Est
    timeOutMax = connectionParameters.numTries;
    int state = S_Start, res = 0;
    unsigned char recv[MAX_SIZE], buf[5];
    timeOutCount = 0;

    // In Development
    if(connectionParameters.role == TRANSMITTER){    /* Transmitter == 0 */
        // Sending SET
        if(w_supervision(&fd, H_A_T, H_C_SET) != 0){
            return -1;
        }
        // Statistics
        totTr++;
        //tempStartTime = clock();

        // Waiting for UA
        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Statistics
                totTr++;
                totReTr++;
                numTimeouts++;
                // Re-sending SET
                if(w_supervision(&fd, H_A_T, H_C_SET) != 0){
                    return -1;
                }
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_UA){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        state = S_BCC1;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Statistics
        //tempEndTime = clock();
        //RTT_n++;
        //Avg_RTT += (double)(tempEndTime - tempStartTime)/(CLOCKS_PER_SEC/(1000*1000));
    } else if(connectionParameters.role == RECEIVER){    /* Receiver == 1 */
        // Waiting for SET
        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Statistics
                numTimeouts++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_SET){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        state = S_BCC1;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Sends UA
        if(w_supervision(&fd, H_A_T, H_C_UA) != 0){
            return -1;
        }
        // Statistics
        totTr++;
    }
    return 1;
}

int llwrite(char* buf, int bufSize)
{
    // Verification
    if(bufSize == 0 || buf == NULL){
        return 0;
    }

    unsigned char bufSend[MAX_SIZE];
    int frameLength = 6;

    // Contruction of buffer to send
    bufSend[0] = H_F;
    bufSend[1] = H_A_T;
    if(numSend == 0){
        bufSend[2] = H_C_I_0;
    } else{
        bufSend[2] = H_C_I_1;
    }
    bufSend[3] = (bufSend[1]^bufSend[2]);
    for(int i = 0; i < bufSize; i++){
        bufSend[4+i] = buf[i];
        frameLength++;
    }
    bufSend[frameLength-2] = data_bcc(bufSend, frameLength);
    bufSend[frameLength-1] = H_F;

    stuffing(bufSend, &frameLength);

    int state = S_Start, res = 0;
    unsigned char recv[MAX_SIZE];
    int done = FALSE;
    timeOutCount = 0;

    // Waiting for a response
    while(!done){
        // Sending or Re-Sending data
        if(write(fd, bufSend, frameLength) != frameLength){
            return -1;
        }
        // Statistics
        totTr++;
        dataTot++;
        //tempStartTime = clock();
        state = S_Start, res = 0;

        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Re-sending data
                if(write(fd, bufSend, frameLength) != frameLength){
                    return -1;
                }
                // Statistics
                totTr++;
                totReTr++;
                numTimeouts++;
                dataTot++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_REJ_0 || buf[0] == H_C_REJ_1 || buf[0] == H_C_RR_0 || buf[0] == H_C_RR_1){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        state = S_BCC1;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Statistics
        //tempEndTime = clock();
        //RTT_n++;
        //Avg_RTT += (double)(tempEndTime - tempStartTime)/(CLOCKS_PER_SEC/(1000*1000));
        // Processing the received packet
        if(numSend == 0){
            if(recv[2] == H_C_RR_1){
                done = TRUE;
                numSend = 1;
                // Statistics
                dataSuc++;
            }else if(recv[2] == H_C_REJ_0){
                timeOutCount = 0;
                state = S_Start;
                // Statistics
                dataRej++;
            } /*else if(recv[2] == H_C_RR_0){
                state = S_Start;
                // Statistics
                dataDupl++;
            }*/ else{
                timeOutCount++;
                state = S_Start;
                // Statistics
                numTimeouts++;
            }
        } else if(numSend == 1){
            if(recv[2] == H_C_RR_0){
                done = TRUE;
                numSend = 0;
                // Statistics
                dataSuc++;
            } else if(recv[2] == H_C_REJ_1){
                timeOutCount = 0;
                state = S_Start;
                // Statistics
                dataRej++;
            } /*else if(recv[2] == H_C_RR_1){
                state = S_Start;
                // Statistics
                dataDupl++;
            }*/ else{
                timeOutCount++;
                state = S_Start;
                // Statistics
                numTimeouts++;
            }
        } else{
            perror("NUMSEND");
            return -1;
        }
        // Statistics
        if(!done){
            totReTr++;
        }
    }
    // Statistics
    dataBytesSend += bufSize;
    //Avg_dataThroughput += (double)bufSize/((double)(tempEndTime - tempStartTime)/(CLOCKS_PER_SEC/(1000*1000)));
    //dataThroughput_n++;

    return bufSize;
}

int llread(char* packet)
{
    if(packet == NULL){
        return 0;
    }

    int frameLength = 4;
    int n = 0;

    int state = S_Start, res = 0;
    unsigned char recv[MAX_SIZE], buf[5];
    int done = FALSE;
    timeOutCount = 0;

    // Waiting for a response
    while(!done){
        state = S_Start, res = 0;

        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Statistics
                numTimeouts++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_I_0 || H_C_I_1 || buf[0] == H_C_SET || buf[0] == H_C_UA || buf[0] == H_C_DISC){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        frameLength = 4;
                        if(recv[2] == H_C_SET || recv[2] == H_C_UA || recv[2] == H_C_DISC){
                            state = S_BCC1_OTHER;
                        } else{
                            state = S_BCC1;
                        }
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    recv[frameLength] = buf[0];
                    frameLength++;
                    if(buf[0] == H_F){
                        state = S_END_F;
                    } break;

                case S_BCC1_OTHER:
                    if(buf[0] == H_F){
                        recv[frameLength] = buf[0];
                        frameLength++;
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Processing the received packet
        if(recv[2] == H_C_SET || recv[2] == H_C_UA || recv[2] == H_C_DISC){
            // Read 0 data, received a supervision frame instead
            if(recv[2] == H_C_SET){
                // Sends UA
                if(w_supervision(&fd, H_A_T, H_C_UA) != 0){
                    return -1;
                }
                // Statistics
                totTr++;
                totReTr++;
            }
            if(recv[2] == H_C_DISC){
                n = -1;
            }
            done = TRUE;
        } else{
            // Statistics
            dataTot++;
            de_stuffing(recv, &frameLength);
            unsigned char bcc2 = data_bcc(recv, frameLength);
            
            if(bcc2 == recv[frameLength-2]){
                if((recv[2] == H_C_I_0) && (numRecv == 0)){
                    // New good frame -> RR good
                    w_supervision(&fd, H_A_T, H_C_RR_1);
                    numRecv = 1;
                    // Prepare data
                    for(int i = 4, j = 0; i < frameLength-2; i++, j++){
                        packet[j] = recv[i];
                        n++;
                    }
                    done = TRUE;
                    // Statistics
                    totTr++;
                    dataSuc++;
                } else if((recv[2] == H_C_I_1) && (numRecv == 1)){
                    // New good frame -> RR good
                    w_supervision(&fd, H_A_T, H_C_RR_0);
                    numRecv = 0;
                    // Prepare data
                    for(int i = 4, j = 0; i < frameLength-2; i++, j++){
                        packet[j] = recv[i];
                        n++;
                    }
                    done = TRUE;
                    // Statistics
                    totTr++;
                    dataSuc++;
                } else if((recv[2] == H_C_I_0) && (numRecv == 1)){
                    // Duplicate -> RR lost
                    w_supervision(&fd, H_A_T, H_C_RR_1);
                    // Statistics
                    dataDupl++;
                } else if((recv[2] == H_C_I_1) && (numRecv == 0)){
                    // Duplicate -> RR lost
                    w_supervision(&fd, H_A_T, H_C_RR_0);
                    // Statistics
                    dataDupl++;
                } else{
                    perror("INFCONTROLFRAME OR NUMRECV");
                    return -1;
                }
            } else{ // if(bcc2 != recv[frameLength-2])
                if((recv[2] == H_C_I_0) && (numRecv == 0)){
                    // New bad frame -> REJ
                    w_supervision(&fd, H_A_T, H_C_REJ_0);
                    // Statistics
                    dataRej++;
                } else if((recv[2] == H_C_I_1) && (numRecv == 1)){
                    // New bad frame -> REJ
                    w_supervision(&fd, H_A_T, H_C_REJ_1);
                    // Statistics
                    dataRej++;
                } else if((recv[2] == H_C_I_0) && (numRecv == 1)){
                    // Duplicate -> RR lost
                    w_supervision(&fd, H_A_T, H_C_RR_1);
                    // Statistics
                    dataDupl++;
                } else if((recv[2] == H_C_I_1) && (numRecv == 0)){
                    // Duplicate -> RR lost
                    w_supervision(&fd, H_A_T, H_C_RR_0);
                    // Statistics
                    dataDupl++;
                } else{
                    perror("INFCONTROLFRAME OR NUMRECV");
                    return -1;
                }
            }
        }
        // Statistics
        if(!done){
            totReTr++;
        }
    }
    // Statistics
    if(n > 0){
        dataBytesRecv += n;
    }

    return n;
}

// Still needs statistics
int llclose(linkLayer connectionParameters, int showStatistics)
{
    // Starting DISC
    timeOutMax = connectionParameters.numTries;
    int state = S_Start, res = 0;
    unsigned char recv[MAX_SIZE], buf[5];
    timeOutCount = 0;

    // In Development
    if(connectionParameters.role == TRANSMITTER){    /* Transmitter == 0 */
        // Sending DISC
        if(w_supervision(&fd, H_A_T, H_C_DISC) != 0){
            return -1;
        }
        // Statistics
        totTr++;
        //tempStartTime = clock();

        // Waiting for DISC
        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Re-sending SET
                if(w_supervision(&fd, H_A_T, H_C_DISC) != 0){
                    return -1;
                }
                // Statistics
                totTr++;
                totReTr++;
                numTimeouts++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_R){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_DISC){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        state = S_BCC1;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Statistics
        //tempEndTime = clock();
        //Avg_RTT += (double)(tempEndTime - tempStartTime)/(CLOCKS_PER_SEC/(1000*1000));
        //RTT_n++;
        // Sending UA
        if(w_supervision(&fd, H_A_R, H_C_UA) != 0){
            return -1;
        }
        // Statistics
        totTr++;
        time(&endTime);

        // Show Statistics
        if(showStatistics == TRUE){
            printf("\n\n--------------------------------------------------------------------\n\n");
            printf("Statistics ON\n\n");
            printf("The closing transmitter recorded:\n");
            printf("-Total time: %.3lf s\n", difftime(endTime, startTime));
            printf("-Total transmissions: %d\n", totTr);
            printf("-Total re-transmissions: %d\n", totReTr);
            printf("-Total number of timeouts: %d\n", numTimeouts);
            //printf("-Average RTT: %lf s\n", Avg_RTT/RTT_n);
            printf("-About data:\n");
            printf("\tTotal transmissions sent: %d\n", dataTot);
            printf("\tTotal number of RR received: %d\n", dataSuc);
            //printf("\tTotal number of RR duplicates received: %d\n", dataDupl);
            printf("\tTotal number of REJ received: %d\n", dataRej);
            printf("\tTotal bytes sent: %d\n", dataBytesSend);
            //printf("\tAverage throughput: %lf B/ns\n", Avg_dataThroughput/dataThroughput_n);
            printf("\nOverall success rate: %.2f%%.", (float)((totTr-totReTr)*100)/totTr);
            printf("\tData success rate: %.2f%%.\n", (float)(dataSuc*100)/dataTot);
            printf("\n\n--------------------------------------------------------------------\n\n");
        }
    } else if(connectionParameters.role == RECEIVER){    /* Receiver == 1 */
        // Waiting for DISC
        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Statistics
                numTimeouts++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_T){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_DISC){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        state = S_BCC1;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Sends DISC
        if(w_supervision(&fd, H_A_R, H_C_DISC) != 0){
            return -1;
        }
        // Statistics
        totTr++;

        int state = S_Start, res = 0;
        timeOutCount = 0;

        // Waiting for UA
        while (state != S_END_F) {       /* loop for input */
            res = read(fd, buf, 1);   /* returns after 1 char have been input or timeout */
            
            // TIMEOUT
            if(res == 0){
                state = S_Start;
                timeOutCount++;
                if(timeOutCount == timeOutMax){
                    perror("TIMEOUT");
                    return -1;
                }
                // Re-sending DISC
                if(w_supervision(&fd, H_A_R, H_C_DISC) != 0){
                    return -1;
                }
                // Statistics
                totTr++;
                totReTr++;
                numTimeouts++;
            }

            switch(state){
                case S_Start:
                    if(buf[0] == H_F){
                        recv[0] = buf[0];
                        state = S_INI_F;
                    } break;

                case S_INI_F:
                    if(buf[0] == H_A_R){
                        recv[1] = buf[0];
                        state = S_A;
                    } else if(buf[0] != H_F){
                        state = S_Start;
                    } break;

                case S_A:
                    if(buf[0] == H_C_UA || buf[0] == H_C_DISC){
                        recv[2] = buf[0];
                        state = S_C;
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_C:
                    if(buf[0] == (recv[1]^recv[2])){
                        recv[3] = buf[0];
                        if(recv[2] == H_C_DISC){
                            state = S_BCC1_OTHER;
                        } else{
                            state = S_BCC1;
                        }
                    } else if(buf[0] == H_F){
                        state = S_INI_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_END_F;
                    } else{
                        state = S_Start;
                    } break;

                case S_BCC1_OTHER:
                    if(buf[0] == H_F){
                        recv[4] = buf[0];
                        state = S_Start;
                        //timeOutCount++;
                        // Re-sending DISC
                        if(w_supervision(&fd, H_A_R, H_C_DISC) != 0){
                            return -1;
                        }
                        // Statistics
                        totTr++;
                        totReTr++;
                    } else{
                        state = S_Start;
                    } break;

                default:
                    perror("STATE");
                    state = S_Start;
                    break;
            }
        }
        // Show Statistics
        time(&endTime);

        if(showStatistics == TRUE){
            printf("\n\n--------------------------------------------------------------------\n\n");
            printf("Statistics ON\n\n");
            printf("The closing receiver recorded:\n");
            printf("-Total time: %.2lf s\n", difftime(endTime, startTime));
            printf("-Total transmissions: %d\n", totTr);
            printf("-Total re-transmissions: %d\n", totReTr);
            printf("-Total number of timeouts: %d\n", numTimeouts);
            printf("-About data:\n");
            printf("\tTotal transmissions received: %d\n", dataTot);
            printf("\tTotal number of RR success sent: %d\n", dataSuc);
            printf("\tTotal number of RR duplicates sent: %d\n", dataDupl);
            printf("\tTotal number of REJ sent: %d\n", dataRej);
            printf("\tTotal bytes read: %d\n", dataBytesRecv);
            printf("\nOverall success rate: %.2f%%.", (float)((totTr-totReTr)*100)/totTr);
            printf("\tData success rate: %.2f%%.\n", (float)(dataSuc*100)/dataTot);
            printf("\n\n--------------------------------------------------------------------\n\n");
        }
    }
    // Close port
    sleep( 1 + connectionParameters.timeOut );

    if (tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
