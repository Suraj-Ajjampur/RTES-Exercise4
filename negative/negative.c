#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

typedef double FLOAT;

typedef unsigned int UINT32;
typedef unsigned long long int UINT64;
typedef unsigned char UINT8;

// PPM Edge Enhancement Code
//
UINT8 header[1024];
UINT8 R[VRES*HRES];
UINT8 G[VRES*HRES];
UINT8 B[VRES*HRES];
UINT8 convR[VRES*HRES];
UINT8 convG[VRES*HRES];
UINT8 convB[VRES*HRES];

int main(int argc, char *argv[])
{
    int fdin, fdout, bytesRead=0, bytesLeft, i, j;
    UINT64 microsecs=0, millisecs=0;
    FLOAT temp;
    
    // bytesLeft=21;

    //printf("Reading header\n");

    int headerlen;
    int *headerlen_ptr=&headerlen;
    *headerlen_ptr=0;

    int chans, rows, cols, bufferlen;
    int *chans_ptr = &chans;
    int *rows_ptr = &rows;
    int *cols_ptr = &cols;
    int *bufferlen_ptr = &bufferlen;

    char *aline=NULL;  size_t linelen; FILE *filep;
    char magic[2]; unsigned col, row, sat, channels=3;
    int nread=0, toread=0, fd=0;


    filep=fopen(argv[1], "r");

    // read and validate header
    if((*headerlen_ptr += getline(&aline, &linelen, filep)) < 0)
        {perror("getline"); exit(-1);}
    strcat(header, aline);
    sscanf(aline, "%s", magic);
    if(strncmp(magic, "P6", 2) == 0) channels=3; else channels=1;

    // ignore comment line or print for debug
    if((*headerlen_ptr += getline(&aline, &linelen, filep)) < 0)
        {perror("getline"); exit(-1);}
    strcat(header, aline);

    if((*headerlen_ptr += getline(&aline, &linelen, filep)) < 0)
        {perror("getline"); exit(-1);}
    sscanf(aline, "%u %u", &col, &row);
    strcat(header, aline);
    *bufferlen_ptr=row*col*channels;  toread=*bufferlen_ptr;
    *rows_ptr=row, *cols_ptr=col, *chans_ptr=channels;

    if((*headerlen_ptr += getline(&aline, &linelen, filep)) < 0)
        {perror("getline"); exit(-1);}
    sscanf(aline, "%u", &sat);
    strcat(header, aline);

    fclose(filep);

    printf("%s", header);

    if(argc < 3)
    {
       printf("Usage: negative input_file.ppm output_file.ppm\n");
       exit(-1);
    }
    else
    {
        if((fdin = open(argv[1], O_RDONLY, 0644)) < 0)
        {
            printf("Error opening %s\n", argv[1]);
        }
        else
           printf("File opened successfully\n");

        if((fdout = open(argv[2], (O_RDWR | O_CREAT | O_TRUNC), 0666)) < 0)
        {
            printf("Error opening %s\n", argv[1]);
        }
        //else
        //    printf("Output file=%s opened successfully\n", "sharpen.ppm");
    }

    bytesLeft = headerlen;

    do
    {
        //printf("bytesRead=%d, bytesLeft=%d\n", bytesRead, bytesLeft);
        bytesRead=read(fdin, (void *)header, bytesLeft);
        bytesLeft -= bytesRead;
    } while(bytesLeft > 0);

    printf("header = %s\n", header); 

    // Read RGB data
    for(i=0; i<(VRES*HRES); i++)
    {
        while(read(fdin, (void *)&R[i], 1)!=1)
        ; 
        while(read(fdin, (void *)&G[i], 1)!=1)
        ;
        while(read(fdin, (void *)&B[i], 1)!=1)
        ; 
    }

    // Skip first and last row, no neighbors to convolve with
    for(i=0; i<(VRES*HRES); i++)
    {
            convR[i] = (UINT8)(255-R[i]);
            convG[i] = (UINT8)(255-G[i]);
            convB[i] = (UINT8)(255-B[i]);
    }

    int bytesToWrite=headerlen;
    int bytesWritten;
    do
    {
        bytesWritten = write(fdout, (void *)header, bytesToWrite);
        bytesToWrite-=bytesWritten;
    } while (bytesToWrite>0);

    // Write RGB data
    for(i=0; i<(VRES*HRES); i++)
    {
        while(write(fdout, (void *)&convR[i], 1)!=1)
        ;
        while(write(fdout, (void *)&convG[i], 1)!=1)
        ;
        while(write(fdout, (void *)&convB[i], 1)!=1)
        ;
    }

    close(fdin);
    close(fdout);
 
}