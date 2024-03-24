/**
 * @file ppm_handling.c
 * @brief Functions for reading and writing PPM images and brightening
 */
#include "brighten.h"
/**
 * Reads a PPM image from a file.
 * 
 * @param buffer Pointer to the buffer where the image data will be stored.
 * @param bufferlen Pointer to an integer to store the length of the image data.
 * @param header Pointer to the buffer where the PPM header will be stored.
 * @param headerlen Pointer to an integer to store the length of the header.
 * @param rows Pointer to an unsigned integer to store the number of rows in the image.
 * @param cols Pointer to an unsigned integer to store the number of columns in the image.
 * @param chans Pointer to an unsigned integer to store the number of channels in the image (e.g., 3 for RGB).
 * @param file The file path of the PPM image to read.
 * 
 * Opens the specified PPM file and reads its header to determine the image dimensions and the
 * number of channels. It then reads the image data into the provided buffer. The function
 * allocates memory for the image data and header, which must be freed by the caller.
 */
void readppm(unsigned char *buffer, int *bufferlen, 
             char *header, int *headerlen,
             unsigned *rows, unsigned *cols, unsigned *chans,
             char *file)
{
    char *aline=NULL;  // Temporary string for reading lines
    size_t linelen; // Length of the line read
    FILE *filep; // File pointer for the PPM file
    char magic[2]; // Magic number to identify PPM format
    unsigned col, row, sat, channels=3; // Image dimensions and the number of color channels
    int nread=0, toread=0; // Number of bytes read and to read

    *headerlen=0; // Initialize header length

    // Open the file for reading
    filep=fopen(file, "r");

    // Read and validate the PPM header
    // PPM header consists of a magic number, image dimensions, and the maximum color value
    if((*headerlen += getline(&aline, &linelen, filep)) < 0)
    {
        perror("getline"); exit(-1);
    }
    strcat(header, aline); // Append the line read to the header
    sscanf(aline, "%s", magic); // Extract the magic number
    if(strncmp(magic, "P6", 2) == 0) channels=3; else channels=1; // Determine the number of color channels based on the magic number

    // Read and ignore comment lines, if any
    if((*headerlen += getline(&aline, &linelen, filep)) < 0)
    {
        perror("getline"); exit(-1);
    }
    strcat(header, aline); // Append the comment line to the header

    // Read the image dimensions
    if((*headerlen += getline(&aline, &linelen, filep)) < 0)
    {
        perror("getline"); exit(-1);
    }
    sscanf(aline, "%u %u", &col, &row); // Extract image width and height
    strcat(header, aline);
    *bufferlen=row*col*channels; // Calculate the total number of bytes to read
    toread=*bufferlen;
    *rows=row, *cols=col, *chans=channels; // Set the image dimensions

    // Read the maximum color value (saturation)
    if((*headerlen += getline(&aline, &linelen, filep)) < 0)
    {
        perror("getline"); exit(-1);
    }
    sscanf(aline, "%u", &sat);
    strcat(header, aline);

    // Read the actual pixel data into the buffer
    do
    {
        if((nread=fread(buffer, 1, (col*row*channels), filep)) == 0)
        { 
            if(feof(filep))
            {
                printf("completed readppm\n"); // Successfully read the entire file
                break;
            }
            else
            {
                perror("readppm"); exit(-1); // Handle read error
            }
        }

        buffer+=nread; // Move the buffer pointer forward
        toread=toread-nread; // Decrease the number of bytes to read
        printf("read %d bytes, buffer=%p, toread=%d\n", nread, buffer, toread);
    } while(toread > 0 && (!feof(filep)));

    printf("readppm done\n\n");
    fclose(filep); // Close the file
}

/**
 * Writes a PPM image to a file.
 * 
 * @param buffer Pointer to the buffer containing the image data to write.
 * @param bufferlen The length of the image data.
 * @param header Pointer to the buffer containing the PPM header.
 * @param headerlen The length of the header.
 * @param file The file path where the PPM image will be written.
 * 
 * Creates a new PPM file at the specified path and writes the provided header and image data to it.
 * The function assumes the header and image data are correctly formatted and compatible. It does
 * not perform any validation on the data or header contents.
 */
void writeppm(unsigned char *buffer, int bufferlen,
              char *header, int headerlen,
              char *file)
{
    FILE *filep; // File pointer to the output PPM file
    int nwritten=0, towrite=headerlen; // Variables to track the number of bytes written and to be written

    // Open the output file in write mode
    filep=fopen(file, "w");

    // Write the PPM header to the file
    do
    {
        if((nwritten=fwrite(header, 1, towrite, filep)) == 0)
        { 
            if(feof(filep))
            {
                printf("completed writeppm header\n"); // Successfully wrote the header
                break;
            }
            else
            {
                perror("writeppm header"); // Handle fwrite error for header
                exit(-1);
            }
        }

        header += nwritten; // Move the header pointer forward
        towrite -= nwritten; // Decrease the amount left to write
        printf("wrote %d bytes, header=%p, towrite=%d\n", nwritten, header, towrite);
    } while(towrite > 0);

    // Reset the towrite and nwritten for buffer data
    towrite = bufferlen;
    nwritten = 0;

    // Write the image data (buffer) to the file
    do
    {
        if((nwritten=fwrite(buffer, 1, towrite, filep)) == 0)
        { 
            if(feof(filep))
            {
                printf("completed writeppm\n"); // Successfully wrote the image data
                break;
            }
            else
            {
                perror("writeppm"); // Handle fwrite error for buffer
                exit(-1);
            }
        }

        buffer += nwritten; // Move the buffer pointer forward
        towrite -= nwritten; // Decrease the amount left to write
        printf("wrote %d bytes, buffer=%p, towrite=%d\n", nwritten, buffer, towrite);
    } while(towrite > 0);

    fclose(filep); // Close the file
}



#define PIXIDX ((i*col*chan)+(j*chan)+k)
#define SAT (255)

/**
 * Main program to process a PPM image.
 * 
 * @param argc The argument count.
 * @param argv The argument vector.
 * 
 * Takes a PPM file as input, applies brightness adjustment, and writes the result to a new file
 * named "brighter.ppm". The brightness adjustment parameters (alpha for gain and beta for bias)
 * are hardcoded in this example.
 */
void transform(int argc, char *argv[])
{
  char header[512];
  unsigned char img[640*480*3], newimg[640*480*3];
  int bufflen, hdrlen; unsigned row=0, col=0, chan=0, pix; int i, j, k;
  double alpha=1.25;  unsigned char beta=25;

  header[0]='\0';
  readppm(img, &bufflen, header, &hdrlen, &row, &col, &chan, argv[1]);

  for(i=0; i < row; i++)
    for(j=0; j < col; j++)
      for(k=0; k < chan; k++)
      {
          newimg[PIXIDX] = (pix=(unsigned)((img[PIXIDX])*alpha)+beta) > SAT ? SAT : pix;
      }

  writeppm(newimg, bufflen, header, hdrlen, "brighter.ppm");
}
