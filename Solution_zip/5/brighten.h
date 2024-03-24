

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

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
             char *file);

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
              char *file);




