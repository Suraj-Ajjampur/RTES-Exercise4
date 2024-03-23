/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 320
#define VRES 240
#define HRES_STR "320"
#define VRES_STR "240"

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};

static char            *dev_name;
//static enum io_method   io = IO_METHOD_USERPTR;
//static enum io_method   io = IO_METHOD_READ;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 30;

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

/**
 * @brief Executes an ioctl command with retry mechanism.
 *
 * This function is a wrapper for the ioctl system call, providing a
 * mechanism to retry the call if it was interrupted by a signal (EINTR).
 * It is commonly used for device-specific or driver-specific calls
 * that require a robust mechanism for handling interruptions.
 *
 * @param fh The file descriptor of the device on which the operation is to be performed.
 * @param request The device-dependent request code.
 * @param arg Pointer to a memory area that is relevant for the request.
 *            The exact type and content depend on the request code.
 *
 * @return On success, returns the return value of the ioctl call, which is usually
 *         non-negative. On error, -1 is returned, and errno is set appropriately
 *         by the ioctl system call to indicate the error.
 * 
 * @ref 1. IOCTL - AESD Lecture Slides
 * 
 */
static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno); // Retry if call was interrupted

        return r;
}

char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="test00000000.ppm";

/**
 * @brief Dumps image data to a PPM file with timestamp and resolution in the header.
 *
 * This function generates a PPM file for the provided image data. It creates a unique filename
 * based on a tag, writes a header including a timestamp and image resolution, and then writes
 * the image data itself. The PPM format is chosen for its simplicity, supporting easy image data
 * dumps without needing complex encoding.
 *
 * @param p Pointer to the image data to be dumped.
 * @param size Size of the image data in bytes.
 * @param tag An unsigned integer used to generate a unique filename.
 * @param time A pointer to a timespec structure containing the timestamp to be included in the header.
 */
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, total, dumpfd;
   
    // Create filename using the tag.
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    // Open or create the file with write permissions.
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    // Prepare the PPM header including the timestamp and resolution.
    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // Write the header to the file, excluding the null terminator.
    written = write(dumpfd, ppm_header, sizeof(ppm_header)-1);

    total = 0;

    // Write the image data to the file in chunks until all data is written.
    do
    {
        written = write(dumpfd, p, size);
        total += written;
    } while(total < size);

    // Log the total bytes written to the file.
    printf("wrote %d bytes\n", total);

    // Close the file descriptor.
    close(dumpfd);
}



char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[]="test00000000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[4], 9, "%08d", tag);
    strncat(&pgm_dumpname[12], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 because sizeof for string includes null terminator
    written=write(dumpfd, pgm_header, sizeof(pgm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}


void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

/**
 * @brief Converts a single YUV pixel to RGB format.
 *
 * This function converts YUV pixel values to RGB using integer math for efficiency,
 * avoiding floating point operations. The conversion uses the BT.601 standard, which
 * is commonly used for standard-definition television. The input Y, U, and V values 
 * are assumed to be in the range of Y [16, 235] and U, V [16, 240] according to the 
 * standard. The function also ensures that the resulting RGB values are clipped to 
 * the range [0, 255] to be valid for display.
 *
 * @param y The luminance component of the pixel.
 * @param u The chrominance component representing blue projection.
 * @param v The chrominance component representing red projection.
 * @param r Pointer to the variable where the computed red value will be stored.
 * @param g Pointer to the variable where the computed green value will be stored.
 * @param b Pointer to the variable where the computed blue value will be stored.
 */
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}



unsigned int framecnt=0;
unsigned char bigbuffer[(1280*960)];

/**
 * @brief This function processes the captured image data based on its format. 
 * It supports different pixel formats, such as GREY, YUYV, and RGB24, and performs appropriate 
 * processing for each. The processing could range from dumping the raw data to converting YUYV to RGB 
 * or to greyscale format and then dumping it. 
 * 
 * It logs the frame processing time and increments a frame counter for each processed frame.
*/
static void process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    printf("frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("Dump graymap as-is size %d\n", size);
        dump_pgm(p, size, framecnt, &frame_time);
    }
    
    // Our code executes this line to simply process the luminasance component 
    // as V4L2_PIX_FMT_YUYV is what is set as in as the pixel format in init_device
    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT)
        printf("Dump YUYV converted to RGB size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        // // YUYV pixels are processed in groups of 4 bytes.
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        { 
            // Extract YUV components.
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            // Convert the first YUV set to RGB and store in bigbuffer.
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            // Convert the second YUV set to RGB and store next to the first RGB set.(lower res)
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }
        // Dump the RGB data as a PPM image.
        dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
#else
        printf("Dump YUYV converted to YY size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
        {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=pptr[i];
            bigbuffer[newi+1]=pptr[i+2];
        }

        dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
#endif

    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("Dump RGB as-is size %d\n", size);
        dump_ppm(p, size, framecnt, &frame_time);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}

/**
 * @brief Captures a frame from the device using Memory-Mapped I/O.
 *
 * This function handles the frame capture for devices configured to use the 
 * Memory-Mapped (MMAP) I/O method. It dequeues a buffer from the driver, 
 * processes the captured image stored in that buffer, and then re-enqueues 
 * the buffer to capture more frames. This method allows efficient access to 
 * video frames by mapping device buffers into the application's address space.
 *
 * @return Returns 1 on successfully processing a frame, and 0 if no frame is 
 *         available (EAGAIN) or on encountering EIO, which can sometimes be ignored.
 *
 */
static int read_frame(void)
{
    struct v4l2_buffer buf; 
    unsigned int i;

    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            // Dequeue a filled buffer from the driver's outgoing queue.
            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        // Buffer is not ready yet, return to try again.
                        return 0;

                    case EIO:
                        /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                           non-fatal errors too.
                         */
                        return 0;


                    default:
                        printf("mmap failure\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }
            //bound check
            assert(buf.index < n_buffers);

            // Process the image data contained in the buffer.
            process_image(buffers[buf.index].start, buf.bytesused);

            // Re-enqueue the buffer to the driver's incoming queue for capturing more data.
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }

    //printf("R");
    return 1;
}

/**
 * @brief Executes the main capture loop.
 *
 * This function continuously captures video frames up to a specified count. It uses
 * select to wait for the next frame to become available within a timeout period. Once
 * a frame is ready, it attempts to read the frame from the device. It also implements
 * a short delay after reading each frame to manage the capture pacing.
 */
static void mainloop(void)
{
    unsigned int count; // Number of frames to capture.
    struct timespec read_delay; // Delay after capturing each frame.
    struct timespec time_error; // Time error from nanosleep.

    // Set the read delay to 30 microseconds.
    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

    count = frame_count; // Initialize the frame count.

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds; // File descriptor set for select.
            struct timeval tv; // Timeout for select.
            int r; 

            FD_ZERO(&fds);
            FD_SET(fd, &fds); // Add the device file descriptor to the set.

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            // Wait for the device to become ready to read.
            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n"); // Timeout occurred.
                exit(EXIT_FAILURE);
            }

            if (read_frame()) // Try to read a frame.
            {
                // Delay after reading a frame.
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
                    printf("time_error.tv_sec=%ld, time_error.tv_nsec=%ld\n", time_error.tv_sec, time_error.tv_nsec);

                count--; // Decrement the frame counter.
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }
}

/**
 * @brief Stops the video capturing process.
 *
 * This function stops the video capture process for the device, depending on the I/O method
 * being used. For memory-mapped (MMAP) and user-pointer (USERPTR) I/O methods, it sends
 * a VIDIOC_STREAMOFF command to the device to stop the stream. For the read method,
 * there is no additional action required to stop capturing as it does not involve starting
 * or stopping a stream in the same way.
 */
static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    // Determine the action based on the I/O method.
    switch (io) {
    case IO_METHOD_READ:
        // For READ method, no action is required to stop capturing.
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        // For MMAP and USERPTR, request the device to stop the video stream.
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // Send the stream off command to the video device.
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF"); // Handle errors if the command fails.
        break;
    }
}


/**
 * @brief Starts the video capturing process.
 *
 * Depending on the I/O method in use (read, memory-mapped, or user-pointer),
 * this function prepares the device for capturing. For memory-mapped and user-pointer
 * methods, it enqueues all the buffers and then initiates video stream capture.
 * For the read method, no preparation is required before starting the capture.
 */
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        // Handles the capture start based on the I/O method.
        switch (io) 
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        // For each buffer, prepare it for capturing using memory mapping.
                for (i = 0; i < n_buffers; ++i) 
                {
                        printf("allocated buffer %d\n", i);
                        struct v4l2_buffer buf;

                        // Initialize buffer structure for enqueueing.
                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        // Enqueue the buffer.
                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                // Set the buffer type and start streaming.
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

/**
 * @brief Deallocates and cleans up resources allocated for the device based on the I/O method.
 *
 * This function is responsible for releasing any resources that were allocated during
 * the initialization and operation of the device. It handles different I/O methods
 * (READ, MMAP, USERPTR) specifically, freeing allocated buffers or unmapping memory
 * regions as appropriate. Finally, it frees the array of buffers itself.
 */
static void uninit_device(void)
{
    unsigned int i;

    // Switch between I/O methods to perform appropriate cleanup actions.
    switch (io) {
    case IO_METHOD_READ:
        // For READ method, only a single buffer was allocated, so we free it.
        free(buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        // For MMAP method, all allocated buffers need to be unmapped.
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
                // If munmap fails, exit with an error message.
                errno_exit("munmap");
        break;

    case IO_METHOD_USERPTR:
        // For USERPTR method, each buffer's start was allocated separately, so we free them individually.
        for (i = 0; i < n_buffers; ++i)
            free(buffers[i].start);
        break;
    }

    // Finally, free the array that was holding the buffers.
    free(buffers);
}


static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

/**
 * @brief Sets up memory-mapped I/O for video capture.
 *
 * Initializes video device buffers for memory-mapped I/O, ensuring efficient access
 * to video frames. It checks device capability, allocates required buffers, and maps
 * them into the application's address space.
 */
static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        // zero out the memory of the requestbuffers structure to ensure clean start.
        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        // Request buffer allocation from the video device.
        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {       
                // Check if memory mapping is not supported by the device.
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {   
                        // Handle other errors from buffer request.
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        // Ensure at least two buffers were allocated for proper operation.
        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        // Allocate memory for buffer tracking.
        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {   
                // Handle memory allocation failure.
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        // Initialize each buffer.
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;
                // Prepare buffer structure for query.
                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;
                // Query the buffer's properties.
                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                // Map the buffer into application's address space.
                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                // Validate the memory mapping operation.
                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

/**
 * @brief Initializes the video capture device.
 *
 * This function configures the video capture device based on global settings
 * and checks for the required capabilities. It queries the device capabilities
 * using VIDIOC_QUERYCAP, configures video cropping using VIDIOC_CROPCAP and
 * VIDIOC_S_CROP, and sets the video format using VIDIOC_S_FMT or VIDIOC_G_FMT.
 * It supports different I/O methods including read, memory mapping (MMAP), and
 * user pointer (USERPTR). The function also handles format forcing and
 * adjusts parameters to account for driver-specific quirks.
 *
 *.
 * 
 * @ref 1. https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/vidioc-querycap.html#c.v4l2_capability
 *      2. https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/dev-capture.html#capture
 */
static void init_device(void)
{
    struct v4l2_capability cap; // Device capabilities
    struct v4l2_cropcap cropcap; // Cropping capabilities
    struct v4l2_crop crop; // Cropping settings
    unsigned int min; // Minimum size calculations

    /* Query device capabilities by 
    *
    * All V4L2 devices support the VIDIOC_QUERYCAP ioctl. 
    * It is used to identify kernel devices compatible with this specification and to obtain information 
    * about driver and hardware capabilities. The ioctl takes a pointer to a struct v4l2_capability which is filled by the driver. 
    * When the driver is not compatible with this specification the ioctl returns an EINVAL error code.
    * 
    * @ref https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/vidioc-querycap.html#c.v4l2_capability
    */ 
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    // Ensuring the device supports video capture by checking the cap structure member changed by previous ioctl call
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    // check for which I/O method is chosen here
    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
        //for the below cases check for streaming capabilities of the driver
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }
    /* Select video input, video standard and tune here. */

    // cleares the copping structure
    CLEAR(cropcap);
    // configures for video capture buffering
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Attempt to configure cropping
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }

    // Set video format to video capture
    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        //setting the resolution to the predefined values
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    // Check for driver-specific quirks and ensure the buffer settings are valid.
    // Some drivers may not report these values correctly.
    min = fmt.fmt.pix.width * 2; // Minimum bytes per line.
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height; // Minimum size of an image buffer.
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            // For read I/O method, initialize reading (usually for simpler devices or USB cameras).
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            // For memory-mapped I/O, set up buffers for efficient data transfer.
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}


static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

/**
 * @brief Opens the video device specified by the global variable dev_name.
 * 
 * This function attempts to open the video device for reading and writing
 * with non-blocking behavior. If the device does not exist, is not accessible,
 * or is not a character device, the function reports an error and exits the
 * application.
 * 
 * @ref 1. https://man7.org/linux/man-pages/man2/open.2.html
 *      2. https://pubs.opengroup.org/onlinepubs/7908799/xsh/sysstat.h.html
 *      3. https://www.tutorialspoint.com/c_standard_library/c_function_fprintf.htm
 */
static void open_device(void)
{
        struct stat st; // Declare a structure to store information about the file/device.

        // Attempt to get information about the file/device specified by dev_name.
        if (-1 == stat(dev_name, &st)) {
            // If stat fails, print an error message with the device name, error number, and error message.
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
        // Check if the device is a character device.
        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }
        // Open the device file with read/write permissions and non-blocking mode.
        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        // Error handling if open fails
        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    open_device();
    init_device();
    start_capturing();
    mainloop();
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}
