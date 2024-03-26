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
#include <syslog.h>

#include <linux/videodev2.h>

#include <time.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT_RGB
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"
//#define HRES 320
//#define VRES 240
//#define HRES_STR "320"
//#define VRES_STR "240"

#define START_UP_FRAMES (8)
#define LAST_FRAMES (1)
#define CAPTURE_FRAMES (1800+LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)
// always ignore first 8 frames
int framecnt=-8;

unsigned char bigbuffer[(1280*960)];
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
static int              frame_count = (FRAMES_TO_ACQUIRE);

static double worst_frame_rate;
static double fstart, fnow, fstop;
static struct timespec time_now, time_start, time_stop;

struct time_measure{
    double worst_frame_rate;
    double fstart, fnow, fstop;
    struct timespec time_now, time_start, time_stop;
};

struct timespec acquisition_start, acquisition_end;
double acquisition_duration, acquisition_frame_rate, acq_total;
struct time_measure acquisition;

struct timespec transform_start, transform_end;
double transform_duration, frame_rate, trans_total;
struct time_measure transform;

struct timespec writeback_start, writeback_end;
double writeback_duration, writeback_frame_rate, write_back_total = 0;
struct time_measure write_back;


static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;/* low-level i/o */
}

char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="frames/test0000.ppm";

#define SAT (255)

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
void write_ppm(const unsigned char *transformed_data, int size, unsigned int tag, struct timespec *time) {
    int written, total, dumpfd;
    char filename[255]; // Buffer for filename
    char header[1024]; // Buffer for header

    // Start timing writeback
    clock_gettime(CLOCK_MONOTONIC, &writeback_start);

    // Format the filename and header
    snprintf(filename, sizeof(filename), "/home/suraj/RTES/RTES-Exercise4/Solution_zip/5/frames/test%04d.ppm", tag);
    snprintf(header, sizeof(header), "P6\n# timestamp %ld.%ld\n%d %d\n255\n", time->tv_sec, time->tv_nsec, HRES, VRES);

    // Open or create the file with write permissions
    dumpfd = open(filename,O_WRONLY | O_NONBLOCK | O_CREAT, 0666);
    if (dumpfd < 0) {
        syslog(LOG_ERR, "Failed to open file for writing: %s", strerror(errno));
        return;
    }

    // Write the header and the image data
    write(dumpfd, header, strlen(header));
    total = write(dumpfd, transformed_data, size);

    // End timing writeback and calculate duration
    clock_gettime(CLOCK_MONOTONIC, &writeback_end);
    writeback_duration = (writeback_end.tv_sec - writeback_start.tv_sec) + 
                         (writeback_end.tv_nsec - writeback_start.tv_nsec) / 1e9;
    writeback_frame_rate = 1.0 / writeback_duration;
    write_back_total += writeback_frame_rate;

    // Log transformation time and frame rate
    syslog(LOG_INFO, "Write back duration: %lf s, Frame rate: %lf Hz FPS, for frame %d\n", writeback_duration, writeback_frame_rate, framecnt);

    // Log the total bytes written to the file.
    syslog(LOG_INFO,"wrote %d bytes\n", total);
    // Close the file descriptor.
    // Update worst frame rate 
    if (write_back.worst_frame_rate == 0 || writeback_frame_rate < write_back.worst_frame_rate) {
        write_back.worst_frame_rate = writeback_frame_rate;
    }

    // Close the file descriptor
    close(dumpfd);
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
void process_and_transform_image(const void *p, int size, unsigned char *transformed_data) {
    int i, newi;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;
    double alpha = 1.25;
    unsigned char beta = 25;

    // Start timing processing and transformation
    clock_gettime(CLOCK_MONOTONIC, &transform_start);

    // Process YUYV to RGB and apply brightness transformation
    for (i = 0, newi = 0; i < size; i = i + 4, newi = newi + 6) {
        y_temp = pptr[i];
        u_temp = pptr[i + 1];
        y2_temp = pptr[i + 2];
        v_temp = pptr[i + 3];

        unsigned char r, g, b;
        yuv2rgb(y_temp, u_temp, v_temp, &r, &g, &b);
        // Apply brightness transformation
        transformed_data[newi] = (r * alpha) + beta > SAT ? SAT : (r * alpha) + beta;
        transformed_data[newi + 1] = (g * alpha) + beta > SAT ? SAT : (g * alpha) + beta;
        transformed_data[newi + 2] = (b * alpha) + beta > SAT ? SAT : (b * alpha) + beta;

        yuv2rgb(y2_temp, u_temp, v_temp, &r, &g, &b);
        transformed_data[newi + 3] = (r * alpha) + beta > SAT ? SAT : (r * alpha) + beta;
        transformed_data[newi + 4] = (g * alpha) + beta > SAT ? SAT : (g * alpha) + beta;
        transformed_data[newi + 5] = (b * alpha) + beta > SAT ? SAT : (b * alpha) + beta;
    }

    // End timing transformation
    clock_gettime(CLOCK_MONOTONIC, &transform_end);

    // Log transformation process
    transform_duration = (transform_end.tv_sec - transform_start.tv_sec) +
                         (transform_end.tv_nsec - transform_start.tv_nsec) / 1e9;
    frame_rate = 1.0 / transform_duration;
    trans_total +=  frame_rate;

    // Update worst frame rate 
    if (transform.worst_frame_rate == 0 || frame_rate < transform.worst_frame_rate) {
        transform.worst_frame_rate = frame_rate;
    }

    // Log transformation time and frame rate
    syslog(LOG_INFO, "Transformation duration: %lf s, Frame rate: %lf Hz FPS, for frame %d\n", transform_duration, frame_rate, framecnt);
}


void process_image(const void *p, int size) {
    struct timespec frame_time;
    unsigned char transformed_data[(1280*960)*3]; 

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    syslog(LOG_INFO,"frame %d: ", framecnt);

    // Check for the frame format and process accordingly
    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {

        // Process and transform the image (including YUYV to RGB conversion and brightness adjustment)
        process_and_transform_image(p, size, transformed_data);

        // Perform writeback
        write_ppm(transformed_data, ((size*6)/4), framecnt, &frame_time); // Make sure the size is correct based on the conversion ratio
    } else {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    fflush(stdout);
}



static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    // Start timing transformation
    clock_gettime(CLOCK_MONOTONIC, &acquisition_start);
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
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

    assert(buf.index < n_buffers);
    // End timing for acquisition
    clock_gettime(CLOCK_MONOTONIC, &acquisition_end);
    // Calculate acquisition duration and frame rate
    acquisition_duration = (acquisition_end.tv_sec - acquisition_start.tv_sec) +
                        (acquisition_end.tv_nsec - acquisition_start.tv_nsec) / 1e9;
    acquisition_frame_rate = 1.0 / acquisition_duration;
    acq_total += acquisition_frame_rate;
            

    // Update worst frame rate 
    if (acquisition.worst_frame_rate == 0 || acquisition_frame_rate < acquisition.worst_frame_rate) {
        acquisition.worst_frame_rate = acquisition_frame_rate;
    }

    // Log acquisition time and frame rate
    syslog(LOG_INFO, "Acquision duration: %lf s, Frame rate: %lf Hz FPS, for frame %d\n", acquisition_duration, acquisition_frame_rate, framecnt);

    process_image(buffers[buf.index].start, buf.bytesused);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    
    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;
    double calculated_frame_rate;

    // Replace this with a sequencer DELAY
    //
    // 250 million nsec is a 250 msec delay, for 4 fps
    // 1 sec for 1 fps
    //
    read_delay.tv_sec=0;
    read_delay.tv_nsec=10000000; //changed according to assignment requirements

    count = frame_count;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                else
                {

                    if(framecnt>1)
                    {	

                        clock_gettime(CLOCK_MONOTONIC, &time_now);
                        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;

                        calculated_frame_rate = (double)(framecnt+1) / (fnow-fstart);
                        if(framecnt==2)
                        {
                            worst_frame_rate = calculated_frame_rate;
                        }
                        else
                        {
                            if(calculated_frame_rate<worst_frame_rate)
                            {
                                worst_frame_rate = calculated_frame_rate;
                            }
                        }

                        syslog(LOG_INFO,"SIMPCAP: read at %lf, @ %lf FPS Hz\n", (fnow-fstart), calculated_frame_rate);
                    }
                    else
                    {

                    }
                }

                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;    
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
        
}

static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;
        for (i = 0; i < n_buffers; ++i) 
        {
            syslog(LOG_INFO,"allocated buffer %d\n", i);
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}

static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
    if (-1 == munmap(buffers[i].start, buffers[i].length))
            errno_exit("munmap");

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

static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
    {
        if (EINVAL == errno) 
        {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", dev_name);
                    exit(EXIT_FAILURE);
        } else 
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) 
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers) 
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
        mmap(NULL /* start anywhere */,
            buf.length,
            PROT_READ | PROT_WRITE /* required */,
            MAP_SHARED /* recommended */,
            fd, buf.m.offset);

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

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

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

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

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


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

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


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        syslog(LOG_INFO,"FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here
        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

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
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;


    init_mmap();
}


static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

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

    // initialization of V4L2
    open_device();
    init_device();
    start_capturing();

    // service loop frame read
    mainloop();

    // shutdown of frame acquisition service
    stop_capturing();

    // Calculate average fps freq
    double average_transformation_fps = trans_total /(CAPTURE_FRAMES-LAST_FRAMES) ;
    double average_writeback_fps = write_back_total /(CAPTURE_FRAMES-LAST_FRAMES) ;
    double average_aquisition_fps = acq_total/ (CAPTURE_FRAMES-LAST_FRAMES);
    // Log the total acquisition time, average FPS, and worst frame rate
    syslog(LOG_INFO, "Acquisition -- %d frames, Lowest FPS=%lf hz,  Average FPS=%lf hz\n",
        CAPTURE_FRAMES, acquisition.worst_frame_rate, average_aquisition_fps);
    syslog(LOG_INFO, "Transformation -- %d frames, %lf lowest FPS hz, Average FPS is %lf hz", 
         CAPTURE_FRAMES + 1, transform.worst_frame_rate, average_transformation_fps);
    syslog(LOG_INFO, "Write back --%d total frames, %lf lowest FPS hz, Average FPS is %lf hz", 
    CAPTURE_FRAMES + 1, write_back.worst_frame_rate, average_writeback_fps);

    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}