// file name = sprig_audio.c
// (c) 2017 JH1OOD/Mike
// % gcc -Wall -std=c99 sprig_audio.c -o sprig_audio -lm -lasound -lfftw3

#define NFFT     2048
#define NBFO        2
#define BAUDRATE B19200
#define M_PI     3.141592653589793

#include "asoundlib.h"
#include <fftw3.h>
#include <alloca.h>
#include <complex.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static char myrig  [256]  = "/dev/ttyUSB0"; /* IC-7410 control */
static char device [256]  = "hw:1,0";       /* capture  device (IC-7410) */
static char device2[256]  = "hw:0,0";       /* playback device (Raspberry) */

static unsigned int myrate       [2] = { 16000,  16000};    /* in Hz */
static unsigned int mychannels   [2] = {     1,      2};    /* number of channels */
static unsigned int mybuffer_time[2] = {512000, 512000};    /* in uS */
static unsigned int myperiod_time[2] = {128000, 128000};    /* in uS */
static snd_pcm_sframes_t mybuffer_size[2]; // 16kHz*512uS = 8192 frames
static snd_pcm_sframes_t myperiod_size[2]; // 16kHz*128uS = 2048 frames (= NFFT)

static int byte_per_sample = 2; /* 16 bit format */

int lring = 16384;
static signed short *ringbuffer; // 0x4000
static int wpnt =         0;     // 0x0000
static int rpnt = 16384 / 2;     // 0x2000

static double complex ringbuffer2[151];
static double complex ringbuffer3[151];
static int wpnt2 = 0;

static double audio_signal      [NFFT];
static double fft_window        [NFFT];
static int nbyte, nbyte2;
static double bin_size;
static double amax = 13.0, amin = 7.0;

double bfo_freq [NBFO] = { 1000.0, 2000.0};
double bfo_phase[NBFO] = {    0.0,    0.0};
double bfo_delta[NBFO];

double cw_freq  [NBFO] = {  400.0,  700.0};
double cw_phase [NBFO] = {    0.0,    0.0};
double cw_delta [NBFO];

const int nhilbert = 51; 
const double normalization = 1.0028050845;
double hilbert_coeff[51] = { 
                -1.244274239685367e-02 ,        //  0
                +8.812508503481302e-16 ,        //  1
                -9.007745694645722e-03 ,        //  2
                -4.892921491956048e-16 ,        //  3
                -1.224827000343725e-02 ,        //  4
                -2.457355030119336e-16 ,        //  5
                -1.626770260654829e-02 ,        //  5
                -1.076580799233997e-15 ,        //  7
                -2.126259131785366e-02 ,        //  8
                +1.551646837953304e-15 ,        //  9
                -2.754013546165147e-02 ,        // 10
                -8.271548913440781e-16 ,        // 11
                -3.555144114893793e-02 ,        // 12
                +1.538194937222543e-16 ,        // 13
                -4.616069183085086e-02 ,        // 14
                -2.517547480270933e-16 ,        // 15
                -6.087912031697079e-02 ,        // 16
                +1.529522013663762e-16 ,        // 17
                -8.310832605502005e-02 ,        // 18
                -7.153673299041007e-16 ,        // 19
                -1.216347994939241e-01 ,        // 20
                -3.571474290448791e-17 ,        // 21
                -2.087518418318809e-01 ,        // 22
                -3.409056833722323e-16 ,        // 23
                -6.354638176591967e-01 };       // 24

int    ntap = 151;

double filter_coeff2[151] = { 
                -3.395305452627101e-04 ,
                -3.451719723840665e-04 ,
                -3.530612137292319e-04 ,
                -3.631649177352709e-04 ,
                -3.753639928353780e-04 ,
                -3.894505086974910e-04 ,
                -4.051255048204917e-04 ,
                -4.219977489075693e-04 ,
                -4.395834787202738e-04 ,
                -4.573071519606507e-04 ,
                -4.745032192165208e-04 ,
                -4.904189252244857e-04 ,
                -5.042181337480122e-04 ,
                -5.149861613279066e-04 ,
                -5.217355951351015e-04 ,
                -5.234130602372101e-04 ,
                -5.189068918766871e-04 ,
                -5.070556589445432e-04 ,
                -4.866574758121344e-04 ,
                -4.564800311443923e-04 ,
                -4.152712543468728e-04 ,
                -3.617705329774267e-04 ,
                -2.947203878567924e-04 ,
                -2.128785068104263e-04 ,
                -1.150300330287973e-04 ,
                -6.347328586093010e-19 ,
                1.333341981065508e-04 ,
                2.860304138055082e-04 ,
                4.590695456642169e-04 ,
                6.533436439384927e-04 ,
                8.696444356953537e-04 ,
                1.108652380544769e-03 ,
                1.370926364961174e-03 ,
                1.656894139010764e-03 ,
                1.966843594141855e-03 ,
                2.300914974582423e-03 ,
                2.659094107869474e-03 ,
                3.041206732166469e-03 ,
                3.446913989373043e-03 ,
                3.875709143669074e-03 ,
                4.326915575142740e-03 ,
                4.799686087616610e-03 ,
                5.293003558798781e-03 ,
                5.805682949544904e-03 ,
                6.336374677422204e-03 ,
                6.883569348021978e-03 ,
                7.445603825678312e-03 ,
                8.020668613524446e-03 ,
                8.606816501260849e-03 ,
                9.201972427726550e-03 ,
                9.803944494460752e-03 ,
                1.041043605601629e-02 ,
                1.101905880293651e-02 ,
                1.162734674412459e-02 ,
                1.223277098690524e-02 ,
                1.283275520548310e-02 ,
                1.342469168181124e-02 ,
                1.400595779716308e-02 ,
                1.457393284800538e-02 ,
                1.512601505614670e-02 ,
                1.565963864062149e-02 ,
                1.617229081739082e-02 ,
                1.666152859271505e-02 ,
                1.712499521698524e-02 ,
                1.756043616788736e-02 ,
                1.796571453499896e-02 ,
                1.833882568225810e-02 ,
                1.867791107016340e-02 ,
                1.898127112601672e-02 ,
                1.924737705795075e-02 ,
                1.947488151683051e-02 ,
                1.966262801930648e-02 ,
                1.980965905524953e-02 ,
                1.991522281342583e-02 ,
                1.997877847048119e-02 ,
                2.000000000000000e-02 ,
                1.997877847048119e-02 ,
                1.991522281342583e-02 ,
                1.980965905524953e-02 ,
                1.966262801930648e-02 ,
                1.947488151683051e-02 ,
                1.924737705795075e-02 ,
                1.898127112601672e-02 ,
                1.867791107016340e-02 ,
                1.833882568225810e-02 ,
                1.796571453499896e-02 ,
                1.756043616788736e-02 ,
                1.712499521698524e-02 ,
                1.666152859271505e-02 ,
                1.617229081739082e-02 ,
                1.565963864062149e-02 ,
                1.512601505614670e-02 ,
                1.457393284800538e-02 ,
                1.400595779716308e-02 ,
                1.342469168181124e-02 ,
                1.283275520548310e-02 ,
                1.223277098690524e-02 ,
                1.162734674412459e-02 ,
                1.101905880293651e-02 ,
                1.041043605601629e-02 ,
                9.803944494460752e-03 ,
                9.201972427726550e-03 ,
                8.606816501260849e-03 ,
                8.020668613524446e-03 ,
                7.445603825678312e-03 ,
                6.883569348021978e-03 ,
                6.336374677422204e-03 ,
                5.805682949544904e-03 ,
                5.293003558798781e-03 ,
                4.799686087616610e-03 ,
                4.326915575142740e-03 ,
                3.875709143669074e-03 ,
                3.446913989373043e-03 ,
                3.041206732166469e-03 ,
                2.659094107869474e-03 ,
                2.300914974582423e-03 ,
                1.966843594141855e-03 ,
                1.656894139010764e-03 ,
                1.370926364961174e-03 ,
                1.108652380544769e-03 ,
                8.696444356953537e-04 ,
                6.533436439384927e-04 ,
                4.590695456642169e-04 ,
                2.860304138055082e-04 ,
                1.333341981065508e-04 ,
                -6.347328586093010e-19 ,
                -1.150300330287973e-04 ,
                -2.128785068104263e-04 ,
                -2.947203878567924e-04 ,
                -3.617705329774267e-04 ,
                -4.152712543468728e-04 ,
                -4.564800311443923e-04 ,
                -4.866574758121344e-04 ,
                -5.070556589445432e-04 ,
                -5.189068918766871e-04 ,
                -5.234130602372101e-04 ,
                -5.217355951351015e-04 ,
                -5.149861613279066e-04 ,
                -5.042181337480122e-04 ,
                -4.904189252244857e-04 ,
                -4.745032192165208e-04 ,
                -4.573071519606507e-04 ,
                -4.395834787202738e-04 ,
                -4.219977489075693e-04 ,
                -4.051255048204917e-04 ,
                -3.894505086974910e-04 ,
                -3.753639928353780e-04 ,
                -3.631649177352709e-04 ,
                -3.530612137292319e-04 ,
                -3.451719723840665e-04 ,
                -3.395305452627101e-04 
};

int fd = -1;
double *in;
fftw_complex *out;
fftw_plan p;

static int myset_hwparams(snd_pcm_t *handle, snd_pcm_hw_params_t *params, int id) {
  snd_pcm_uframes_t size;
  int err, dir;

  fprintf(stderr, "myset_hwparams: device id = %2d \n", id);

  err = snd_pcm_hw_params_any(handle, params);
  if (err < 0) {
    fprintf(stderr, "Broken configuration for playback: no configurations available: %s\n",
        snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_set_rate_resample(handle, params, 0); // 0: no resampling
  if (err < 0) {
    fprintf(stderr, "Resampling setup failed for playback: %s\n", snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (err < 0) {
    fprintf(stderr, "Access type not available for playback: %s\n", snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16);
  if (err < 0) {
    fprintf(stderr, "Sample format not available for playback: %s\n", snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_set_channels(handle, params, mychannels[id]);
  if (err < 0) {
    fprintf(stderr, "Channels count (%i) not available for the device: %s\n", mychannels[id],
           snd_strerror(err));
    return err;
  }

  unsigned int rrate;
  rrate = myrate[id];
  err   = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
  if (err < 0) {
    fprintf(stderr, "Rate %iHz not available for playback: %s\n", myrate[id], snd_strerror(err));
    return err;
  }
  if (rrate != myrate[id]) {
    fprintf(stderr, "set_hwparams2: Rate doesn't match (requested %iHz, get %iHz)\n", myrate[id], err);
    return -EINVAL;
  }

  err = snd_pcm_hw_params_set_buffer_time_near(handle, params, &mybuffer_time[id], &dir);
  if (err < 0) {
    fprintf(stderr, "Unable to set buffer time %i for the device: %s\n", mybuffer_time[id], snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_get_buffer_size(params, &size);
  if (err < 0) {
    fprintf(stderr, "Unable to get buffer size for playback: %s\n", snd_strerror(err));
    return err;
  }
  mybuffer_size[id] = size;

  err = snd_pcm_hw_params_set_period_time_near(handle, params, &myperiod_time[id], &dir);
  if (err < 0) {
    fprintf(stderr, "Unable to set period time %i for playback: %s\n", myperiod_time[id], snd_strerror(err));
    return err;
  }

  err = snd_pcm_hw_params_get_period_size(params, &size, &dir);
  if (err < 0) {
    fprintf(stderr, "Unable to get period size for playback: %s\n", snd_strerror(err));
    return err;
  }
  myperiod_size[id] = size;

  err = snd_pcm_hw_params(handle, params);
  if (err < 0) {
    fprintf(stderr, "Unable to set hw params for playback: %s\n", snd_strerror(err));
    return err;
  }

  fprintf(stderr, " obtained rate = %8d  \n", rrate);
  fprintf(stderr, " mybuffer_size = %8ld \n", mybuffer_size[id]);
  fprintf(stderr, " myperiod_size = %8ld \n", myperiod_size[id]);

  return 0;
}

// ---------------------------------------------------------------------
static int myset_swparams(snd_pcm_t *handle, snd_pcm_sw_params_t *swparams, int id) {
  int err;
  fprintf(stderr, "myset_swparams: device id = %2d \n", id);

  err = snd_pcm_sw_params_current(handle, swparams);
  if (err < 0) {
    fprintf(stderr, "Unable to determine current swparams for playback: %s\n",
           snd_strerror(err));
    return err;
  }

  err = snd_pcm_sw_params_set_start_threshold(
      handle, swparams, (mybuffer_size[id] / myperiod_size[id]) * myperiod_size[id]);
  if (err < 0) {
    fprintf(stderr, "Unable to set start threshold mode for playback: %s\n",
           snd_strerror(err));
    return err;
  }

  err = snd_pcm_sw_params_set_avail_min(handle, swparams, myperiod_size[id]);
  if (err < 0) {
    fprintf(stderr, "Unable to set avail min for playback: %s\n", snd_strerror(err));
    return err;
  }

  err = snd_pcm_sw_params(handle, swparams);
  if (err < 0) {
    fprintf(stderr, "Unable to set sw params for playback: %s\n", snd_strerror(err));
    return err;
  }
  return 0;
}

// ---------------------------------------------------------------------
static void async_callback2(snd_async_handler_t *ahandler) {
  snd_pcm_t *handle     = snd_async_handler_get_pcm(ahandler);
  signed short *samples = snd_async_handler_get_callback_private(ahandler);
  snd_pcm_sframes_t avail;
  int err;
  static int icount = 0;
  signed char mychar[4096];

  avail = snd_pcm_avail_update(handle);

  while (avail >= myperiod_size[1]) {
    err = snd_pcm_writei(handle, samples, myperiod_size[1]);

    if (err < 0) {
      fprintf(stderr, "Write error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }

    if (err != myperiod_size[1]) {
      fprintf(stderr, "Write error: written %i expected %li\n", err, myperiod_size[1]);
      exit(EXIT_FAILURE);
    }

    if(0) fprintf( stderr, "async_callback2: icount = %8d, avail = %12ld, period_size = %12ld \n",
        icount++, avail, myperiod_size[1]);

    for (int i = 0; i < myperiod_size[1]; i++) { // also NFFT
      audio_signal[i] = ringbuffer[rpnt];

      double val2 = 0.0; // hilbert filter output
      int index = 0;
      for (int j=0;j<nhilbert;j++) {
        index = rpnt - j;
        if (index < 0) {
          index += 16384;
        }
          val2 += ringbuffer[index] * hilbert_coeff[j];
      }

      int idelayed = rpnt - (nhilbert-1)/2;
      if (idelayed < 0) idelayed += 16834;

      ringbuffer2[wpnt2] = (ringbuffer[idelayed] + I * val2)
                         * (cos(bfo_phase[0])    - I * sin(bfo_phase[0]));

      ringbuffer3[wpnt2] = (ringbuffer[idelayed] + I * val2)
                         * (cos(bfo_phase[1])    - I * sin(bfo_phase[1]));

      double complex val3 = 0.0;
      double complex val4 = 0.0;
      int index3 = 0;
      for(int j=0;j<ntap;j++) {
        index3 = wpnt2 - j;
        if(index3 < 0) index3 += ntap;
        val3 += ringbuffer2[index3] * filter_coeff2[j];
        val4 += ringbuffer3[index3] * filter_coeff2[j];
      }
      val3 *= cos(cw_phase [0]) + I * sin(cw_phase [0]);
      val4 *= cos(cw_phase [1]) + I * sin(cw_phase [1]);

      samples[2 * i]     = (signed short) creal(val3);
      samples[2 * i + 1] = (signed short) creal(val4);
      mychar [i]         = ( (signed short) creal(val3 + val4) ) >> 8;

      rpnt++;
      rpnt &= 0x3fff;

      wpnt2++;
      if(wpnt2 == ntap) wpnt2 = 0;

      for(int j=0;j<NBFO;j++) {
        bfo_phase[j] += bfo_delta[j];
        cw_phase [j] += cw_delta [j];
      }
    }

    for (int i = 0; i < NFFT; i++) {
      in[i] = fft_window[i] * audio_signal[i];
    }

    fftw_execute(p);

    for (int i = 0; i < NFFT / 4; i++) {   // NFFT/4 = 2048/4 = 512
      double val;
      val = out[i][0] * out[i][0] + out[i][1] * out[i][1];
      if (val < pow(10.0, amin)) {
        val = 0.0;
      } else if (val > pow(10.0, amax)) {
        val = 1.0;
      } else {
        val = (log10(val) - amin) / (amax - amin);
      }
      unsigned char c = 255 * val;
      if(i<512) fprintf(stdout, "%c", c); // apple
    }
    for (int i= 0;i<2048;i++) {
      fprintf(stdout, "%c", mychar[i]); // apple
    }
    fflush(stdout);
    avail = snd_pcm_avail_update(handle);
  } // end of while
}

// ---------------------------------------------------------------------
static int async_loop2(snd_pcm_t *handle, signed short *samples) {
  snd_async_handler_t *ahandler;
  int err, count;

  err = snd_async_add_pcm_handler(&ahandler, handle, async_callback2, samples);
  if (err < 0) {
    fprintf(stderr, "Unable to register async handler\n");
    exit(EXIT_FAILURE);
  }
  for (count = 0; count < 2; count++) {
    err = snd_pcm_writei(handle, samples, myperiod_size[1]);
    if (err < 0) {
      fprintf(stderr, "Initial write error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
    if (err != myperiod_size[1]) {
      fprintf(stderr, "Initial write error: written %i expected %li\n", err,
             myperiod_size[1]);
      exit(EXIT_FAILURE);
    }
  }
  if (snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) {
    err = snd_pcm_start(handle);
    if (err < 0) {
      fprintf(stderr, "Start error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
  }
  return 0;
}

// ---------------------------------------------------------------------
static void async_callback(snd_async_handler_t *ahandler) {
  snd_pcm_t *handle     = snd_async_handler_get_pcm(ahandler);
  signed short *samples = snd_async_handler_get_callback_private(ahandler);
  snd_pcm_sframes_t avail;
  int err;
  static int icount = 0;
  static double phase = 0.0;
  double phase_delta;
  static double amp = 0.0;

  phase_delta  = 2.0 * M_PI * 2000.0 / 16000.0;
  phase_delta *= 1.0 + 0.1 * sin(2.0 * M_PI * icount / 180.0);

  avail = snd_pcm_avail_update(handle);

  while (avail >= myperiod_size[0]) {
    err = snd_pcm_readi(handle, samples, myperiod_size[0]);
    if (err < 0) {
      fprintf(stderr, "Write error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }

    if (err != myperiod_size[0]) {
      fprintf(stderr, "Write error: written %i expected %li\n", err,
              myperiod_size[0]);
      exit(EXIT_FAILURE);
    }

    int up, down;
    if( icount % 4 == 0) {
      up = 1; down = 0;
    } else if (icount % 4 == 3) {
      up = 0; down = 1;
    } else {
      up = 0; down = 0;
    }

    if(0) fprintf( stderr, "async_callback : icount = %8d, avail = %12ld, amp = %10.4f up = %3d, down = %3d \n",
        icount, avail, amp, up, down);

    for (int i = 0; i < NFFT; i++) {
      if(up == 1) {
        if(i < 20) {
         amp = 1.0-cos(M_PI*i/20.0);
        } else {
         amp = 2.0;
        }
      }
      if (down == 1) {
        if(i < 20) {
          amp = 1.0+cos(M_PI*i/20.0);
        } else {
          amp = 0.0;
        }
      }
      ringbuffer[wpnt++] = 1.0 * samples[i] + 0.0 * amp *sin (phase); phase += phase_delta;
      wpnt &= 0x3fff;
    }
    avail = snd_pcm_avail_update(handle);
  }
  icount++;
}

// ---------------------------------------------------------------------
static int async_loop(snd_pcm_t *handle, signed short *samples) {
  snd_async_handler_t *ahandler;
  int err;

  err = snd_async_add_pcm_handler(&ahandler, handle, async_callback, samples);
  if (err < 0) {
    fprintf(stderr, "Unable to register async handler\n");
    exit(EXIT_FAILURE);
  }

  if (snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) {
    err = snd_pcm_start(handle);
    if (err < 0) {
      fprintf(stderr, "Start error: %s\n", snd_strerror(err));
      exit(EXIT_FAILURE);
    }
  }
  return 0;
}

// ---------------------------------------------------------------------
void serial_init(void) {
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag     = CS8 | CLOCAL | CREAD;
  tio.c_cc[VEOL]  = 0xfd; /* IC-7410 postamble */
  tio.c_lflag     = 0;    /* non canonical mode */
  tio.c_cc[VTIME] = 0;    /* non canonical mode */
  tio.c_cc[VMIN]  = 1;    /* non canonical mode */

  tio.c_iflag = IGNPAR | ICRNL;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd, TCSANOW, &tio);
}

// ---------------------------------------------------------------------
int main(int argc, char *argv[]) {
  struct termios oldtio;
  snd_pcm_t *handle;
  snd_pcm_t *handle2;
  snd_pcm_hw_params_t *hwparams;
  snd_pcm_hw_params_t *hwparams2;
  snd_pcm_sw_params_t *swparams;
  snd_pcm_sw_params_t *swparams2;
  signed short *samples;
  signed short *samples2;
  int err;
  int id;  /* 0: capture device, 1: playback device */

  char buf4stdin[1024];
  
  for(int i=0;i<NBFO;i++) {
    bfo_delta[i] = 2.0 * M_PI * bfo_freq[i] / myrate[1];
    cw_delta [i] = 2.0 * M_PI * cw_freq [i] / myrate[1];
  }

  setvbuf(stdout, NULL, _IOFBF, 0);

  if (argc != 4) {
    fprintf(stderr, "Usage %s /dev/ttyUSB0 hw:1,0 hw:0,0 \n", argv[0]);
    fprintf(stderr,
            " try ls -l /dev/ttyUSB*; arecord -l; aplay -l to know "
            "these parameters.\n");
    return -1;
  }

  strcpy(myrig  , argv[1]);
  strcpy(device , argv[2]);
  strcpy(device2, argv[3]);
  fprintf(stderr, "serial        device is [%s] \n", myrig);
  fprintf(stderr, "audio capture device is [%s] \n", device);
  fprintf(stderr, "audio output  device is [%s] \n", device2);

  bin_size = myrate[0] / (double)NFFT;
  fprintf(stderr, "NFFT = %d, bin_size = %f [Hz] \n", NFFT, bin_size);
  for (int i = 0; i < NFFT; i++) {
    fft_window[i] = 0.54 - 0.46 * cos(2.0 * M_PI * i / (double)NFFT);
  }

  in  = malloc(sizeof(double) * NFFT);
  out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (NFFT / 2 + 1));
  p   = fftw_plan_dft_r2c_1d(NFFT, in, out, FFTW_MEASURE);

  snd_pcm_hw_params_alloca(&hwparams);
  snd_pcm_hw_params_alloca(&hwparams2);
  snd_pcm_sw_params_alloca(&swparams);
  snd_pcm_sw_params_alloca(&swparams2);

  if ((err = snd_pcm_open(&handle, device, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
    fprintf(stderr, "Audio capture device open error: %s\n", snd_strerror(err));
    return 0;
  }

  if ((err = snd_pcm_open(&handle2, device2, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
    fprintf(stderr, "Audio output device open error: %s\n", snd_strerror(err));
    return 0;
  }

  id = 0; /* capture device */
  if ((err = myset_hwparams(handle, hwparams, id)) < 0) {
    fprintf(stderr, "Setting of hwparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  if ((err = myset_swparams(handle, swparams, id)) < 0) {
    fprintf(stderr, "Setting of swparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  id = 1; /* playback device */
  if ((err = myset_hwparams(handle2, hwparams2, id)) < 0) {
    fprintf(stderr, "Setting of hwparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }
  if ((err = myset_swparams(handle2, swparams2, id)) < 0) {
    fprintf(stderr, "Setting of swparams failed: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  nbyte  = myperiod_size[0] * mychannels[0] * byte_per_sample;
  nbyte2 = myperiod_size[1] * mychannels[1] * byte_per_sample;

  samples = malloc(nbyte);
  if (samples == NULL) {
    fprintf(stderr, "cannot malloc samples \n");
    exit(EXIT_FAILURE);
  }

  samples2 = malloc(nbyte2);
  if (samples2 == NULL) {
    fprintf(stderr, "cannot malloc samples2 \n");
    exit(EXIT_FAILURE);
  }

  ringbuffer = malloc( sizeof(short) * lring);
  if (ringbuffer == NULL) {
    fprintf(stderr, "cannot malloc ringbuffer \n");
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < lring; i++) {
    ringbuffer[i] = 8096.0 * sin(i * 2.0 * 3.14 / 32.0);
  }

  for (int i = 0; i < nbyte2 / byte_per_sample; i++) {
    samples2[2 * i + 0] = 8096.0 * sin(i * 2.0 * 3.14 / 64.0);
    samples2[2 * i + 1] = 8096.0 * sin(i * 2.0 * 3.14 / 64.0);
  }

        hilbert_coeff[(nhilbert-1)/2] = 0.0;          // 25
        for (int i=((nhilbert-1)/2)+1;i<nhilbert;i++) {
                hilbert_coeff[i] = - hilbert_coeff[(nhilbert-1)-i]; // 26 <- -24 
        }   

  if ((err = async_loop(handle, samples)) < 0) {
    fprintf(stderr, "async_loop set error: %s\n", snd_strerror(err));
  }

  if ((err = async_loop2(handle2, samples2)) < 0) {
    fprintf(stderr, "async_loop2 set error: %s\n", snd_strerror(err));
  }

  fd = open(myrig, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    fprintf(stderr, "Error: can not open %s \n", myrig);
    return (-1);
  }
  tcgetattr(fd, &oldtio);
  serial_init();

  while (1) {
    fgets(buf4stdin,512,stdin);
    if(buf4stdin[0] == '0') {
        bfo_freq [0] += 100.0;
        bfo_delta[0] = 2.0 * M_PI * bfo_freq [0] / myrate[1];
        fprintf(stderr, "bfo_freq  = %f \n", bfo_freq[0]);
    }
    if(buf4stdin[0] == '1') {
        bfo_freq [0] -= 100.0;
        bfo_delta[0] = 2.0 * M_PI * bfo_freq [0] / myrate[1];
        fprintf(stderr, "bfo_freq  = %f \n", bfo_freq[0]);
    }
    if(buf4stdin[0] == '2') {
        bfo_freq [1] += 100.0;
        bfo_delta[1] = 2.0 * M_PI * bfo_freq [1] / myrate[1];
        fprintf(stderr, "bfo_freq  = %f \n", bfo_freq[1]);
    }
    if(buf4stdin[0] == '3') {
        bfo_freq [1] -= 100.0;
        bfo_delta[1] = 2.0 * M_PI * bfo_freq[1] / myrate[1];
        fprintf(stderr, "bfo_freq = %f \n", bfo_freq[1]);
    }

    sleep(1);
  }

  fftw_destroy_plan(p);
  fftw_free(in);
  fftw_free(out);

  return 0;
}
