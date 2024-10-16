/* Support files for GNU libc syscalls */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include <errno.h>
#include <unistd.h>
#include <sys/wait.h>

#include "startup/device/fsl_device_registers.h"

#define UNUSED(x)	((void)(x))

#ifdef USE_RTT
#include "SEGGER_RTT.h"
#endif

#undef errno
extern int errno;

#define MAX_STACK_SIZE 0x2000

/* start of heap space ('end' symbol defined in ld script) */
extern char end[];

caddr_t _sbrk(int incr) {
  static char *heap_end=end;
  char *prev_heap_end = heap_end;

  if (heap_end + incr > (char*)__get_MSP()) {
//		write(1, "Heap and stack collision\n", 25);
//		abort();
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

#if 0
int _open(char* path, int flags, ...) {
  UNUSED(path);
  UNUSED(flags);
  /* Pretend like we always fail */
  return -1;
}

int _close(int fd) {
  UNUSED(fd);
  return -1;
}

#ifndef USE_RTT
volatile int32_t ITM_RxBuffer=ITM_RXBUFFER_EMPTY;
#endif

int _read(int fd, char* ptr, int len) {
  UNUSED(fd);
#ifdef __DEBUG__
 #ifdef USE_RTT
  int i;
  if (fd!=0) return -1;

  for (i=0; i<len; i++) {
    ptr[i] = (char)SEGGER_RTT_WaitKey();
    if (ptr[i]=='\n') break;
  }
  ptr[i+1]=0;
  return i+1;
 #else
  int i;
  if (fd!=0) return -1;

  for (i=0; i<len; i++) {
  	while (ITM_CheckChar()!=1) ;	// busy wait
    ptr[i] = (char)ITM_ReceiveChar();
    if (ptr[i]=='\n') break;
  }
  ITM_RxBuffer=ITM_RXBUFFER_EMPTY;
  return i+1;
 #endif
#else
  UNUSED(ptr);
  UNUSED(len);
  return -1;
#endif
}

int _write(int fd, char* ptr, int len) {
  UNUSED(fd);
#ifdef __DEBUG__
 #ifdef USE_RTT
  if (fd!=1 && fd!=2) return -1;
  
  SEGGER_RTT_Write(0,ptr,len);

  return len;
 #else
  if (fd!=1 && fd!=2) return -1;
  
  for (int i=0; i<len; i++) {
    ITM_SendChar( *ptr++ );
  }
  return len;
 #endif
#else
  UNUSED(ptr);
  UNUSED(len);
  return -1;
#endif
}
#endif

int _lseek(int fd, int ptr, int dir) {
  UNUSED(fd);
  UNUSED(ptr);
  UNUSED(dir);
  return -1;
}

int _isatty(int fd) {
  UNUSED(fd);
  return 1;
}

int _fstat(int fd, struct stat* st) {
  UNUSED(fd);
  UNUSED(st);
  st->st_mode = S_IFCHR;
  return 0;
}

int _stat(char* file, struct stat* st) {
  UNUSED(file);
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(char* old, char* new) {
  UNUSED(old);
  UNUSED(new);
  errno = EMLINK;
  return -1;
}

int _unlink(char* name) {
  UNUSED(name);
  errno = ENOENT;
  return -1;
}

int _getpid(void) {
  return 1;
}

int _kill(int pid, int sig) {
  UNUSED(pid);
  UNUSED(sig);
  errno = EINVAL;
  return -1;
}

#if 0
void _exit (int status) {
  _kill(status, -1);
  while (1) {}
}
#endif

int _fork(void) {
  errno = EAGAIN;
  return -1;
}

int _wait(int* status) {
  UNUSED(status);
  errno = ECHILD;
  return -1;
}

int _execve(char* name, char** argv, char** env) {
  UNUSED(name);
  UNUSED(argv);
  UNUSED(env);
  errno = ENOMEM;
  return -1;
}

/*
 * _gettimeofday primitive (Stub function)
 * */
int _gettimeofday (struct timeval* tp, struct timezone* tzp) {
  /* Return fixed data for the timezone.  */
  UNUSED(tp);
  if (tzp) {
    tzp->tz_minuteswest = 0;
    tzp->tz_dsttime = 0;
  }
  tp->tv_sec=947846794;
  tp->tv_usec=0;
  return 0;
}

int _times(struct tms* buf) {
  UNUSED(buf);
  return -1;
}
