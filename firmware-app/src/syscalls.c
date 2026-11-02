/* No file I/O: all host traffic goes through usb_send. */
#include <errno.h>
#include <sys/stat.h>
int _close(int fd){(void)fd;errno=EBADF;return -1;}
int _read(int fd,char *p,int n){(void)fd;(void)p;(void)n;errno=EBADF;return -1;}
int _write(int fd,const char *p,int n){(void)fd;(void)p;(void)n;errno=EBADF;return -1;}
int _lseek(int fd,int offset,int whence){(void)fd;(void)offset;(void)whence;errno=ESPIPE;return -1;}
int _fstat(int fd,struct stat *s){(void)fd;s->st_mode=S_IFCHR;return 0;}
int _isatty(int fd){(void)fd;return 0;}
