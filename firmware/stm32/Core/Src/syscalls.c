/*
 * Minimal newlib system-call stubs.
 *
 * Only snprintf/vsnprintf are used (no stdio streams), so these are never
 * called; defining them replaces the nosys.specs versions and their
 * "is not implemented and will always fail" link warnings.
 */

#include <errno.h>
#include <sys/stat.h>

int _close(int fd)                        { (void)fd; errno = EBADF; return -1; }
int _lseek(int fd, int off, int whence)   { (void)fd; (void)off; (void)whence; return 0; }
int _read(int fd, char *buf, int len)     { (void)fd; (void)buf; (void)len; return 0; }
int _write(int fd, const char *buf, int len) { (void)fd; (void)buf; return len; }
int _fstat(int fd, struct stat *st)       { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                       { (void)fd; return 1; }
