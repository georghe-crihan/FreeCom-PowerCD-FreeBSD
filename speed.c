#include <stdio.h>
#include <fcntl.h>
#include <sys/errno.h>
#include <sys/cdrio.h>
#include <unistd.h>

int main(int argc, char **argv)
{
int rc = 0;
int fd;
int speed = 1 * 177;
char *tag = "ok";

fd = open(argc > 1 ? argv[1] : "/dev/cd0c", O_WRONLY);
if (fd == -1) {
  rc = -1;
  tag = "open";
  goto done;
  }

  rc = ioctl(fd, CDRIOCREADSPEED, &speed);
  tag = "ioctl";

done: 
  if (rc == -1)
    perror(tag);
  close(fd);
  
return rc;
}
