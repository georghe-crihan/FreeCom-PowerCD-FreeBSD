#!/bin/sh
/sbin/mount -ur /
/sbin/mount -ur /usr
/sbin/mount -ur /var/tmp
/sbin/mount -ur /var
sync
sync
sync
/sbin/kldload ./ppcd.ko
#/sbin/kldunload ppcd
if [ $# = 1 ]; then
  /sbin/mount -uw /var
  /sbin/mount -uw /var/tmp
  /sbin/mount -uw /usr
  /sbin/mount -uw /
fi;

