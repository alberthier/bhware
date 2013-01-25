#!/bin/sh

for item in `ls /`
do
  if [ "$item" != "proc" -a "$item" != "tmp" -a "$item" != "lost+found" -a "$item" != "sys" ]
  then
    rm -rf /$item
  fi
done
