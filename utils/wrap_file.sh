#!/bin/bash
printf "R\"=====(" > $2
cat $1 >> $2
printf ")=====\"" >> $2
