#! /bin/bash
# Here we setup the environment
# variables needed by the tinyos 
# make system

echo "Setting up for TinyOS 2.1.1"
export TOSROOT=
export TOSDIR=
export CLASSPATH=
export PYTHONPATH=
export MAKERULES=

#TOSROOT=$(echo ~/TinyOS-STM)
TOSROOT=/mnt/shared/TinyOS-STM-v0.9b
TOSDIR="$TOSROOT/tos"
CLASSPATH=$CLASSPATH:$TOSROOT/support/sdk/java
PYTHONPATH=.:$TOSROOT/support/sdk/python:$PYTHONPATH
MAKERULES="$TOSROOT/support/make/Makerules"

alias cda="cd $TOSROOT/apps"
alias chip="cd $TOSROOT/tos/chips/stm32"
alias cdp="cd $TOSROOT/tos/platforms/emote"
export TOSROOT
export TOSDIR
export CLASSPATH
export PYTHONPATH
export MAKERULES

