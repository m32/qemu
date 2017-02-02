#!/bin/bash

export CONF_OPTS=(
--audio-drv-list='sdl'
--disable-aio
--disable-brlapi
--disable-bsd-user
--disable-curses
--disable-darwin-user
--disable-kqemu
--disable-kvm
--disable-linux-user
--disable-pthread
--disable-sparse
--disable-strip
--disable-vde
--disable-vnc-sasl
--disable-vnc-tls
--disable-werror
--disable-xen

--cc=gcc
#--extra-cflags='-Wall -g -fno-omit-frame-pointer'
--target-list=arm-softmmu
)

./configure "${CONF_OPTS[@]}"
