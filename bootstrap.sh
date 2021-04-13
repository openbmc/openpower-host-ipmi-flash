#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2018 IBM Corp.

set -e
set -x

AUTOCONF_FILES="Makefile.in aclocal.m4 ar-lib autom4te.cache compile \
        config.guess config.h.in config.sub configure depcomp install-sh \
        ltmain.sh missing *libtool test-driver"

case $1 in
    clean)
        test -f Makefile && make maintainer-clean
        for file in ${AUTOCONF_FILES}; do
            find . -name "$file" -print0 | xargs -0 -r rm -rf
        done
        exit 0
        ;;
esac

autoreconf -vfi
# shellcheck disable=SC2016
echo 'Run "./configure ${CONFIGURE_FLAGS} && make"'
