libhostflashdir = ${libdir}/ipmid-providers
libhostflash_LTLIBRARIES = libhostflash.la

libhostflash_la_SOURCES = hostflash.cpp

libhostflash_la_LDFLAGS = $(SYSTEMD_LIBS)\
                          $(SDBUSPLUS_LIBS) \
                          $(PHOSPHOR_LOGGING_LIBS) \
                          -version-info 0:0:0 -shared

libhostflash_la_CXXFLAGS = $(SYSTEMD_CFLAGS) \
                           $(SDBUSPLUS_CFLAGS) \
                           $(PHOSPHOR_LOGGING_CFLAGS) \
                           $(AM_CXXFLAGS)
