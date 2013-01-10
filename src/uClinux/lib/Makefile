##############################################################################
#
#	Makefile -- Master makefile for all libraries.
#

.EXPORT_ALL_VARIABLES:

##############################################################################
#
# Include architecture specific build rules.
#

ifndef ROOTDIR
ROOTDIR=..
endif

UCLINUX_BUILD_LIB=1
include $(LINUX_CONFIG)
include $(CONFIG_CONFIG)
include $(ARCH_CONFIG)

##############################################################################
#
# always build these dir_
#
dir_1_y  = $(LIBCDIR)
dir_1_n  =
dir_1_   =

ifeq ($(LIBCDIR),libc)
dir_2_y += libm libcrypt_old
dir_2_n +=
dir_2_  +=
endif

#
# and build these if someone needs them
#
dir_3_$(CONFIG_LIB_FLEX)         += flex
dir_3_$(CONFIG_LIB_FLEX_FORCE)   += flex
dir_3_$(CONFIG_LIB_LIBAES)       += libaes
dir_3_$(CONFIG_LIB_LIBAES_FORCE) += libaes
dir_3_$(CONFIG_LIB_LIBDES)       += libdes
dir_3_$(CONFIG_LIB_LIBDES_FORCE) += libdes
dir_3_$(CONFIG_LIB_LIBSSL)       += libssl
dir_3_$(CONFIG_LIB_LIBSSL_FORCE) += libssl
dir_3_$(CONFIG_LIB_LIBGMP)       += libgmp
dir_3_$(CONFIG_LIB_LIBGMP_FORCE) += libgmp
dir_3_$(CONFIG_LIB_LIBG)         += libg
dir_3_$(CONFIG_LIB_LIBG_FORCE)   += libg
dir_3_$(CONFIG_LIB_LIBPAM)       += libpam
dir_3_$(CONFIG_LIB_LIBPAM_FORCE) += libpam
dir_3_$(CONFIG_LIB_LIBPCAP)      += libpcap
dir_3_$(CONFIG_LIB_LIBPCAP_FORCE)+= libpcap
dir_3_$(CONFIG_LIB_ZLIB)         += zlib
dir_3_$(CONFIG_LIB_ZLIB_FORCE)   += zlib
dir_3_$(CONFIG_LIB_LIBBZ2)       += libbzip2
dir_3_$(CONFIG_LIB_LIBBZ2_FORCE) += libbzip2
dir_3_$(CONFIG_LIB_LIBATM)       += libatm
dir_3_$(CONFIG_LIB_LIBATM_FORCE) += libatm
dir_3_$(CONFIG_LIB_LIBNET)       += Libnet
dir_3_$(CONFIG_LIB_LIBNET_FORCE) += Libnet
dir_3_$(CONFIG_LIB_LIBNETOLD)    += libnet
dir_3_$(CONFIG_LIB_LIBJPEG)      += libjpeg
dir_3_$(CONFIG_LIB_LIBJPEG_FORCE)+= libjpeg
dir_3_$(CONFIG_LIB_LIBUPNP)      += libupnp
dir_3_$(CONFIG_LIB_LIBUPNP_FORCE)+= libupnp
dir_3_$(CONFIG_LIB_STLPORT)      += STLport
dir_3_$(CONFIG_LIB_STLPORT_FORCE)+= STLport
dir_3_$(CONFIG_LIB_EXPAT)        += expat
dir_3_$(CONFIG_LIB_EXPAT_FORCE)  += expat
dir_3_$(CONFIG_LIB_LIBLDAP)      += libldap
dir_3_$(CONFIG_LIB_LIBLDAP_FORCE)+= libldap
dir_3_$(CONFIG_LIB_TINYTCL)      += ../user/tinytcl
dir_3_$(CONFIG_LIB_LIBCCMALLOC)  += libccmalloc
dir_3_$(CONFIG_LIB_ADNS_FORCE)   += adns
dir_3_$(CONFIG_LIB_ARES)         += libares
dir_3_$(CONFIG_LIB_ARES_FORCE)   += libares
dir_3_$(CONFIG_LIB_LIBIDN)       += libidn
dir_3_$(CONFIG_LIB_LIBIDN_FORCE) += libidn
dir_3_$(CONFIG_LIB_LIBCURL)      += libcurl
dir_3_$(CONFIG_LIB_LIBCURL_FORCE)+= libcurl
dir_3_$(CONFIG_LIB_LIBCRYPT_OLD) += libcrypt_old
dir_3_$(CONFIG_LIB_LIBCRYPT_OLD_FORCE) += libcrypt_old
dir_3_$(CONFIG_LIB_LIBLZO)       += lzo
dir_3_$(CONFIG_LIB_LIBLZO_FORCE) += lzo
dir_3_$(CONFIG_LIB_LIBPCRE)      += pcre
dir_3_$(CONFIG_LIB_LIBPCRE_FORCE)+= pcre
dir_3_$(CONFIG_LIB_LIBLZMA)      += liblzma
dir_3_$(CONFIG_LIB_LIBLZMA_FORCE)+= liblzma
dir_3_$(CONFIG_LIB_OSIP2)        += osip2
dir_3_$(CONFIG_LIB_LIBEFENCE)    += libefence
dir_3_$(CONFIG_LIB_LIBEFENCE_FORCE) += libefence
dir_3_$(CONFIG_LIB_TERMCAP)      += termcap
dir_3_$(CONFIG_LIB_TERMCAP_FORCE)+= termcap
dir_3_$(CONFIG_LIB_LIBSYSFS)     += sysfsutils
dir_3_$(CONFIG_LIB_LIBSYSFS_FORCE) += sysfsutils
dir_3_$(CONFIG_LIB_NFNETLINK)    += libnfnetlink
dir_4_$(CONFIG_LIB_NETFILTER_LOG)+= libnetfilter_log
dir_4_$(CONFIG_LIB_NETFILTER_CONNTRACK) += libnetfilter_conntrack
dir_4_$(CONFIG_LIB_LIBPNG)       += libpng
dir_4_$(CONFIG_LIB_LIBPNG_FORCE) += libpng

# And build libraries in the prop directory last
dir_5_y = $(ROOTDIR)/prop

DIRS_y   = $(foreach n,1 2 3 4 5,$(sort $(dir_$(n)_y)))
DIRS_all = $(foreach n,1 2 3 4 5,$(sort $(dir_$(n)_y) $(dir_$(n)_n) $(dir_$(n)_)))

##############################################################################

all:
	$(MAKE) -j1 -C ../include                 || exit $$?
	[ -z "$(dir_1_y)" ] ||\
		$(MAKE) -j$(HOST_NCPU) $(sort $(dir_1_y)) || exit $$?
	$(MAKE) -j1 -C ../include
	$(MAKE) make_links                        || exit $$?
	[ -z "$(dir_2_y)" ] ||\
		$(MAKE) -j$(HOST_NCPU) $(sort $(dir_2_y)) || exit $$?
	$(MAKE) -j1 -C ../include
	$(MAKE) make_links                        || exit $$?
	[ -z "$(dir_3_y)" ] ||\
		$(MAKE) -j$(HOST_NCPU) $(sort $(dir_3_y)) || exit $$?
	$(MAKE) -j1 -C ../include
	$(MAKE) make_links                        || exit $$?
	[ -z "$(dir_4_y)" ] ||\
		$(MAKE) -j$(HOST_NCPU) $(sort $(dir_4_y)) || exit $$?
	$(MAKE) -j1 -C ../include
	$(MAKE) make_links                        || exit $$?
	[ -z "$(dir_5_y)" ] ||\
		$(MAKE) -j$(HOST_NCPU) $(sort $(dir_5_y)) || exit $$?
	$(MAKE) -j1 -C ../include
	$(MAKE) make_links                        || exit $$?

# the actual dir target
.PHONY: $(DIRS_y)
$(DIRS_y):
	[ ! -d "$@" ] || ( touch $@/.sgbuilt_lib && $(MAKE) -j1 -C $@ ) || exit $$?

%_only:
	touch $(@:_only=)/.sgbuilt_lib && $(MAKE) -j1 -C $(@:_only=)

%_clean:
	$(MAKE) -j1 -C $(@:_clean=) clean; rm -f $(@:_clean=)/.sgbuilt_lib; true

##############################################################################

# Not the best place to install these symlinks, but it will do for now
romfs:
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -s config/passwd /etc/passwd
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -s config/group /etc/group
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -s config/mtab /etc/mtab
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -s config/fstab /etc/fstab
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -s config/resolv.conf /etc/resolv.conf
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD -e CONFIG_USER_TINYLOGIN_SHADOWPASSWDS -s config/shadow /etc/shadow
	for i in $(DIRS_y) ; do \
		[ ! -d $$i ] || make -C $$i romfs || exit $$? ; \
	done

##############################################################################

clean: clean_links
	-for i in $(DIRS_all); do \
		if [ -f $$i/.sgbuilt_lib ]; then \
			make -C $$i clean ; \
			rm -f $$i/.sgbuilt_lib; \
		fi; \
	done

##############################################################################
#
# The libs we link into this directory
#
#
LINKLIBS=\
	$(ROOTDIR)/lib/libjpeg/*.a \
	$(ROOTDIR)/lib/libpng/*.a \
	$(ROOTDIR)/lib/libm/*.a \
	$(ROOTDIR)/lib/libatm/src/lib/.libs/*.a \
	$(ROOTDIR)/lib/libdes/*.a \
	$(ROOTDIR)/lib/libaes/*.a \
	$(ROOTDIR)/lib/libpam/build/libpam*/.libs/*.so \
	$(ROOTDIR)/lib/libpcap/build/*.a \
	$(ROOTDIR)/lib/libnet/*.a \
	$(ROOTDIR)/lib/liblzma/*.a \
	$(ROOTDIR)/lib/libares/build/*.a \
	$(ROOTDIR)/lib/libidn/build/lib/.libs/libidn.a \
	$(ROOTDIR)/lib/libcurl/build/lib/.libs/libcurl.a \
	$(ROOTDIR)/lib/libcurl/build/lib/.libs/libcurl.so \
	$(ROOTDIR)/lib/libssl/build/*.so* \
	$(ROOTDIR)/lib/libssl/build/*.a \
	$(ROOTDIR)/lib/zlib/libz.a \
	$(ROOTDIR)/lib/libbzip2/libbz2.so* \
	$(ROOTDIR)/lib/libatm/src/lib/*.so* $(ROOTDIR)/lib/libatm/src/lib/*.a \
	$(ROOTDIR)/lib/libcrypt_old/libcrypt_old.a \
	$(ROOTDIR)/lib/libldap/builddir/libraries/libldap/.libs/*.a \
	$(ROOTDIR)/lib/libldap/builddir/libraries/liblber/.libs/*.a \
	$(ROOTDIR)/lib/pcre/builddir/.libs/*.a \
	$(ROOTDIR)/prop/libsnapgear++/*.a $(ROOTDIR)/prop/libsnapgear++/*.so* \
	$(ROOTDIR)/prop/libsnapgear/*.a $(ROOTDIR)/prop/libsnapgear/*.so* \
	$(ROOTDIR)/user/tinytcl/*.a $(ROOTDIR)/user/tinytcl/*.so* \
	$(ROOTDIR)/prop/config/*.a $(ROOTDIR)/prop/config/*.so* \
	$(ROOTDIR)/prop/uberconfig/*.a $(ROOTDIR)/prop/uberconfig/*.so* \
	$(ROOTDIR)/prop/gccmapping/*.a $(ROOTDIR)/prop/gccmapping/*.so* \
	$(ROOTDIR)/lib/expat/*.a $(ROOTDIR)/lib/expat/*.so* \
	$(ROOTDIR)/lib/libupnp/upnp/bin/*.so* $(ROOTDIR)/lib/adns/src/libadns.a \
	$(ROOTDIR)/lib/flex/build/*.a \
	$(ROOTDIR)/lib/libgmp/*.a \
	$(ROOTDIR)/lib/lzo/build/src/.libs/*.a \
	$(ROOTDIR)/lib/osip2/build/install/lib/*.a \
	$(ROOTDIR)/prop/libsscep/*.a \
	$(ROOTDIR)/prop/statsd/*.a \
	$(ROOTDIR)/lib/libefence/*.a \
	$(ROOTDIR)/lib/termcap/*.a \
	$(ROOTDIR)/lib/sysfsutils/build/lib/.libs/*.a \
	$(ROOTDIR)/lib/libnfnetlink/build/src/.libs/*.so* \
	$(ROOTDIR)/lib/libnetfilter_conntrack/build/src/.libs/*.so* \
	$(ROOTDIR)/lib/libnetfilter_log/build/src/.libs/*.so* \
	$(ROOTDIR)/lib/STLport/lib/*.a

##############################################################################

make_links: clean_links
	@echo "Making symlinks in lib/"
	@for p in $(LINKLIBS); do \
		for i in $$p; do \
			[ ! -e "$$i" ] && continue; \
			[ "`basename $$i`" = CVS ] && continue; \
			[ -L "./`basename $$i`" ] && continue; \
			ln -s $$i .; \
		done ; \
	done

clean_links:
	@-for i in *; do \
		[ "$$i" = "Makefile" ] && continue; \
		[ -L "$$i" ] || continue ; \
		case "`file -L $$i`" in \
		*directory*) ;; \
		*) rm -f $$i ;; \
		esac ; \
	done

##############################################################################

prune:
	-for i in $(DIRS_all) ; do \
		found=0; \
		for j in $(sort $(DIRS_y)) ; do \
			if [ $$i = $$j ]; then \
				found=1; \
			fi; \
		done; \
		if [ $$found = 0 ]; then \
			rm -fr $$i; \
		fi; \
	done
##############################################################################
