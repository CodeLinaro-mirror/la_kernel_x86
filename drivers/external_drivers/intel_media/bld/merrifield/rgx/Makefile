# Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.
# DRIVER_NAME := pvrrgx
DRIVER_NAME := tngdisp

obj-y += $(DRIVER_NAME).o

KERNELDIR ?= $(ANDROID_BUILD_TOP)/$(KERNEL_SRC_DIR)

MEDIADIR=../../..
RGXDIR=$(MEDIADIR)/graphics/rgx
DISPLAYDIR=$(MEDIADIR)/display/tng/drv

MEDIAINC=drivers/external_drivers/intel_media
RGXINC=$(MEDIAINC)/graphics/rgx
DISPINC=$(MEDIAINC)/display/tng

ccflags-y += \
        -I$(MEDIAINC)/interface \
	-I$(RGXINC)/include \
	-I$(RGXINC)/include/drm \
	-I$(RGXINC)/include/public \
	-I$(RGXINC)/include/public/powervr \
	-I$(RGXINC)/generated/rgxcmp_bridge \
	-I$(RGXINC)/generated/pdumpmm_bridge \
	-I$(RGXINC)/generated/dc_bridge \
	-I$(RGXINC)/generated/hostportio_bridge \
	-I$(RGXINC)/generated/rgxtq_bridge \
	-I$(RGXINC)/generated/rgxtq2_bridge \
	-I$(RGXINC)/generated/mm_bridge \
	-I$(RGXINC)/generated/cmm_bridge \
	-I$(RGXINC)/generated/rgxpdump_bridge \
	-I$(RGXINC)/generated/rgxray_bridge \
	-I$(RGXINC)/generated/rgxsignals_bridge \
	-I$(RGXINC)/generated/smm_bridge \
	-I$(RGXINC)/generated/pdump_bridge \
	-I$(RGXINC)/generated/sync_bridge \
	-I$(RGXINC)/generated/debugmisc_bridge \
	-I$(RGXINC)/generated/htbuffer_bridge \
	-I$(RGXINC)/generated/rgxinit_bridge \
	-I$(RGXINC)/generated/rgxkicksync_bridge \
	-I$(RGXINC)/generated/rgxta3d_bridge \
	-I$(RGXINC)/generated/srvcore_bridge \
	-I$(RGXINC)/generated/breakpoint_bridge \
	-I$(RGXINC)/generated/cache_bridge \
	-I$(RGXINC)/generated/pvrtl_bridge \
	-I$(RGXINC)/generated/rgxhwperf_bridge \
	-I$(RGXINC)/generated/regconfig_bridge \
	-I$(RGXINC)/generated/ion_bridge \
	-I$(RGXINC)/generated/ri_bridge \
	-I$(RGXINC)/generated/synctracking_bridge \
	-I$(RGXINC)/generated/timerquery_bridge \
	-I$(RGXINC)/generated/devicememhistory_bridge \
	-I$(RGXINC)/generated/pdumpctrl_bridge \
	-I$(RGXINC)/tools/intern/debug/include \
	-I$(RGXINC)/tools/intern/debug/dbgdriv/common \
	-I$(RGXINC)/hwdefs \
	-I$(RGXINC)/hwdefs/km \
	-I$(RGXINC)/hwdefs/km/cores \
	-I$(RGXINC)/services/include \
	-I$(RGXINC)/services/include/shared \
	-I$(RGXINC)/services/include/env/linux \
	-I$(RGXINC)/services/3rdparty/intel_drm \
	-I$(RGXINC)/services/3rdparty/intel_devfreq \
	-I$(RGXINC)/services/system/rgx_intel \
	-I$(RGXINC)/services/system/include \
	-I$(RGXINC)/services/system/common/env/linux \
	-I$(RGXINC)/services/shared/devices/rgx \
	-I$(RGXINC)/services/shared/include \
	-I$(RGXINC)/services/srvinit/devices \
	-I$(RGXINC)/services/srvinit/devices/rgx \
	-I$(RGXINC)/services/srvinit/include \
	-I$(RGXINC)/services/server/env/linux \
	-I$(RGXINC)/services/server/include \
	-I$(RGXINC)/services/server/devices/rgx \
	-I$(RGXINC)/kernel/drivers/staging/imgtec \
	-I$(DISPINC)/interface \
	-I$(DISPINC)/drv/ospm \
	-I$(DISPINC)/drv \
	-Iinclude/linux \
	-Iinclude/drm \
	-Idrivers/staging/android \
	-Idrivers/staging/android/ion \
	-Iarch/arm/include

ifeq ($(CONFIG_GFX_RGX_BVNC),)
# If not defined, default to Tangier/Merrifield
CONFIG_GFX_RGX_BVNC := 1.76.4.6
endif

cgr_bvnc := $(subst ., ,$(CONFIG_GFX_RGX_BVNC))
cgr_b := $(word 1, $(cgr_bvnc))
cgr_v := $(word 2, $(cgr_bvnc))
cgr_n := $(word 3, $(cgr_bvnc))
cgr_c := $(word 4, $(cgr_bvnc))

# Example: ccflags-y += -DRGX_BVNC_CORE_KM_HEADER="\"cores/rgxcore_km_1.76.4.6.h\""
# Example: ccflags-y += -DRGX_BNC_CONFIG_KM_HEADER="\"configs/rgxconfig_km_1.V.4.4.h\""

ccflags-y += -DRGX_BVNC_CORE_KM_HEADER="\"cores/rgxcore_km_$(CONFIG_GFX_RGX_BVNC).h\""
ccflags-y += -DRGX_BNC_CONFIG_KM_HEADER="\"configs/rgxconfig_km_$(cgr_b).V.$(cgr_n).$(cgr_c).h\""

ccflags-y += \
	-DANDROID \
        -DLINUX \
        -DPVRSRV_MODNAME="\"pvrsrvkm\"" \
        -DPVR_BUILD_DIR="\"intel_android\"" \
        -DPVR_LDM_DRIVER_REGISTRATION_NAME="\"pvrsrvkm\"" \
        -DSUPPORT_RGX=1 \
        -DSUPPORT_LINUX_X86_WRITECOMBINE \
        -DSUPPORT_LINUX_X86_PAT \
        -DSYS_USING_INTERRUPTS \
        -DLDM_PCI \
        -DSUPPORT_DRM \
        -DSUPPORT_DRM_EXT \
        -DBC_DISCONTIG_BUFFERS \
        -DCACHEFLUSH_TYPE=CACHEFLUSH_X86 \
        -DSUPPORT_GPUTRACE_EVENTS \
        -DPVR_LINUX_PHYSMEM_MAX_POOL_PAGES=5120 \
        -DPVR_LINUX_ARM_PAGEALLOC_FLUSH_THRESHOLD=256 \
        -DPVR_LINUX_PHYSMEM_MIN_NUM_PAGES=256 \
        -DPVR_LINUX_PHYSMEM_MAX_ALLOC_ORDER=2 \
        -DSUPPORT_NATIVE_FENCE_SYNC \
        -DPVRSRV_NEED_PVR_DPF \
        -DSUPPORT_SYSTEM_INTERRUPT_HANDLING \
        -DSUPPORT_SHARED_SLC \
        -DPVR_MMAP_USE_VM_INSERT \
        -DPVR_LINUX_KMALLOC_ALLOCATION_THRESHOLD=16384\
        -DPVR_ANDROID_ION_USE_SG_LENGTH \
        -DPVRSRV_ENABLE_PROCESS_STATS \
        -DPVR_ANDROID_SYNC_HEADER="\"../../drivers/staging/android/sync.h\"" \
        -DGPUVIRT_VALIDATION_NUM_OS=8 \
        -DPVRSRV_FULL_SYNC_TRACKING_HISTORY_LEN=256 \
        -DSUPPORT_PERCONTEXT_FREELIST \
        -DSUPPORT_AUTH \
		-DRGX_FW_HEAP_SHIFT=25	\
		-DPVRSRV_APPHINT_CLEANUPTHREADPRIORITY=0	\
		-DPVRSRV_APPHINT_CLEANUPTHREADWEIGHT=0	\
		-DPVRSRV_APPHINT_WATCHDOGTHREADPRIORITY=0	\
		-DPVRSRV_APPHINT_WATCHDOGTHREADWEIGHT=0	\
		-DPVR_LINUX_PHYSMEM_MAX_ALLOC_ORDER_NUM=2	\
		-DPVR_DIRTY_BYTES_FLUSH_THRESHOLD=1048576	\
		-DPVR_LINUX_HIGHORDER_ALLOCATION_THRESHOLD=256	\
		-DPVR_DRM_NAME="\"pvr\""	\
		-DPVRSRV_APPHINT_ENABLEFWPOISONONFREE=IMG_FALSE	\
		-DPVRSRV_APPHINT_FWPOISONONFREEVALUE=0xBD	\
		-DPVRSRV_APPHINT_GENERAL_NON4K_HEAP_PAGE_SIZE=0x4000	\
		-DPVRSRV_APPHINT_TIMECORRCLOCK=0	\
		-DDMABUF_IMPORT_PHYSHEAP_ID=0		\
		-DPVRSRV_GPUVIRT_NUM_OSID=2			\
		-DSUPPORT_KERNEL_SRVINIT=1			\
		-DPVRSRV_APPHINT_HWRDEBUGDUMPLIMIT=1			\
		-DPVRSRV_APPHINT_ENABLETRUSTEDDEVICEACECONFIG=IMG_FALSE	\
		-DPVRSRV_APPHINT_HTBUFFERSIZE=0x1000					\
		-DPVRSRV_APPHINT_ENABLESIGNATURECHECKS=IMG_FALSE		\
		-DPVRSRV_APPHINT_SIGNATURECHECKSBUFSIZE=1024			\
		-DPVRSRV_APPHINT_DISABLECLOCKGATING=0					\
		-DPVRSRV_APPHINT_DISABLEDMOVERLAP=0						\
		-DPVRSRV_APPHINT_ENABLECDMKILLINGRANDMODE=0				\
		-DPVRSRV_APPHINT_ENABLEFWCONTEXTSWITCH=7				\
		-DPVRSRV_APPHINT_ENABLERDPOWERISLAND=2					\
		-DPVRSRV_APPHINT_FIRMWAREPERF=0							\
		-DPVRSRV_APPHINT_FWCONTEXTSWITCHPROFILE=2				\
		-DPVRSRV_APPHINT_HWPERFDISABLECUSTOMCOUNTERFILTER=0		\
		-DPVRSRV_APPHINT_HWPERFFWBUFSIZEINKB=2048				\
		-DPVRSRV_APPHINT_HWPERFHOSTBUFSIZEINKB=128				\
		-DPVRSRV_APPHINT_JONESDISABLEMASK=0						\
		-DPVRSRV_APPHINT_NEWFILTERINGMODE=1						\
		-DPVRSRV_APPHINT_TRUNCATEMODE=0							\
		-DPVRSRV_APPHINT_USEMETAT1=0							\
		-DPVRSRV_APPHINT_RGXBVNC="\"BVNC_INTEL\""				\
		-DPVRSRV_APPHINT_ENABLEHTBLOGGROUP=0			\
		-DPVRSRV_APPHINT_HTBOPERATIONMODE=1				\
		-DPVRSRV_APPHINT_HWPERFFWFILTER=0				\
		-DPVRSRV_APPHINT_HWPERFHOSTFILTER=0				\
		-DPVRSRV_APPHINT_ASSERTONHWRTRIGGER=IMG_FALSE	\
		-DPVRSRV_APPHINT_ASSERTOUTOFMEMORY=IMG_FALSE	\
		-DPVRSRV_APPHINT_CHECKMLIST=0					\
		-DPVRSRV_APPHINT_ENABLELOGGROUP=0				\
		-DPVRSRV_APPHINT_FIRMWARELOGTYPE=0				\
		-DPVRSRV_APPHINT_ENABLEAPM=2					\
		-DPVRSRV_APPHINT_DISABLEFEDLOGGING=IMG_FALSE	\
		-DPVRSRV_APPHINT_ZEROFREELIST=IMG_FALSE			\
		-DPVRSRV_APPHINT_DUSTREQUESTINJECT=IMG_FALSE	\
		-DPVRSRV_APPHINT_DISABLEPDUMPPANIC=IMG_FALSE	\
		-DRGX_FW_FILENAME="\"rgx.fw.signed\""		\
	$(NULL)

#	-DPVRSRV_ENABLE_FW_TRACE_DEBUGFS \

ccflags-y += \
	-DSUPPORT_DISPLAY_CLASS \
	$(NULL)

# Skip the following dependencies on Merrifield Virtual Platform
ifneq ($(CONFIG_BOARD_MRFLD_VP),y)
ccflags-y += \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES
endif

# for ION
ccflags-$(CONFIG_ION) += -DSUPPORT_ION=1

ccflags-y += \
	-DSUPPORT_POWMON_WO_GPIO_PIN 

# for PDUMP
ccflags-$(CONFIG_PDUMP) += -DPDUMP -DSUPPORT_DBGDRV_EVENT_OBJECTS

# Either BUILD is debug or release. It can NOT be both.
ccflags-$(CONFIG_DRM_HANDSET_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE

ccflags-$(CONFIG_DRM_HANDSET_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\"" \
	-DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_HANDLEALLOC_KM \
	-DRGXFW_ALIGNCHECKS \
	-DPVR_RI_DEBUG

# The following is an option, it can be enabled for more debug information
# Add 'DEBUG_PVR_BRIDGE=y' to AndroidRGX.mk to enable it.
ccflags-$(DEBUG_PVR_BRIDGE) += \
	-DDEBUG_BRIDGE_KM \
	-DDEBUG_TRACE_BRIDGE_KM \
	-DDEBUG_BRIDGE_KM_DISPATCH_TABLE

# pvrsrvkm
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/server/common/pmr.o \
	$(RGXDIR)/services/server/common/devicemem_server.o \
	$(RGXDIR)/services/server/common/devicemem_heapcfg.o \
	$(RGXDIR)/services/server/common/devicemem_history_server.o \
	$(RGXDIR)/services/server/common/dc_server.o \
	$(RGXDIR)/services/server/common/connection_server.o \
	$(RGXDIR)/services/server/common/pvrsrv.o \
	$(RGXDIR)/services/server/common/sync_server.o \
	$(RGXDIR)/services/server/common/physmem_lma.o \
	$(RGXDIR)/services/server/common/power.o \
	$(RGXDIR)/services/server/common/physheap.o \
	$(RGXDIR)/services/server/common/scp.o \
	$(RGXDIR)/services/server/common/mmu_common.o \
	$(RGXDIR)/services/server/common/srvcore.o \
	$(RGXDIR)/services/server/common/lists.o \
	$(RGXDIR)/services/server/common/handle.o \
	$(RGXDIR)/services/server/common/physmem.o \
	$(RGXDIR)/services/server/common/process_stats.o \
	$(RGXDIR)/services/server/common/cache_km.o \
	$(RGXDIR)/services/server/common/htbserver.o \
	$(RGXDIR)/services/server/common/physmem_tdsecbuf.o \
	$(RGXDIR)/services/server/common/pvr_notifier.o \
	$(RGXDIR)/services/server/common/sync_checkpoint.o \
	$(RGXDIR)/services/server/env/linux/allocmem.o \
	$(RGXDIR)/services/server/env/linux/physmem_osmem_linux.o \
	$(RGXDIR)/services/server/env/linux/osfunc_x86.o \
	$(RGXDIR)/services/server/env/linux/osfunc.o \
	$(RGXDIR)/services/server/env/linux/event.o \
	$(RGXDIR)/services/server/env/linux/pvr_bridge_k.o \
	$(RGXDIR)/services/server/env/linux/pvr_debug.o \
	$(RGXDIR)/services/server/env/linux/osconnection_server.o \
	$(RGXDIR)/services/server/env/linux/pdump.o \
	$(RGXDIR)/services/server/env/linux/pvr_debugfs.o \
	$(RGXDIR)/services/server/env/linux/pvr_gputrace.o \
	$(RGXDIR)/services/server/env/linux/trace_events.o \
	$(RGXDIR)/services/server/env/linux/handle_idr.o \
	$(RGXDIR)/services/server/env/linux/module_common.o \
	$(RGXDIR)/services/server/env/linux/pmr_os.o \
	$(RGXDIR)/services/server/env/linux/km_apphint.o \
	$(RGXDIR)/services/server/env/linux/osmmap_stub.o \
	$(RGXDIR)/services/shared/common/devicemem.o \
	$(RGXDIR)/services/shared/common/hash.o \
	$(RGXDIR)/services/shared/common/devicemem_utils.o \
	$(RGXDIR)/services/shared/common/sync.o \
	$(RGXDIR)/services/shared/common/ra.o \
	$(RGXDIR)/services/shared/common/uniq_key_splay_tree.o \
	$(RGXDIR)/services/shared/devices/rgx/rgx_compat_bvnc.o \
	$(RGXDIR)/services/shared/devices/rgx/rgx_hwperf_table.o \
	$(RGXDIR)/services/server/common/tlintern.o \
	$(RGXDIR)/services/server/common/tlstream.o \
	$(RGXDIR)/services/server/common/tlserver.o \
    $(RGXDIR)/services/shared/common/tlclient.o	\
	$(RGXDIR)/services/shared/common/htbuffer.o

# SUPPORT_NATIVE_FENCE_SYNC
$(DRIVER_NAME)-y += \
	$(RGXDIR)/kernel/drivers/staging/imgtec/pvr_sync.o	\
	$(RGXDIR)/kernel/drivers/staging/imgtec/pvr_drm.o	\
	$(RGXDIR)/kernel/drivers/staging/imgtec/pvr_platform_drv.o	\

$(DRIVER_NAME)-$(CONFIG_DRM_HANDSET_DEBUG) += \
	$(RGXDIR)/services/server/common/ri_server.o

# for PDUMP
$(DRIVER_NAME)-$(CONFIG_PDUMP) += \
	$(RGXDIR)/services/server/common/pdump_common.o \
	$(RGXDIR)/services/server/common/pdump_mmu.o \
	$(RGXDIR)/services/server/common/pdump_physmem.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/linux/main.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/linux/hostfunc.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/common/ioctl.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/common/dbgdriv.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/common/dbgdriv_handle.o \
	$(RGXDIR)/tools/intern/debug/dbgdriv/common/hotkey.o \
	$(RGXDIR)/services/server/devices/rgx/rgxpdump.o \
	$(RGXDIR)/services/shared/common/devicemem_pdump.o	\
    $(RGXDIR)/services/shared/common/devicememx_pdump.o \

# For SUPPORT_RGX
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/server/devices/rgx/rgxinit.o \
	$(RGXDIR)/services/server/devices/rgx/rgxmmuinit.o \
	$(RGXDIR)/services/server/devices/rgx/rgxtransfer.o \
	$(RGXDIR)/services/server/devices/rgx/rgxfwutils.o \
	$(RGXDIR)/services/server/devices/rgx/rgxcompute.o \
	$(RGXDIR)/services/server/devices/rgx/rgxutils.o \
	$(RGXDIR)/services/server/devices/rgx/rgxpower.o \
	$(RGXDIR)/services/server/devices/rgx/rgxregconfig.o \
	$(RGXDIR)/services/server/devices/rgx/rgxta3d.o \
	$(RGXDIR)/services/server/devices/rgx/debugmisc_server.o \
	$(RGXDIR)/services/server/devices/rgx/rgxmem.o \
	$(RGXDIR)/services/server/devices/rgx/rgxccb.o \
	$(RGXDIR)/services/server/devices/rgx/rgxtimerquery.o \
	$(RGXDIR)/services/server/devices/rgx/rgxdebug.o \
	$(RGXDIR)/services/server/devices/rgx/rgxbreakpoint.o \
	$(RGXDIR)/services/server/devices/rgx/rgxhwperf.o \
	$(RGXDIR)/services/server/devices/rgx/rgxtimecorr.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxkicksync.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxlayer_km_impl.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxmipsmmuinit.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxray.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxsignals.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxstartstop.o	\
	$(RGXDIR)/services/server/devices/rgx/rgxtdmtransfer.o	\

# For devfreq
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/3rdparty/intel_devfreq/rgxdf.o \
	$(RGXDIR)/services/3rdparty/intel_devfreq/dfrgx_utilstats.o \
	$(RGXDIR)/services/3rdparty/intel_devfreq/dfrgx_interface.o

# For Power Monitor
#$(DRIVER_NAME)-y += \
#	$(RGXDIR)/services/server/devices/rgx/rgxpowermon.o

$(DRIVER_NAME)-y += \
	$(RGXDIR)/generated/breakpoint_bridge/server_breakpoint_bridge.o \
	$(RGXDIR)/generated/cache_bridge/client_cache_direct_bridge.o \
	$(RGXDIR)/generated/cache_bridge/server_cache_bridge.o \
	$(RGXDIR)/generated/cmm_bridge/server_cmm_bridge.o \
	$(RGXDIR)/generated/dc_bridge/server_dc_bridge.o \
	$(RGXDIR)/generated/pdump_bridge/server_pdump_bridge.o \
	$(RGXDIR)/generated/pdump_bridge/client_pdump_direct_bridge.o \
	$(RGXDIR)/generated/srvcore_bridge/server_srvcore_bridge.o \
	$(RGXDIR)/generated/rgxinit_bridge/client_rgxinit_direct_bridge.o \
	$(RGXDIR)/generated/rgxkicksync_bridge/server_rgxkicksync_bridge.o \
	$(RGXDIR)/generated/pdumpmm_bridge/server_pdumpmm_bridge.o \
	$(RGXDIR)/generated/pdumpmm_bridge/client_pdumpmm_direct_bridge.o \
	$(RGXDIR)/generated/rgxcmp_bridge/server_rgxcmp_bridge.o \
	$(RGXDIR)/generated/debugmisc_bridge/server_debugmisc_bridge.o \
	$(RGXDIR)/generated/sync_bridge/server_sync_bridge.o \
	$(RGXDIR)/generated/sync_bridge/client_sync_direct_bridge.o \
	$(RGXDIR)/generated/rgxtq_bridge/server_rgxtq_bridge.o \
	$(RGXDIR)/generated/rgxtq2_bridge/server_rgxtq2_bridge.o \
	$(RGXDIR)/generated/rgxta3d_bridge/server_rgxta3d_bridge.o \
	$(RGXDIR)/generated/rgxpdump_bridge/server_rgxpdump_bridge.o \
	$(RGXDIR)/generated/rgxpdump_bridge/client_rgxpdump_direct_bridge.o \
	$(RGXDIR)/generated/rgxray_bridge/server_rgxray_bridge.o \
	$(RGXDIR)/generated/rgxsignals_bridge/server_rgxsignals_bridge.o \
	$(RGXDIR)/generated/pvrtl_bridge/server_pvrtl_bridge.o \
	$(RGXDIR)/generated/pvrtl_bridge/client_pvrtl_direct_bridge.o \
	$(RGXDIR)/generated/rgxhwperf_bridge/server_rgxhwperf_bridge.o \
	$(RGXDIR)/generated/regconfig_bridge/server_regconfig_bridge.o \
	$(RGXDIR)/generated/synctracking_bridge/server_synctracking_bridge.o \
	$(RGXDIR)/generated/synctracking_bridge/client_synctracking_direct_bridge.o \
	$(RGXDIR)/generated/timerquery_bridge/server_timerquery_bridge.o \
	$(RGXDIR)/generated/devicememhistory_bridge/server_devicememhistory_bridge.o \
	$(RGXDIR)/generated/devicememhistory_bridge/client_devicememhistory_direct_bridge.o \
	$(RGXDIR)/generated/pdumpctrl_bridge/server_pdumpctrl_bridge.o	\
	$(RGXDIR)/generated/pdumpctrl_bridge/client_pdumpctrl_direct_bridge.o	\
	$(RGXDIR)/generated/htbuffer_bridge/server_htbuffer_bridge.o	\
	$(RGXDIR)/generated/htbuffer_bridge/client_htbuffer_direct_bridge.o	\


$(DRIVER_NAME)-$(CONFIG_DRM_HANDSET_DEBUG) += \
	$(RGXDIR)/generated/ri_bridge/server_ri_bridge.o \
	$(RGXDIR)/generated/ri_bridge/client_ri_direct_bridge.o \

# for SUPPORT_SECURE_EXPORT
$(DRIVER_NAME)-y += \
	$(RGXDIR)/generated/mm_bridge/server_mm_bridge.o \
	$(RGXDIR)/generated/mm_bridge/client_mm_direct_bridge.o \

ccflags-y += \
       -DPVR_ANDROID_ION_HEADER="\"../../drivers/staging/android/ion/ion.h\"" \
       -DPVR_ANDROID_ION_PRIV_HEADER="\"../../drivers/staging/android/ion/ion_priv.h\"" \
       -I$(RGXINC)/generated/dmabuf_bridge

$(DRIVER_NAME)-y += \
       $(RGXDIR)/services/server/env/linux/physmem_dmabuf.o \
       $(RGXDIR)/generated/dmabuf_bridge/server_dmabuf_bridge.o

ifeq ($(CONFIG_ION),y)
$(DRIVER_NAME)-y += \
       $(RGXDIR)/services/system/rgx_intel/ion_support_intel.o
endif

# buffer class video
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/system/rgx_intel/bufferclass_video_linux.o \
	$(RGXDIR)/services/system/rgx_intel/bufferclass_video.o

# display class
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/server/env/linux/pvr_drm_gem.o \
	$(RGXDIR)/services/server/env/linux/pvr_drm_prime.o \
	$(RGXDIR)/services/system/rgx_intel/sysconfig.o \
	$(RGXDIR)/services/system/rgx_intel/pvr_drm_ext.o \
	$(RGXDIR)/services/3rdparty/intel_drm/dc_mrfld.o \
	$(RGXDIR)/services/system/common/env/linux/pci_support.o	\
	$(RGXDIR)/services/system/common/env/linux/dma_support.o	\
	$(RGXDIR)/services/system/common/env/linux/interrupt_support.o	\

# server initialization
$(DRIVER_NAME)-y += \
	$(RGXDIR)/services/srvinit/common/srvinit_pdump.o	\
	$(RGXDIR)/services/srvinit/devices/rgx/rgxfwimageutils.o	\
	$(RGXDIR)/services/srvinit/devices/rgx/rgxlayer_impl.o	\
	$(RGXDIR)/services/srvinit/devices/rgx/rgxsrvinit.o	\
	$(RGXDIR)/services/srvinit/devices/rgx/rgxsrvinit_script.o	\
	$(RGXDIR)/services/srvinit/devices/rgx/env/linux/km/rgxfwload.o	

# for SUPPORT_PVRSRV_GPUVIRT
$(DRIVER_NAME)-$(SUPPORT_PVRSRV_GPUVIRT) += \
	$(RGXDIR)/services/server/common/pvrsrv_vz_common.o \
	$(RGXDIR)/services/server/devices/rgx/rgxfwutils_vz_guest.o \
	$(RGXDIR)/services/system/common/vmm_pvz_client.o	\
	$(RGXDIR)/services/system/common/vmm_type_l4linux.o	\
	$(RGXDIR)/services/system/common/vz_generic_guest.o	\
	$(RGXDIR)/services/system/common/vz_physheap_common.o	\
	$(RGXDIR)/services/system/common/vz_physheap_generic.o	\
	$(RGXDIR)/services/system/common/vz_physheap_guest.o	\
	$(RGXDIR)/services/system/common/vz_vmm_pvz.o
# for PVR_DVFS
$(DRIVER_NAME)-$(PVR_DVFS) += \
	$(RGXDIR)/services/server/devices/rgx/rgxpdvfs.o	\

# for SUPPORT_WORKLOAD_ESTIMATION
$(DRIVER_NAME)-$(SUPPORT_WORKLOADE_ESTIMATION) += \
	$(RGXDIR)/services/server/devices/rgx/rgxworkest.o	\

# for SUPPORT_VIRTUALIZATION
$(DRIVER_NAME)-$(SUPPORT_VIRTUALIZATION) += \
	$(RGXDIR)/services/server/devices/rgx/rgxpower_vz_guest.o \
	$(RGXDIR)/services/server/devices/rgx/rgxhwperf_vz_guest.o \
	$(RGXDIR)/services/server/devices/rgx/rgxinit_vz_common.o \
	$(RGXDIR)/services/server/devices/rgx/rgxinit_vz_guest.o \

