# This makefile is included from vendor/intel/*/AndroidBoard.mk.
$(eval $(call build_kernel_module,$(call my-dir),pn544, CONFIG_NFC_PN544_PLATFORM_DATA=y))

