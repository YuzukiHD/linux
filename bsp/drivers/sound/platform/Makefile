# SPDX-License-Identifier: GPL-2.0-or-later
#
# Allwinner audio platform driver Makefile
#
# Copyright(c) 2022-2027 Allwinnertech Co., Ltd.

# components
snd_soc_sunxi_component_rxsync-objs		+= snd_sunxi_rxsync.o
obj-$(CONFIG_SND_SOC_SUNXI_RXSYNC)		+= snd_soc_sunxi_component_rxsync.o

snd_soc_sunxi_component_sfx-objs		+= snd_sunxi_sfx.o
obj-$(CONFIG_SND_SOC_SUNXI_SFX)			+= snd_soc_sunxi_component_sfx.o

snd_soc_sunxi_component_test_clk-objs		+= snd_sunxi_test_clk.o
obj-$(CONFIG_SND_SOC_SUNXI_TEST_CLK)		+= snd_soc_sunxi_component_test_clk.o

ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_JACK_CODEC)))
snd_soc_sunxi_component_jack-objs		+= snd_sunxi_jack_codec.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_JACK_EXTCON)))
snd_soc_sunxi_component_jack-objs		+= snd_sunxi_jack_extcon.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_JACK_GPIO)))
snd_soc_sunxi_component_jack-objs		+= snd_sunxi_jack_gpio.o
endif
ifeq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_JACK_NONE)))
snd_soc_sunxi_component_jack-objs		+= snd_sunxi_jack_utils.o
obj-$(CONFIG_SND_SOC_SUNXI_JACK)		+= snd_soc_sunxi_component_jack.o
endif

# common -> platform of dma
snd_soc_sunxi_pcm-objs				+= snd_sunxi_pcm.o
obj-$(CONFIG_SND_SOC_SUNXI_PCM)			+= snd_soc_sunxi_pcm.o

# common -> platform of hdmi audio (use for ahub | i2s)
snd_soc_sunxi_pcm_hdmi-objs			+= snd_sunxi_pcm_hdmi.o
obj-$(CONFIG_SND_SOC_SUNXI_PCM_HDMI)		+= snd_soc_sunxi_pcm_hdmi.o

# common -> common interface
snd_soc_sunxi_common-objs			+= snd_sunxi_common.o
obj-$(CONFIG_SND_SOC_SUNXI_COMMON)		+= snd_soc_sunxi_common.o

# codec -> internal codec
ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_SUN50IW10_CODEC)))
snd_soc_sunxi_internal_codec-objs		+= snd_sun50iw10_codec.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_SUN50IW9_CODEC)))
snd_soc_sunxi_internal_codec-objs		+= snd_sun50iw9_codec.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_SUN8IW11_CODEC)))
snd_soc_sunxi_internal_codec-objs		+= snd_sun8iw11_codec.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_SUN8IW18_CODEC)))
snd_soc_sunxi_internal_codec-objs		+= snd_sun8iw18_codec.o
else ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_SUN55IW3_CODEC)))
snd_soc_sunxi_internal_codec-objs		+= snd_sun55iw3_codec.o
endif
snd_soc_sunxi_internal_codec-objs		+= snd_sunxi_dap.o
obj-$(CONFIG_SND_SOC_SUNXI_INTERNALCODEC)	+= snd_soc_sunxi_internal_codec.o

# codec -> dummy codec
snd_soc_sunxi_dummy_codec-objs			+= snd_sunxi_dummy_codec.o
obj-$(CONFIG_SND_SOC_SUNXI_DUMMYCODEC)		+= snd_soc_sunxi_dummy_codec.o

# codec -> hdmi codec
snd_soc_sunxi_codec_hdmi-objs			+= snd_sunxi_codec_hdmi.o
obj-$(CONFIG_SND_SOC_SUNXI_CODEC_HDMI)		+= snd_soc_sunxi_codec_hdmi.o

# codec -> edp codec
snd_soc_sunxi_codec_edp-objs			+= snd_sunxi_codec_edp.o
obj-$(CONFIG_SND_SOC_SUNXI_CODEC_EDP)		+= snd_soc_sunxi_codec_edp.o

# platform -> analog audio
snd_soc_sunxi_aaudio-objs			+= snd_sunxi_aaudio.o
obj-$(CONFIG_SND_SOC_SUNXI_AAUDIO)		+= snd_soc_sunxi_aaudio.o

# platform -> owa
ifneq (, $(filter y m, $(CONFIG_SND_SOC_SUNXI_OWA_RXIEC61937)))
snd_soc_sunxi_owa-objs				+= snd_sunxi_owa_rx61937.o
endif
snd_soc_sunxi_owa-objs				+= snd_sunxi_owa.o
obj-$(CONFIG_SND_SOC_SUNXI_OWA)			+= snd_soc_sunxi_owa.o

# platform -> dmic
snd_soc_sunxi_dmic-objs				+= snd_sunxi_dmic.o
obj-$(CONFIG_SND_SOC_SUNXI_DMIC)		+= snd_soc_sunxi_dmic.o

# platform -> i2s
snd_soc_sunxi_i2s-objs				+= snd_sunxi_i2s.o
obj-$(CONFIG_SND_SOC_SUNXI_I2S)			+= snd_soc_sunxi_i2s.o

# platform -> ahub
snd_soc_sunxi_ahub_dam-objs			+= snd_sunxi_ahub_dam.o
obj-$(CONFIG_SND_SOC_SUNXI_AHUB_DAM)		+= snd_soc_sunxi_ahub_dam.o

snd_soc_sunxi_ahub-objs				+= snd_sunxi_ahub.o
obj-$(CONFIG_SND_SOC_SUNXI_AHUB)		+= snd_soc_sunxi_ahub.o

# common -> machine (note: Finally compile, save system startup time)
snd_soc_sunxi_machine-objs			+= snd_sunxi_mach.o
snd_soc_sunxi_machine-objs			+= snd_sunxi_mach_utils.o
obj-$(CONFIG_SND_SOC_SUNXI_MACH)		+= snd_soc_sunxi_machine.o
