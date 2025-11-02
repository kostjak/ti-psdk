#
# Utility makefile for SBL boot support with combined appimage tool
#
# Edit this file to suit your build needs
#

#######################################################################################
###################################[ PDK SBL ]#########################################

# Check if the RTOS SDK is set to mcu_plus_sdk or pdk
ifeq ($(RTOS_SDK),pdk)

SBL_CORE=mcu1_0
BOARD=$(BUILD_PDK_BOARD)
COMBINED_APPIMAGE_ATF_OPTEE_PATH=$(VISION_APPS_PATH)/out/sbl_combined_bootfiles/atf_optee_dir
ATF_TARGET_BOARD=generic
ifeq ($(SOC),j784s4)
	ATF_TARGET_BOARD=j784s4
endif
SBL_REPO_PATH=$(PDK_PATH)/packages/ti/boot/sbl
COMBINED_APPIMAGE_TOOL_PATH=$(SBL_REPO_PATH)/tools/combined_appimage

CROSS_COMPILE=$(GCC_LINUX_ARM_ROOT)/bin/$(CROSS_COMPILE_LINARO)

##############
### Main
##############
sbl_combined_bootimage: sbl_sd atf_optee sbl_vision_apps sbl_qnx_combined_bootimage

##############
### Install
##############
sbl_combined_bootimage_install_sd: sbl_combined_bootimage
	cp $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/tiboot3.bin $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/tifs.bin $(SBL_SD_FS_PATH)
	cp $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/app $(SBL_SD_FS_PATH)
	sync

##############
### SBL
##############
sbl_sd:
	$(MAKE) -C $(PDK_PATH)/packages/ti/build sbl_mmcsd_img TOOLS_INSTALL_PATH=$(PSDK_TOOLS_PATH) DISABLE_RECURSE_DEPS=no BOARD=$(BOARD) CORE=$(SBL_CORE) -s
	mkdir -p $(VISION_APPS_PATH)/out/sbl_combined_bootfiles
	cp $(PDK_PATH)/packages/ti/boot/sbl/binary/$(BOARD)/mmcsd/bin/sbl_mmcsd_img_$(SBL_CORE)_release.tiimage $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/tiboot3.bin
	cp $(PDK_PATH)/packages/ti/drv/sciclient/soc/V1/tifs.bin $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/tifs.bin

##############
### atf_optee
##############
atf_optee: sbl_sd
ifeq ($(BUILD_QNX_MPU), yes)
	# For ATF, setting HANDLE_EA_EL3_FIRST_NS=0 for QNX so that the all runtime exception to be routed to current exception level (or in EL1 if the current exception level is EL0)
	$(MAKE) -C $(VISION_APPS_PATH)/../trusted-firmware-a -s -j32 CROSS_COMPILE=$(CROSS_COMPILE) CC="$(CROSS_COMPILE)gcc --sysroot=$(LINUX_SYSROOT_ARM)" PLAT=k3 TARGET_BOARD=$(ATF_TARGET_BOARD) SPD=opteed  HANDLE_EA_EL3_FIRST_NS=0
endif
ifeq ($(BUILD_LINUX_MPU), yes)
	$(MAKE) -C $(VISION_APPS_PATH)/../trusted-firmware-a -s -j32 CROSS_COMPILE=$(CROSS_COMPILE) CC="$(CROSS_COMPILE)gcc --sysroot=$(LINUX_SYSROOT_ARM)" PLAT=k3 TARGET_BOARD=$(ATF_TARGET_BOARD) SPD=opteed
endif
	$(MAKE) -C $(VISION_APPS_PATH)/../ti-optee-os -s -j32 CROSS_COMPILE_core=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- CROSS_COMPILE_ta_arm32=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- CROSS_COMPILE_ta_arm64=$(GCC_LINUX_ARM_ROOT)/bin/aarch64-none-linux-gnu- NOWERROR=1 CFG_TEE_TA_LOG_LEVEL=0 CFG_TEE_CORE_LOG_LEVEL=2 CFG_ARM64_core=y ta-targets=ta_arm64 PLATFORM=k3 PLATFORM_FLAVOR=j7
	mkdir -p $(COMBINED_APPIMAGE_ATF_OPTEE_PATH)
	cp $(VISION_APPS_PATH)/../trusted-firmware-a/build/k3/$(ATF_TARGET_BOARD)/release/bl31.bin $(COMBINED_APPIMAGE_ATF_OPTEE_PATH)/bl31.bin
	cp $(VISION_APPS_PATH)/../ti-optee-os/out/arm-plat-k3/core/tee-pager_v2.bin $(COMBINED_APPIMAGE_ATF_OPTEE_PATH)/bl32.bin


##############
### vision apps
##############
INPUT_IMG_PATH = $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/vision_apps

sbl_vision_apps: sbl_sd
	mkdir -p $(INPUT_IMG_PATH)
	# build DM image
ifeq ($(BUILD_CPU_MCU1_0),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu1_0.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu1_0.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu1_0.xer5f
else
	$(MAKE) -C $(PDK_PATH)/packages/ti/build   ipc_qnx_echo_testb_freertos TOOLS_INSTALL_PATH=$(PSDK_TOOLS_PATH) CORE=mcu1_0 BOARD=$(BOARD) SOC=$(SOC) BUILD_PROFILE=release -s
	cp $(PDK_PATH)/packages/ti/binary/ipc_qnx_echo_testb_freertos/bin/$(BOARD)/ipc_qnx_echo_testb_freertos_mcu1_0_release_strip.xer5f $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu1_0.xer5f
endif
ifeq ($(BUILD_CPU_MCU2_0),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_0.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_0.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_0.xer5f
endif
ifeq ($(BUILD_CPU_MCU2_1),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu2_1.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_1.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_1.xer5f
endif
ifeq ($(BUILD_CPU_MCU3_0),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_0.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_0.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_0.xer5f
endif
ifeq ($(BUILD_CPU_MCU3_1),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu3_1.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_1.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_1.xer5f
endif
ifeq ($(BUILD_CPU_MCU4_0),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu4_0.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu4_0.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu4_0.xer5f
endif
ifeq ($(BUILD_CPU_MCU4_1),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_mcu4_1.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu4_1.xer5f
	$(TIARMCGT_LLVM_ROOT)/bin/tiarmstrip -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu4_1.xer5f
endif
ifeq ($(BUILD_CPU_C6x_1),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/C66/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_1.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_1.xe66
	$(CGT6X_ROOT)/bin/strip6x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_1.xe66
endif
ifeq ($(BUILD_CPU_C6x_2),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/C66/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c6x_2.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_2.xe66
	$(CGT6X_ROOT)/bin/strip6x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_2.xe66
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/$(C7X_TARGET)/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_1.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_1.xe71
	$(CGT7X_ROOT)/bin/strip7x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_1.xe71
endif
ifeq ($(BUILD_CPU_C7x_2),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/$(C7X_TARGET)/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_2.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_2.xe71
	$(CGT7X_ROOT)/bin/strip7x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_2.xe71
endif
ifeq ($(BUILD_CPU_C7x_3),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/$(C7X_TARGET)/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_3.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_3.xe71
	$(CGT7X_ROOT)/bin/strip7x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_3.xe71
endif
ifeq ($(BUILD_CPU_C7x_4),yes)
	cp $(VISION_APPS_PATH)/out/$(TARGET_SOC)/$(C7X_TARGET)/$(RTOS)/$(QNX_APP_PROFILE)/vx_app_rtos_qnx_c7x_4.out $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_4.xe71
	$(CGT7X_ROOT)/bin/strip7x -p $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_4.xe71
endif

##############################
## Combined bootapp image
##############################
OUT_DIR=$(VISION_APPS_PATH)/out/sbl_combined_bootfiles/
ATF_IMG=mpu1_0,$(COMBINED_APPIMAGE_ATF_OPTEE_PATH)/bl31.bin,0x70000000,0x70000000
OPTEE_IMG=load_only,$(COMBINED_APPIMAGE_ATF_OPTEE_PATH)/bl32.bin,0x9e800000,0x9e800000
KERNEL_IMG=load_only,$(QNX_BOOT_PATH)/qnx-ifs,0x80080000,0x80080000
DTB_IMG=
SPL_IMG=
  IMG1=mcu1_0,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu1_0.xer5f
ifeq ($(BUILD_CPU_MCU2_0),yes)
  IMG2=mcu2_0,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_0.xer5f
else
  IMG2=
endif
ifeq ($(BUILD_CPU_MCU2_1),yes)
  IMG3=mcu2_1,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_1.xer5f
else
  IMG3=
endif
ifeq ($(BUILD_CPU_MCU3_0),yes)
  IMG4=mcu3_0,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_0.xer5f
else
  IMG4=
endif
ifeq ($(BUILD_CPU_MCU3_1),yes)
  IMG5=mcu3_1,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu3_1.xer5f
else
  IMG5=
endif
ifeq ($(BUILD_CPU_C6x_1),yes)
  IMG6=c66xdsp_1,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_1.xe66
else
  IMG6=
endif
ifeq ($(BUILD_CPU_C6x_2),yes)
  IMG7=c66xdsp_2,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c6x_2.xe66
else
  IMG7=
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
  IMG8=c7x_1,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_1.xe71
else
  IMG8=
endif
ifeq ($(BUILD_CPU_C7x_2),yes)
  IMG9=c7x_2,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_2.xe71
else
  IMG9=
endif
ifeq ($(BUILD_CPU_C7x_3),yes)
  IMG10=c7x_3,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_3.xe71
else
  IMG10=
endif
ifeq ($(BUILD_CPU_C7x_4),yes)
  IMG11=c7x_4,$(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_4.xe71
else
  IMG11=
endif

sbl_qnx_combined_bootimage: atf_optee sbl_vision_apps
	echo "---------------------------"
	echo "start"
	echo "---------------------------"
	$(MAKE) -C $(COMBINED_APPIMAGE_TOOL_PATH) PDK_INSTALL_PATH=$(PDK_PATH)/packages OUT_DIR=$(OUT_DIR) ATF_IMG=$(ATF_IMG) OPTEE_IMG=$(OPTEE_IMG) KERNEL_IMG=$(KERNEL_IMG) DTB_IMG=$(DTB_IMG) SPL_IMG=$(SPL_IMG) IMG1=$(IMG1) IMG2=$(IMG2) IMG3=$(IMG3) IMG4=$(IMG4) IMG5=$(IMG5) IMG6=$(IMG6) IMG7=$(IMG7) IMG8=$(IMG8)
	mv $(OUT_DIR)/combined.appimage $(OUT_DIR)/app
	echo "---------------------------"
	echo "done"
	echo "---------------------------"

##############
### clean up
##############
sbl_combined_bootimage_clean: sbl_combined_bootimage_scrub

sbl_combined_bootimage_scrub: atf_optee_scrub
	rm -rf $(PDK_PATH)/packages/ti/binary/sbl_*
	rm -rf $(PDK_PATH)/packages/ti/binary/ti/boot/
	rm -rf $(PDK_PATH)/packages/ti/boot/sbl/binary
	rm -rf $(MCUSW_PATH)/binary
	rm -rf $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/

atf_optee_scrub:
	$(MAKE) -C $(VISION_APPS_PATH)/../trusted-firmware-a clean
	$(MAKE) -C $(VISION_APPS_PATH)/../ti-optee-os clean CFG_ARM64_core=y PLATFORM=k3 PLATFORM_FLAVOR=j7
	rm -rf $(VISION_APPS_PATH)/../trusted-firmware-a/build/k3
	rm -rf $(VISION_APPS_PATH)/../ti-optee-os/out/arm-plat-k3

endif # ifeq ($(RTOS_SDK),pdk)

#######################################################################################
#################################[ MCU PLUS SDK SBL ]##################################

# Check if the RTOS SDK is set to mcu_plus_sdk and BUILD_QNX_MPU is enabled
ifeq ($(RTOS_SDK),mcu_plus_sdk)
ifeq ($(BUILD_QNX_MPU), yes)

# Include the configuration file for the specified SOC
include $(MCU_PLUS_SDK_PATH)/tools/boot/qnxAppimageGen/board/$(SOC)-evm/config.mak

# Define paths and commands for generating RPRC images and SBL binaries
OUTRPRC_CMD = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/elf2rprc.js
INPUT_IMG_PATH = $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/vision_apps

ifeq ($(ECU_BUILD), no)
SBL_SD_HLOS_PATH = $(MCU_PLUS_SDK_PATH)/examples/drivers/boot/sbl_sd_hlos/$(SOC)-evm/wkup-r5fss0-0_nortos/ti-arm-clang
SBL_SD_HLOS_OUT_IMG = $(SBL_SD_HLOS_PATH)/sbl_sd_hlos.release.hs_fs.tiimage
SBL_OSPI_HLOS_PATH = $(MCU_PLUS_SDK_PATH)/examples/drivers/boot/sbl_ospi_hlos/$(SOC)-evm/wkup-r5fss0-0_nortos/ti-arm-clang
SBL_OSPI_HLOS_OUT_IMG = $(SBL_OSPI_HLOS_PATH)/sbl_ospi_hlos.release.hs_fs.tiimage
else
SBL_SD_HLOS_PATH = $(MCU_PLUS_SDK_PATH)/examples/drivers/boot/sbl_sd_hlos_${ECU_BUILD}/$(SOC)-evm/wkup-r5fss0-0_nortos/ti-arm-clang
SBL_SD_HLOS_OUT_IMG = $(SBL_SD_HLOS_PATH)/sbl_sd_hlos_${ECU_BUILD}.release.hs_fs.tiimage
SBL_OSPI_HLOS_PATH = $(MCU_PLUS_SDK_PATH)/examples/drivers/boot/sbl_ospi_hlos_${ECU_BUILD}/$(SOC)-evm/wkup-r5fss0-0_nortos/ti-arm-clang
SBL_OSPI_HLOS_OUT_IMG = $(SBL_OSPI_HLOS_PATH)/sbl_ospi_hlos_${ECU_BUILD}.release.hs_fs.tiimage
endif

# Ensure required variables are defined, else throw an error
ifndef MCU_PLUS_SDK_PATH
    $(error MCU_PLUS_SDK_PATH is not defined)
endif
ifndef VISION_APPS_PATH
    $(error VISION_APPS_PATH is not defined)
endif
ifndef SOC
    $(error SOC is not defined)
endif
ifndef QNX_BOOT_PATH
    $(error QNX_BOOT_PATH is not defined)
endif
ifndef QNX_SD_FS_BOOT_PATH
    $(error QNX_SD_FS_BOOT_PATH is not defined)
endif

# Define input images for R5F and C7x cores
IMAGE_MCU2_0 = $(VISION_APPS_PATH)/out/$(TARGET_SOC)/R5F/FREERTOS/release/vx_app_rtos_qnx_mcu2_0.out
IMAGE_C7X_1 = $(VISION_APPS_PATH)/out/$(TARGET_SOC)/C7524/FREERTOS/release/vx_app_rtos_qnx_c7x_1.out
IMAGE_C7X_2 = $(VISION_APPS_PATH)/out/$(TARGET_SOC)/C7524/FREERTOS/release/vx_app_rtos_qnx_c7x_2.out

# Define output RPRC files for R5F and C7x cores
RPRC_MCU2_0 = $(INPUT_IMG_PATH)/vx_app_rtos_qnx_mcu2_0.rprc
RPRC_C7X_1 = $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_1.rprc
RPRC_C7X_2 = $(INPUT_IMG_PATH)/vx_app_rtos_qnx_c7x_2.rprc

# Define the list of images to be included in the RTOS image list
IMG1 = $(BOOTIMAGE_CORE_ID_wkup-r5fss0-0) $(MCU_PLUS_SDK_PATH)/tools/sysfw/sciserver_binary/$(SOC)/sciclient_get_version.release.rprc
ifeq ($(BUILD_CPU_MCU2_0),yes)
    IMG2=$(BOOTIMAGE_CORE_ID_main-r5fss0-0) $(RPRC_MCU2_0)
else
    IMG2=
endif
ifeq ($(BUILD_CPU_C7x_1),yes)
    IMG3 = $(BOOTIMAGE_CORE_ID_c75ss0-0) $(RPRC_C7X_1)
else
    IMG3 =
endif
ifeq ($(BUILD_CPU_C7x_2),yes)
    IMG4 = $(BOOTIMAGE_CORE_ID_c75ss1-0) $(RPRC_C7X_2)
else
    IMG4 =
endif

RTOS_IMG_LIST = $(IMG1) $(IMG2) $(IMG3) $(IMG4)

# Build the SBL and app image
sbl: sbl_appimage_hlos sbl_sd_hlos sbl_ospi_hlos

# Clean up generated files
sbl_clean: sbl_scrub
sbl_scrub: sbl_appimage_hlos_clean sbl_sd_hlos_clean sbl_ospi_hlos_clean

# Generate the SBL MMC SD binary
sbl_sd_hlos:
	@echo "Generating SBL MMC SD Binary..."
	make -C $(SBL_SD_HLOS_PATH) clean
	make -C $(SBL_SD_HLOS_PATH)

# Generate the OSPI binary
sbl_ospi_hlos:
	@echo "Generating SBL OSPI Binary..."
	make -C $(SBL_OSPI_HLOS_PATH) clean
	make -C $(SBL_OSPI_HLOS_PATH)

# Generate RPRC images for vision apps
sbl_vision_apps_rprc:
	mkdir -p $(INPUT_IMG_PATH)
	@echo "SOC=$(SOC)"
	@echo "SW_VERSION=$(SW_VERSION)"
	$(call GENERATE_OUT_TO_RPRC, MCU2_0, $(IMAGE_MCU2_0), $(RPRC_MCU2_0))
	$(call GENERATE_OUT_TO_RPRC, C7X_1, $(IMAGE_C7X_1), $(RPRC_C7X_1))
	$(call GENERATE_OUT_TO_RPRC, C7X_2, $(IMAGE_C7X_2), $(RPRC_C7X_2))
	@echo "RPRC generation completed."

# Generate the combined app image for QNX
sbl_appimage_hlos: sbl_vision_apps_rprc
	@echo "------------------------------------------------------"
	@echo "Generating combined app image for QNX..."
	@echo "------------------------------------------------------"

	@echo "Copying QNX IFS image to prebuilt bin path..."
	mkdir -p ${MCU_PLUS_SDK_PATH}/tools/boot/hlos_prebuilt/${SOC}-evm/qnx/
	cp ${QNX_BOOT_PATH}/qnx-ifs ${MCU_PLUS_SDK_PATH}/tools/boot/hlos_prebuilt/${SOC}-evm/qnx/;

	@echo "Generating multicore hlos qnx app image..."
	$(MAKE) -C $(MCU_PLUS_SDK_PATH)/tools/boot/qnxAppimageGen BOARD=${SOC}-evm clean
	$(MAKE) -C $(MCU_PLUS_SDK_PATH)/tools/boot/qnxAppimageGen BOARD=${SOC}-evm RTOS_IMG_LIST="$(RTOS_IMG_LIST)"

	@echo "------------------------------------------------------"
	@echo "multicore hlos qnx app image generated successfully."
	@echo "------------------------------------------------------"

# Clean up app image files
sbl_appimage_hlos_clean:
	rm -f ${MCU_PLUS_SDK_PATH}/tools/boot/hlos_prebuilt/${SOC}-evm/qnx/qnx-ifs
	rm -rf $(VISION_APPS_PATH)/out/sbl_combined_bootfiles/

# Clean up SBL SD HLOS files
sbl_sd_hlos_clean:
	make -C $(SBL_SD_HLOS_PATH) clean

# Clean up SBL OSPI HLOS files
sbl_ospi_hlos_clean:
	make -C $(SBL_OSPI_HLOS_PATH) clean

# Copy SBL and QNX app image to SD card
sbl_bootimage_hs_install_sd: sbl_bootimage_install_sd
sbl_bootimage_install_sd:
	@echo "Copying SBL SD HLOS for ECU build to SD card..."
	cp $(SBL_SD_HLOS_OUT_IMG) $(QNX_SD_FS_BOOT_PATH)/tiboot3.bin

	@echo "Copying QNX app image for ECU build to SD card..."
	cp $(MCU_PLUS_SDK_PATH)/tools/boot/qnxAppimageGen/board/$(SOC)-evm/qnx.appimage.hs_fs $(QNX_SD_FS_BOOT_PATH)/app
	sync

	@echo "SBL tiboot3.bin and QNX app image copied to SD card successfully."

# Flash SBL OSPI HLOS to the device
sbl_bootimage_install_ospi:
	@echo "Flashing SBL OSPI HLOS for ECU build to device..."
	@read -p "Enter UART port (default: /dev/ttyUSB2): " input; \
	UART_FLASH_PORT=$${input:-/dev/ttyUSB2}; \
	echo "Using UART port: $$UART_FLASH_PORT"; \
	cd $(MCU_PLUS_SDK_PATH)/tools/boot && \
	python uart_uniflash.py -p $$UART_FLASH_PORT --cfg=sbl_prebuilt/j722s-evm/default_sbl_ospi_qnx_hs_fs_fc.cfg

# Macro to generate RPRC images
define GENERATE_OUT_TO_RPRC =
	@echo "Creating $(1) RPRC image"
	@if [ -f $(2) ]; then \
		$(OUTRPRC_CMD) $(2) $(SW_VERSION) $(3); \
		echo "$(1) RPRC image created: $(3)"; \
	else \
		echo "Warning: $(1) image $(2) does not exist. Skipping RPRC creation."; \
	fi
endef

endif  # ifeq ($(BUILD_QNX_MPU), yes)
endif  # ifeq ($(RTOS_SDK),mcu_plus_sdk)
