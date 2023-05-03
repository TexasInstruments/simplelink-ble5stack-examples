SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/imports.mak

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
LNK = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"

SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler ticlang --product $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output . ../../tirtos7/hci_test.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))

OBJECTS = Application_hci_test_app.obj Startup_osal_icall_ble.obj Application_util.obj Application_rcosc_calibration.obj Drivers_NV_crc.obj Drivers_NV_nvocmp.obj iCall_icall.obj iCall_icall_cc2650.obj iCall_icall_user_config.obj iCallBLE_ble_user_config.obj iCallBLE_ble_user_config_stack.obj iCallBLE_icall_api_lite.obj iCallBLE_icall_hci_tl.obj NPI_Transport_SPI_npi_tl_spi.obj NPI_Transport_UART_npi_tl_uart.obj NPI_Transport_npi_tl.obj NPI_npi_frame_hci.obj NPI_npi_rxbuf.obj NPI_npi_task.obj StackWrapper_HAL_Common_hal_assert.obj StackWrapper_HAL_Target_CC2650_Drivers_hal_flash_wrapper.obj StackWrapper_Host_gap.obj StackWrapper_Host_gapbondmgr.obj StackWrapper_Host_gatt_uuid.obj StackWrapper_Host_gattservapp_util.obj StackWrapper_Host_sm_ecc.obj StackWrapper_ICallBLE_ble_dispatch_JT.obj StackWrapper_ICallBLE_ble_dispatch_lite.obj StackWrapper_ICallBLE_icall_lite_translation.obj StackWrapper_OSAL_osal.obj StackWrapper_OSAL_osal_bufmgr.obj StackWrapper_OSAL_osal_cbtimer.obj StackWrapper_OSAL_osal_clock.obj StackWrapper_OSAL_osal_list.obj StackWrapper_OSAL_osal_memory_icall.obj StackWrapper_OSAL_osal_pwrmgr.obj StackWrapper_OSAL_osal_snv_wrapper.obj StackWrapper_OSAL_osal_timers.obj StackWrapper_ROM_common_rom_init.obj StackWrapper_Startup_icall_startup.obj StackWrapper_Tools_onboard.obj StackWrapper_Host_gattqual.obj StackWrapper_Host_gattqual1.obj StackWrapper_Host_gattqual2.obj StackWrapper_Host_gattqual3.obj StackWrapper_Host_gatttest.obj Startup_rom_init.obj OneLib_HCI_hci.obj OneLib_HCI_hci_data.obj OneLib_HCI_hci_event.obj OneLib_HCI_TL_hci_tl.obj OneLib_Host_ATT_att_client.obj OneLib_Host_ATT_att_server.obj OneLib_Host_ATT_att_util.obj OneLib_Host_GAP_gap_advertiser.obj OneLib_Host_GAP_gap_centlinkmgr.obj OneLib_Host_GAP_gap_configmgr.obj OneLib_Host_GAP_gap_devmgr.obj OneLib_Host_GAP_gap_initiator.obj OneLib_Host_GAP_gap_linkmgr.obj OneLib_Host_GAP_gap_perilinkmgr.obj OneLib_Host_GAP_gap_scanner.obj OneLib_Host_GAP_gap_task.obj OneLib_Host_GAP_linkdb.obj OneLib_Host_GATT_gatt_client.obj OneLib_Host_GATT_gatt_server.obj OneLib_Host_GATT_gatt_task.obj OneLib_Host_L2CAP_l2cap_if.obj OneLib_Host_L2CAP_l2cap_task.obj OneLib_Host_L2CAP_l2cap_util.obj OneLib_Host_Profiles_gapgattserver.obj OneLib_Host_Profiles_gattservapp.obj OneLib_Host_SM_sm_intpairing.obj OneLib_Host_SM_sm_mgr.obj OneLib_Host_SM_sm_pairing.obj OneLib_Host_SM_sm_rsppairing.obj OneLib_Host_SM_sm_task.obj OneLib_Host_SMP_smp.obj OneLib_LL_ble.obj OneLib_LL_ll.obj OneLib_LL_ll_access_address.obj OneLib_LL_ll_adv_end_causes.obj OneLib_LL_ll_ae.obj OneLib_LL_ll_common.obj OneLib_LL_ll_data.obj OneLib_LL_ll_ecc.obj OneLib_LL_ll_enc.obj OneLib_LL_ll_error_end_causes.obj OneLib_LL_ll_init_end_causes.obj OneLib_LL_ll_isr.obj OneLib_LL_ll_master_end_causes.obj OneLib_LL_ll_privacy.obj OneLib_LL_ll_rat.obj OneLib_LL_ll_scan_end_causes.obj OneLib_LL_ll_scheduler.obj OneLib_LL_ll_slave_end_causes.obj OneLib_LL_ll_timer_drift.obj OneLib_LL_ll_wl.obj OneLib_LL_Config_ll_config.obj StackWrapper_Host_rtls_srv.obj StackWrapper_NPI_npi.obj StackWrapper_HAL_Target_CC2650_Drivers_hal_trng_wrapper.obj Startup_main.obj $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = hci_test_src

CFLAGS = -I../.. \
    -I. \
    @${PROJECT_LOC}/Tools/Defines/hci_test.opt \
    @${PROJECT_LOC}/Tools/build_config_src.opt \
    "@$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/config/build_components.opt" \
    -I${PROJECT_LOC}/Application \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/rcosc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/npi/stack" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/devices/cc13x4_cc26x4/rf_patches" \
    -mcpu=cortex-m4  \
    -std=gnu9x \
    -g \
    -Oz \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/heapmgr" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/dev_info" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/profiles/simple_profile" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/inc" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/saddr" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/services/src/sdata" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/cc26xx" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src" \
    -DEM_CC1354P10_6_LP \
    -DCC13XX \
    -DCC13X4 \
    -DDeviceFamily_CC13X4 \
    -DFLASH_ONLY_BUILD \
    -DNVOCMP_NWSAMEITEM=1 \
    -Dxdc_target__isaCompatible_v8M \
    -DHEAPMGR_CONFIG=0x80 \
    -DHEAPMGR_SIZE=0x0 \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/devices/cc13x4_cc26x4" \
    "-I$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/posix/ticlang" \
    -DCC13X2P \
    -gdwarf-3 \
    -mcpu=cortex-m33 \
    -mfloat-abi=hard \
    -mfpu=fpv5-sp-d16

LFLAGS = --diag_wrap=off \
    --display_error_number \
    -x \
    --diag_suppress=16002-D \
    --diag_suppress=10247-D \
    --diag_suppress=10325-D \
    --diag_suppress=10229-D \
    --diag_suppress=16032-D \
    -c \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source" \
    "-L$(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/kernel/tirtos7/packages" \
    --define=FLASH_ONLY_BUILD=2 \
    ti_utils_build_linker.cmd.genlibs \
    -lti/devices/cc13x4_cc26x4/driverlib/bin/ticlang/driverlib.lib \
    ../../tirtos7/ticlang/cc13x4_cc26x4_app.cmd \
    "-Wl,-m,$(NAME).map" \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    "-L$(TICLANG_ARMCOMPILER)/lib" \
    -llibc.a
all: $(NAME).out

.INTERMEDIATE: syscfg
$(SYSCFG_FILES): syscfg
	@ echo generation complete

syscfg: ../../tirtos7/hci_test.syscfg
	@ echo Generating configuration files...
	@ $(SYSCFG_CMD_STUB) --output $(@D) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SYSCFG_H_FILES)
	@ echo Building $$@
	@ $(CC) $(CFLAGS) -c $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))

Application_hci_test_app.obj: ../../Application/hci_test_app.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Startup_osal_icall_ble.obj: ../../Startup/osal_icall_ble.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Application_util.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Application_rcosc_calibration.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/rcosc/rcosc_calibration.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Drivers_NV_crc.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv/crc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Drivers_NV_nvocmp.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/common/nv/nvocmp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCall_icall.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCall_icall_cc2650.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_cc2650.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCall_icall_user_config.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCallBLE_ble_user_config.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/ble_user_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCallBLE_ble_user_config_stack.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_user_config_stack.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCallBLE_icall_api_lite.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_api_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

iCallBLE_icall_hci_tl.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/app/icall_hci_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_Transport_SPI_npi_tl_spi.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl_spi.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_Transport_UART_npi_tl_uart.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl_uart.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_Transport_npi_tl.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_npi_frame_hci.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_frame_hci.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_npi_rxbuf.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_rxbuf.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

NPI_npi_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/npi/src/npi_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_HAL_Common_hal_assert.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/common/hal_assert.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_HAL_Target_CC2650_Drivers_hal_flash_wrapper.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common/hal_flash_wrapper.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gap.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gapbondmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gapbondmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gatt_uuid.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatt_uuid.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gattservapp_util.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattservapp_util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_sm_ecc.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_ecc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_ICallBLE_ble_dispatch_JT.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_dispatch_JT.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_ICallBLE_ble_dispatch_lite.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/stack/ble_dispatch_lite.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_ICallBLE_icall_lite_translation.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/icall/src/icall_lite_translation.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_bufmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_bufmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_cbtimer.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_cbtimer.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_clock.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_clock.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_list.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_list.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_memory_icall.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_memory_icall.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_pwrmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_pwrmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_snv_wrapper.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/mcu/cc26xx/osal_snv_wrapper.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_OSAL_osal_timers.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/osal/src/common/osal_timers.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_ROM_common_rom_init.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/common_rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Startup_icall_startup.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/icall_startup.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Tools_onboard.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/onboard.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gattqual.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattqual.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gattqual1.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattqual1.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gattqual2.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattqual2.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gattqual3.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattqual3.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_gatttest.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatttest.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Startup_rom_init.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/rom/agama_r1/rom_init.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_HCI_hci.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hci/cc26xx/hci.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_HCI_hci_data.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hci/cc26xx/hci_data.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_HCI_hci_event.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hci/cc26xx/hci_event.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_HCI_TL_hci_tl.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hci/cc26xx/hci_tl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_ATT_att_client.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/att_client.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_ATT_att_server.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/att_server.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_ATT_att_util.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/att_util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_advertiser.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_advertiser.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_centlinkmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_centlinkmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_configmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_configmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_devmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_devmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_initiator.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_initiator.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_linkmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_linkmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_perilinkmgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_perilinkmgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_scanner.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_scanner.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_gap_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gap_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GAP_linkdb.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/linkdb.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GATT_gatt_client.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatt_client.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GATT_gatt_server.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatt_server.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_GATT_gatt_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gatt_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_L2CAP_l2cap_if.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/l2cap_if.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_L2CAP_l2cap_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/l2cap_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_L2CAP_l2cap_util.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/l2cap_util.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_Profiles_gapgattserver.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gapgattserver.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_Profiles_gattservapp.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/gattservapp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SM_sm_intpairing.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_intpairing.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SM_sm_mgr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_mgr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SM_sm_pairing.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_pairing.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SM_sm_rsppairing.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_rsppairing.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SM_sm_task.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/sm_task.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_Host_SMP_smp.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/smp.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ble.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ble.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_access_address.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_access_address.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_adv_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_adv_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_ae.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_ae.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_common.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_common.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_data.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_data.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_ecc.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_ecc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_enc.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_enc.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_error_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_error_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_init_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_init_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_isr.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_isr.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_master_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_master_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_privacy.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_privacy.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_rat.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_rat.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_scan_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_scan_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_scheduler.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_scheduler.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_slave_end_causes.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_slave_end_causes.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_timer_drift.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_timer_drift.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_ll_wl.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_wl.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

OneLib_LL_Config_ll_config.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/controller/cc26xx/ll/ll_config.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_Host_rtls_srv.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/host/rtls_srv.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_NPI_npi.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/common/cc26xx/npi/stack/npi.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

StackWrapper_HAL_Target_CC2650_Drivers_hal_trng_wrapper.obj: $(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR)/source/ti/ble5stack_flash/hal/src/target/_common/hal_trng_wrapper.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

Startup_main.obj: ../../Startup/main.c $(SYSCFG_H_FILES)
	@ echo Building $@
	@ $(CC) $(CFLAGS) -c $< -o $@

$(NAME).out: $(OBJECTS)
	@ echo linking $@
	@ $(LNK) -Wl,-u,_c_int00 $(OBJECTS)  $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
	@ $(RM) $(SYSCFG_FILES)> $(DEVNULL) 2>&1
