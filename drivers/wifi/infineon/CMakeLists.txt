# Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
# One source file per layer: chip table, wire protocol, bus transport, boot
# sequence, blob lookup, control path, network interface/data path, management.
zephyr_library_sources(
  infsingle_chip.c
  infsingle_proto.c
  infsingle_boot.c
  infsingle_fw.c
  infsingle_ctrl.c
  infsingle_offload.c
  infsingle_netif.c
  infsingle_mgmt.c
)
zephyr_library_sources_ifdef(CONFIG_WIFI_AIROC_BUS_SDIO infsingle_bus_sdio.c)
zephyr_library_sources_ifdef(CONFIG_WIFI_AIROC_BUS_SPI infsingle_bus_spi.c)
zephyr_library_include_directories(.)

# Select the one device whose blobs get linked. The board's AIROC_PART and module
# choice picks the firmware, CLM and NVRAM from the Infineon HAL module. Fetch the
# blobs with "west blobs fetch hal_infineon".
set(hal_dir ${ZEPHYR_HAL_INFINEON_MODULE_DIR})
set(infsingle_res ${hal_dir}/zephyr/blobs/img/whd/resources)
set(infsingle_nvram_root ${hal_dir}/whd-expansion/WHD)

if(CONFIG_WIFI_AIROC_CUSTOM_BLOBS)
  set(infsingle_fw_bin ${CONFIG_WIFI_AIROC_CUSTOM_FW_PATH})
  set(infsingle_clm_bin ${CONFIG_WIFI_AIROC_CUSTOM_CLM_PATH})
  set(infsingle_nvram_txt ${CONFIG_WIFI_AIROC_CUSTOM_NVRAM_PATH})
else()
  # Per part: firmware directory/name/extension, CLM stem, NVRAM generation.
  # Per module: CLM component directory, NVRAM component directory and file.
  # Unset means the selection is missing or unsupported.
  set(fw_rel)
  set(fw_stem)
  set(fw_ext)
  set(clm_part)
  set(clm_stem)
  set(nv_gen)
  set(nv_part)
  set(clm_mod)
  set(nv_mod)
  set(nv_file)

  if(CONFIG_CYW55500)
    set(fw_rel 55500/COMPONENT_SM)
    set(fw_stem 55500A1)
    set(fw_ext .trxcse)
    set(clm_part 55500)
    set(clm_stem 55500A1)
    set(nv_gen WIFI6)
    set(nv_part 55500)
    if(CONFIG_CYW955513SDM2WLIPA_SM)
      set(clm_mod CYW955513SDM2WLIPA)
      set(nv_mod CYW955513SDM2WLIPA)
      set(nv_file cyw955513sdm2wlipa.txt)
    elseif(CONFIG_CYW55513IUBG_SM)
      set(clm_mod CYW55513IUBG)
      set(nv_mod CYW55513IUBG)
      set(nv_file cyw955513wlipa_rev103.txt)
    elseif(CONFIG_CYW55513_MURATA_2FY_SM)
      # The 2FY module reuses the IUBG CLM but has its own NVRAM.
      set(clm_mod CYW55513IUBG)
      set(nv_mod MURATA-2FY)
      set(nv_file cyw55500-sdio-mur-2FY.txt)
    elseif(CONFIG_CYW55513_MOD_PSE84_SOM)
      set(clm_mod CYW55513_MOD_PSE84_SOM)
      set(nv_mod CYW55513_MOD_PSE84_SOM)
      set(nv_file cyw55513modpse84som_rev3.txt)
    endif()
  elseif(CONFIG_CYW55572)
    set(fw_rel 55572/COMPONENT_SM)
    set(fw_stem 55572A1)
    set(fw_ext .trxse)
    set(clm_part 55572)
    set(clm_stem 55572A1)
    set(nv_gen WIFI6)
    set(nv_part 55572)
    if(CONFIG_CYW955573M2IPA1_SM)
      set(clm_mod CYW955573M2IPA1)
      set(nv_mod CYW955573M2IPA1)
      set(nv_file cyw955573m2ipa1_rev1.01.txt)
    endif()
  elseif(CONFIG_CYW4343W)
    set(fw_rel 4343W)
    set(fw_stem 4343WA1)
    set(fw_ext .bin)
    set(clm_part 4343W)
    set(clm_stem 4343WA1)
    set(nv_gen WIFI5)
    set(nv_part 4343W)
    if(CONFIG_CYW4343W_MURATA_1DX)
      set(clm_mod MURATA-1DX)
      set(nv_mod MURATA-1DX)
      set(nv_file cyw94343cy8cmod.txt)
    endif()
  elseif(CONFIG_CYW43439)
    set(fw_rel 43439)
    set(fw_stem 43439A0)
    set(fw_ext .bin)
    set(clm_part 43439)
    set(clm_stem 43439A0)
    set(nv_gen WIFI5)
    set(nv_part 43439)
    set(nv_file cyfmac43439-1YN.txt)
    if(CONFIG_CYW43439_MURATA_1YN)
      set(clm_mod MURATA-1YN)
      set(nv_mod MURATA-1YN)
    elseif(CONFIG_CYW43439_STERLING_LWBPLUS)
      set(clm_mod STERLING-LWBplus)
      set(nv_mod STERLING-LWBplus)
    endif()
  elseif(CONFIG_CYW43012)
    set(fw_rel 43012)
    set(fw_stem 43012C0)
    set(fw_ext .bin)
    set(clm_part 43012)
    set(clm_stem 43012C0)
    set(nv_gen WIFI5)
    set(nv_part 43012)
    if(CONFIG_CYW43012_MURATA_1LV)
      set(clm_mod MURATA-1LV)
      set(nv_mod MURATA-1LV)
      set(nv_file cyw9cy8ckit_062S2_43012.txt)
    endif()
  elseif(CONFIG_CYW4373)
    set(fw_rel 4373)
    set(fw_stem 4373A0)
    set(fw_ext .bin)
    set(clm_part 4373)
    set(clm_stem 4373A0)
    set(nv_gen WIFI5)
    set(nv_part 4373)
    if(CONFIG_CYW4373_MURATA_2AE)
      set(clm_mod MURATA-2AE)
      set(nv_mod MURATA-2AE)
      set(nv_file cyfmac4373-2AE.txt)
    elseif(CONFIG_CYW4373_MURATA_2BC)
      set(clm_mod MURATA-2BC)
      set(nv_mod MURATA-2BC)
      set(nv_file cyfmac4373-2BC.txt)
    elseif(CONFIG_CYW4373_STERLING_LWB5PLUS)
      set(clm_mod STERLING-LWB5plus)
      set(nv_mod STERLING-LWB5plus)
      set(nv_file cyfmac4373-2AE.txt)
    endif()
  endif()

  if(NOT fw_stem OR NOT clm_mod OR NOT nv_file)
    message(FATAL_ERROR "infsingle: no supported AIROC part/module selected. Set AIROC_PART and "
                        "the matching module choice in the board Kconfig (CYW43022 and CYW43438 "
                        "are not supported), or enable WIFI_AIROC_CUSTOM_BLOBS.")
  endif()

  # Only the production images unless the manufacturing-test build is requested.
  set(variant)
  if(CONFIG_WIFI_AIROC_MFG_FIRMWARE)
    set(variant -mfgtest)
  endif()

  set(infsingle_fw_bin ${infsingle_res}/firmware/COMPONENT_${fw_rel}/${fw_stem}${variant}${fw_ext})
  set(infsingle_clm_bin
      ${infsingle_res}/clm/COMPONENT_${clm_part}/COMPONENT_${clm_mod}/${clm_stem}${variant}.clm_blob)
  set(infsingle_nvram_txt
      ${infsingle_nvram_root}/COMPONENT_${nv_gen}/resources/nvram/COMPONENT_${nv_part}/COMPONENT_${nv_mod}/${nv_file})
endif()

# Fail at configure time, not at link time, when blobs are absent.
foreach(f ${infsingle_fw_bin} ${infsingle_clm_bin} ${infsingle_nvram_txt})
  if(NOT EXISTS ${f})
    message(FATAL_ERROR "infsingle: missing ${f} (run: west blobs fetch hal_infineon)")
  endif()
endforeach()

# Convert the three blobs into C arrays so they are linked into flash and read
# in place by the boot code; the driver needs no filesystem or runtime loader.
# The arrays have fixed names that infsingle_fw.c declares.
set(infsingle_blob_c ${CMAKE_CURRENT_BINARY_DIR}/infsingle_blob.c)
add_custom_command(
  OUTPUT ${infsingle_blob_c}
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/gen_blob.py
          --fw ${infsingle_fw_bin} --clm ${infsingle_clm_bin}
          --nvram ${infsingle_nvram_txt} --out ${infsingle_blob_c}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/gen_blob.py ${infsingle_fw_bin} ${infsingle_clm_bin} ${infsingle_nvram_txt}
)
zephyr_library_sources(${infsingle_blob_c})

