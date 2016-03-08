dtb-y += MOGOLIA_TL_2GFreq_config.dtb
dtb-y += MOGOLIA_TL_V1_TRIAL_config.dtb
dtb-y += MOGOLIA_ULG_V3_noNFC_config.dtb
dtb-y += MOGOLIA_ULCA_GENERAL_newRF_noNFC_config.dtb
dtb-y += MOGOLIA_TL_CMCC_CERTIFICATION_noNFC_config.dtb
dtb-y += MOGOLIA_ULCA_V1_noNFC_config.dtb
dtb-y += MOGOLIA_TL_V1_config.dtb
dtb-y += MOGOLIA_ULG_V1_config.dtb
dtb-y += MOGOLIA_TL_V3_noNFC_config.dtb
dtb-y += MOGOLIA_ULCA_GENERAL_V3_noNFC_config.dtb
dtb-y += MOGOLIA_TL_V3_config.dtb
dtb-y += MOGOLIA_ULG_V3_config.dtb

targets += dtbs

dtbs: $(addprefix $(obj)/, $(dtb-y))
	$(Q)rm -f $(obj)/../*.dtb

clean-files := *.dtb
