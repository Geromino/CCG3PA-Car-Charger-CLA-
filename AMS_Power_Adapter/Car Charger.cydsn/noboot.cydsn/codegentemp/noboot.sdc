# THIS FILE IS AUTOMATICALLY GENERATED
# Project: G:\Projects\TesT_CLONE_Support_Cypress\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\noboot.cyprj
# Date: Sun, 19 Jul 2020 09:42:29 GMT
#set_units -time ns
create_clock -name {CyILO} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFClk} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySysClk} -period 41.666666666666664 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/sysclk}]]


# Component constraints for G:\Projects\TesT_CLONE_Support_Cypress\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\TopDesign\TopDesign.cysch
# Project: G:\Projects\TesT_CLONE_Support_Cypress\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\noboot.cyprj
# Date: Sun, 19 Jul 2020 09:42:28 GMT
