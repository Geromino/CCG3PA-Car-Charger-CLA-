# THIS FILE IS AUTOMATICALLY GENERATED
# Project: G:\Projects\ccg3pa-car-charger-cla\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\noboot.cyprj
# Date: Wed, 08 Jul 2020 12:31:21 GMT
#set_units -time ns
create_clock -name {Clock_2(FFB)} -period 41.666666666666664 -waveform {0 20.8333333333333} [list [get_pins {ClockBlock/ff_div_2}]]
create_clock -name {CyILO} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFClk} -period 25000 -waveform {0 12500} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySysClk} -period 41.666666666666664 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/sysclk}]]
create_generated_clock -name {Clock_2} -source [get_pins {ClockBlock/hfclk}] -edges {1 3 5} [list]


# Component constraints for G:\Projects\ccg3pa-car-charger-cla\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\TopDesign\TopDesign.cysch
# Project: G:\Projects\ccg3pa-car-charger-cla\AMS_Power_Adapter\Car Charger.cydsn\noboot.cydsn\noboot.cyprj
# Date: Wed, 08 Jul 2020 12:31:20 GMT
