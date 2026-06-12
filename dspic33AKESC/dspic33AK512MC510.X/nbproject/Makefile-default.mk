#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../foc/mxlemming_obs.c ../foc/foc_v2_pi.c ../foc/foc_v2_observer.c ../foc/foc_v2_control.c ../foc/foc_v2_detect.c ../foc/foc_v3_smo.c ../foc/foc_v3_control.c ../foc/an1078_smc.c ../foc/an1078_motor.c ../gsp/gsp.c ../gsp/gsp_commands.c ../gsp/gsp_params.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_ata6847.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/hal_zcd.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../hal/hal_input_capture.c ../input/rx_decode.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../motor/bemf_zc.c ../motor/hwzc.c ../motor/speed_pi.c ../scope/scope_burst.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o ${OBJECTDIR}/_ext/2113678661/rx_decode.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o ${OBJECTDIR}/_ext/2109951130/hwzc.o ${OBJECTDIR}/_ext/2109951130/speed_pi.o ${OBJECTDIR}/_ext/2104772283/scope_burst.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360924651/clarke.o.d ${OBJECTDIR}/_ext/1360924651/park.o.d ${OBJECTDIR}/_ext/1360924651/svpwm.o.d ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d ${OBJECTDIR}/_ext/1360925749/gsp.o.d ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d ${OBJECTDIR}/_ext/1360926148/board_service.o.d ${OBJECTDIR}/_ext/1360926148/clock.o.d ${OBJECTDIR}/_ext/1360926148/device_config.o.d ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d ${OBJECTDIR}/_ext/1360926148/port_config.o.d ${OBJECTDIR}/_ext/1360926148/timer1.o.d ${OBJECTDIR}/_ext/1360926148/uart1.o.d ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d ${OBJECTDIR}/_ext/2111190731/quality.o.d ${OBJECTDIR}/_ext/2111190731/health.o.d ${OBJECTDIR}/_ext/2111190731/adaptation.o.d ${OBJECTDIR}/_ext/2111190731/commission.o.d ${OBJECTDIR}/_ext/2111190731/learn_service.o.d ${OBJECTDIR}/_ext/2109951130/commutation.o.d ${OBJECTDIR}/_ext/2109951130/pi.o.d ${OBJECTDIR}/_ext/2109951130/startup.o.d ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d ${OBJECTDIR}/_ext/2109951130/hwzc.o.d ${OBJECTDIR}/_ext/2109951130/speed_pi.o.d ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d ${OBJECTDIR}/_ext/1360926148/eeprom.o.d ${OBJECTDIR}/_ext/1472/garuda_service.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o ${OBJECTDIR}/_ext/2113678661/rx_decode.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o ${OBJECTDIR}/_ext/2109951130/hwzc.o ${OBJECTDIR}/_ext/2109951130/speed_pi.o ${OBJECTDIR}/_ext/2104772283/scope_burst.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../foc/mxlemming_obs.c ../foc/foc_v2_pi.c ../foc/foc_v2_observer.c ../foc/foc_v2_control.c ../foc/foc_v2_detect.c ../foc/foc_v3_smo.c ../foc/foc_v3_control.c ../foc/an1078_smc.c ../foc/an1078_motor.c ../gsp/gsp.c ../gsp/gsp_commands.c ../gsp/gsp_params.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_ata6847.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/hal_zcd.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../hal/hal_input_capture.c ../input/rx_decode.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../motor/bemf_zc.c ../motor/hwzc.c ../motor/speed_pi.c ../scope/scope_burst.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33AK512MC510
MP_LINKER_FILE_OPTION=,--script=p33AK512MC510.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/5adf21ebcf7901919870e37cd1a8499728e063e3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/bca28c0e9a7a9ce2f95930510f6ff507c6b4a4eb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/d8b6757e3cbf5735ab3f212d66ff5a7e43015b82 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/8f2f65b1f5a18e334da83b68a9b309180b07da67 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/6c89cbad3397ed093356e682e6b3dcee1e454d98 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/a748fa77520bda3b559cac3ebd7921dafebbc697 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/27fafd4ff439c303b7269cc9eb6b08aa7ff5947a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/785b25d9df8299e1ed59bfe641dc573fbda85940 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/30a04b514fe291b0555a883205ab9d1e52603576 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/c36553a3e1dc70caeaccae314ad634a242780e82 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/d47022d610471187ff78d53a45ae537dc9fbd08c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/811af19d8de120115075a12e9fd91eb506c4f4d7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/7a1a9a6cf94cff40c332ea6341b873d5f47575f1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/35b34907cf77c5f16e96f33f3eb6422ca4b1f906 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/1866efc5b4b16ee4e98f09d10c230f3b15caa5a1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/26b7f12e630bf3be86657fd08877f50768180ea4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/c7f0bf9100a7f92e7abce3eca472dcfc779e38b2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/91a7720e7e8b65d9f0bfaeb89a8c738303afb538 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/b9ea7985bccf33c918403c6c7384af9778fbfd74 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/7816a69cb1223871cd22e87dab0b62741d3f5c8a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/e021134ea9e0bc88a0a09457d2c80d74c4f7504d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/b90974a6ff185a6f9d57aedb65727ad7ef514b7c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/a2fc10ffc7e8303a1e1cc6466b1d42eb25110d8e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/49da306e65c63e099ef8ce7279853dd8360b62ec .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/e53987c2df3f4eefd42aa1a0c2b991859fe0e2b5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/178226bbd0504ac8ceb61ce1ba96e842780dcab1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/b7d0a2c8d47fca265a7d4d6936eb83f9822f7b8c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/cf8fd61cbbc1c08020bce49298dec87114a7720 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/fe1657fe867565085e178688360e2381392d4c5f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/316ea911369213b85aff502ef934e0a193ee0089 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/f13444ef23067974a366947be34ab4c0b8dea519 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/dcc3b6536271b085567cd1c74b905f2ac1883db8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/9b9cd60f7ba7812ade60179b73b23a31a9555791 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/6d587ad4803d0b3e30454695006634d37609fac1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/6140a8dbfeb1acd462a3b42a85c974015a3f5a9a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/a29404ebf31d1e7271fc81f0e3990a2035a4466 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/c206fcf0a6252a636b1e856ffb8e47c658afaae2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/3e3c48b7fd60e4bec162a5587e930652a3138d6a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/741b15064847d1b747376679a8b3c6f7290c01e8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/ba9fa9a770a2259e378b0369dafa3562056705db .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/d562f7dff1bf79303b3d001b8da564cd95392a2f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/6b8df942ddc0878f92a0304a4122d5d6c6dadff1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/a08e32b26592aff209281180a57e5b22ffb3710c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/823023e6330b4233b1d89c66e0a96aababd54f1b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/204e93994c2113f0d3ae34579939a73399dcd044 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/ec99009bd32591e17919ab0e913b2454e7b3a796 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/speed_pi.o: ../motor/speed_pi.c  .generated_files/flags/default/8117434602ed2697ea6a164979ddb3720e13cd77 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/speed_pi.c  -o ${OBJECTDIR}/_ext/2109951130/speed_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/speed_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/da08347d344359d0340f84d2629837564dc542bc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/232b2f237c13f4987831407c9cab683de687b3e4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/5800b86114dba08d2c08c7391d3c7fa233c69f6f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/4a627e499710ca650368e2504c167cfed7b67598 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/9f865bb3bb17bbda3ade6b1212d21f039af72640 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/aff2740367aa62ecb8e5caa39d51ea7504a3a006 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/71ad31947bd2fb20bd0e927f5ffdb159eb15ee50 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/7ccbd21a0185849f1a0b2abed345dea047e766c9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/41fdc978d0acd5eb3a737e813799fd07c61786d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/7d7c2363dd9baa6d8c81ba595a5396b639fac15e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/d86fd2d26af5009ecb97f3b37263e4c954a637ef .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/a5de20d143a542c3580b026868476f06c4245e64 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/4a70be5e3c53fd1e1766bd8f9da11dfaa6c20230 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/7d440e8ee43b3baa94c2ec6f72eaab2f2ad99a91 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/bbd03495485f3c50fbd5d93e4436856b7e45b5fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/48faadda71ce03635f014a5b50a14a63b72d105f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/f36988401b20e8d37854368bddf75bf6331d64b1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/3a0645dea82a544e09151df5b4b1a05c2c577e45 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/cd17811619f5a7f28ad8ff87b24e22bed4fb7084 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/4e6167562330ae2f5cc3d950d1d495c748509710 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/ef0fd127952b71e684b4955ca7c3e561bbe0647d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/b31a5f9cefbb475a9d5df64cd6fd3b0b33590677 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/37219d93d716aa26d48c860dac91754605e40264 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/4bc5db8307aceccc1e671206b2c7baf1fe74934 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/b96bdb3d96305b65c1c02baeb7ad4b9f56f301b7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/d051c7a3310b723eeb0e41c6b331b1cceaefac39 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/7569c63d2613b7732231635b058ccf7fb15c7734 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/4c7ccd5a7697591db9354f66de080b546ef1720d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/c495f666ce6c67ad11555e7c482de43ec7d6d679 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/3764bce3699380d6db2aaa469fbe8e45a07ceadb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/7f762fef8fe58a67216ea641b216946a1527b37f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/56347b917d0e5f56a110b003607f5e1ded2e375f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/f3c5c38eae2bdbfafccef3fc5af44ac720ed2b5e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/d1416667106c818b619693ec98f1cde655e9027b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/8d4f901bc4cc523e8fe623dbf6d33be9ffd8fc89 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/60b6501f30d25ad6edf67f221adecde6038c2f9c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/ee0760074ea408fde288a82613f10fc063dd7099 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/55ce12e0aa53ae5b67bc15c71cfbf3485c453289 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/1fef30649f85a625589d70074011dc5f60a3ccbd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/5b9412174515acc96424d9bf9258390ece5d5d9a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/2771f52b49f61bbe6f7f6d3b5874e6f22075668b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/bf6f76b488180054cce86ea1fa285b28e1f00cd8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/50fb4bff9114fc577f483d2d3e28e823900b7bee .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/a5c5b38307be3d29706afbbb7dc972526af189ea .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/174d274fb18daaaab861a2e1b2fe1dc30504b738 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/839ef105541bd5710fb0b9683d078d09d43fdb70 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/fd9fdcda4f67d2d9bb54cce3acf1d5dc839f2fa8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/a124880994ba549d9b2d5d2e88945e81df12d00a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/be26c5b86b05cb02eda15a91660ebd9bc3689f42 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/450b554e8890e7c5946d29ea6e016dc8d7e1f8a5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/ca5bd8dec542080f3eaef96a3cd9bd6d1a3ddc13 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/speed_pi.o: ../motor/speed_pi.c  .generated_files/flags/default/5254a7f1958f49b705fe1052d3214c8c4d6fb997 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/speed_pi.c  -o ${OBJECTDIR}/_ext/2109951130/speed_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/speed_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/491424226d699b64b8cfaa94d8c00e2c3b455c00 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/ed998c3dbaf3f71ef4545a8db851ef070188db26 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/3c1968e1941aef89ba2ef5d085f9e0120d1ae036 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/f2984b915b694a38b92371ae3b171625b55d3dc1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/417ab9bf15f6272649388cef1d96973c9dcad58e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  ../x2cscope/X2CScopeLib.X.a     -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PKOB4=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--library-path="../x2cscope",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  ../x2cscope/X2CScopeLib.X.a -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--library-path="../x2cscope",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}/xc-dsc-bin2hex ${DISTDIR}/dspic33AK512MC510.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}
