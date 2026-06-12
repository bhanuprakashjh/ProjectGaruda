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
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/63de865751d3af4b289dd2daea4a8a89d46613de .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/8d6a5b7596fa773d9fc24a66c84af478d12aa9fc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/4746ae2d32579cc4a15eba32887cf05aeacc76f7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/d4810f521e41ca51d9821e3216ad6c6f9a12280c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/a0d4bf58d00d431d72360a257247b62235bd6e5c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/1692a47d11373de7f8ec3cbb709bb839fb9fe45b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/c129113168939e4dc7259e003bf2548d8fb613df .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/963cf22de02d0f9dbc2ba107410028e2310c271 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/e40fcd2aaf3d0c49c9d62ba56aedb62efd686a17 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/6850857509db4c623144fee01fe06334c04b936d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/17d7d02cc02310daf107891e6893539c34a100d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/4003501db3ac9d324886fd2d6f69fa56f437da58 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/1d3eb11e443ec1fa66dfc10668b885021844c1f4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/327818bf33c084d32bd4a21ff403bdf5d9124a44 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/3d6b22c7543f7e386d69a55b96054b74056e785f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/c80aa9245b0982b96e3b4417177264d000d5042a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/6b1aac61b51943e803afb14677eb7fd902eb45fb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/59a97a724ac75764c5d961300dd6244d7d0cee0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/3ab3c533de4b92bb1c8c3410757dd7e131876a3e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/22eac22e5b003b64ab01e3db255b8503f9bf78d5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/7c79842e9fa3b5ddae144f1c1a1742b5449b86e6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/74218346283762cc1cd2f088d09542fcce29b785 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/25765097336335b766652ef478f333beab904ff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/8dea14c59d294061b2a0a1e2f3ebfa8492210e37 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/44b390864e956f2e2ea2c78131933e64439c570e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/831cebfc7ab1241c518bc5a017a31511e449c8fe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/ff80e5170af00d29f56ad4705cfe121e23f8277f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/1aac7910050962292d0bc65e9847cf6fcc71d6cd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/9aaea0b2a2d2ee094150966bffa45218a2d63df9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/914d1b21c82c2e9d5570c7aebdaa5aa2aecc2553 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/f5b62d1ba204ae53629891440dea15b469d48287 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/d8f14fd5a4e7aba31f5982d9d8ee87fae7b54c89 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/6f76d303683880bc638df5c0470e089542f06cef .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/9fd339e5499dac928bd96c97d7a390e7412c5b2a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/c29ec6e7294fce531cf658e1e919553f48bd3a82 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/932cbce698d47b21539de46585ae8d4799646746 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/9d6138306e450529c49b6ad61f2016ef5f0819b3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/91220786d6c6d5f5080c835c85690c5132e9e90d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/801714df71eaa99569680c797507653195e930ba .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/5c463ab5f238649db4fcf704daed23dfa793043c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/7cfcdd908116162342bcde7cfbb5ec3c0f1bf027 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/211c7f354f78be51106c14f167781dc197516608 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/61d3120b425ef26c2d22088299b824efd55403a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/fe3e8245bde948b70cdece662c7ce7142c811f5e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/d3ea5639c5df0ff331f0040b2ec73dc64791e9f2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/618c880e70694efc11fb79b1d167bcdfbba3e199 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/speed_pi.o: ../motor/speed_pi.c  .generated_files/flags/default/4db2bec6b46b25772b1e18ee7e7b69bc9c64c4b6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/speed_pi.c  -o ${OBJECTDIR}/_ext/2109951130/speed_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/speed_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/3f97385489c0f5f444d064f0812e6e6ef55d387 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/8a4c49973c47579ed5af88f6fadc3a95fbc490b9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/5898c449c6178d6fd1d57cf2d5b8296258af474b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/2015cf4d1e393626eb453257fb4a3a4cef4a5d83 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/7adc92acb99eba088470b624151a7014ff75e394 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/f6e8cdee7b8f4e4611545e5396e1754e24483265 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/1a6e3a440fc5f6960cf3781526e0dbe4dfe72b73 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/46a5aeb315693ff6665c66a18ee6123b688f936a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/bd003706efd63c3656c45d37f839b775a08004f7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/b9e40c36bac9d2af7430869d4de2035b3fd76b47 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/cec839806d3b2a88b7c9f0309beae6c1b0906cc1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/de4fa58b54053049e66c2184a8b6f1f5b57bdac3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/dd241267c6796dcdf26df214463dabaa6f0f8092 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/10caf8cf7b1f14e5e0e1a027c2096549384cc470 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/7fd645dc190db24d9fd4ba0934d3d5948a9b7f77 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/396f533b48b35a1fc02963737aac9798b44ec61d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/f9c0edead102724f8bf48791e0cf0d9288267651 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/5508e58c1616a8ee47dd1bd31bc6e39b85727292 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/b9061a1d7b8aa3383d3c6aafe84091f83cf5a1e8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/91ce7a8403b2bab627ff4649ff475926d85cc0a1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/2b7aa2cc523d532cabfe2471e3690e0e3183fa4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/d193d5c1850c66b471ba12858467d03891573351 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/1198feef29ce3ac6982e454c0871b9f7bd51a2ab .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/c0108bc7b80cdd1d4589f7861b2cfd3899f26b20 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/d49637c11a7c3f0e537d222da198205fd58afa8c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/a0926547a0c4f6c503d58d0bf3c88effffa22db5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/f9bc7db3875a2cc7ce517ab2c3f6be42c6ae69fc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/c216f3fd4b9864b90f56091faac573887f7fcbcd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/a68f15e3e39b695d752d997c40df3cf84a148cb4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/f185df509ff7b52c7ed3f3396ac43ce8b079f205 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/7091a7595b5bc43b878fac2b34ba1c925c8f4aca .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/a0bf1680def741eb2d16fa9e892492edd27d612c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/23b7d2de4f887c8970f8057258f58ee0805028b2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/15e10848346d7f602393ac6c063fd6d1c32a898f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/9e179385cb9ef59aa852fe90c2712b9bfc9cfa2c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/6e86354ee016d17da4cae5c3d3a3ea9e5616e3aa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/939f49859800e43e074998ed22e8dcb0ebdda9a1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/576bbc05013a6d21cf121e1d7d0d2a86a769444b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/90ab5c97215b9e3aba70a8a8bba325f5cf93d7d6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/123a8dcadf413004eeb9bc3bed81651da57c1f83 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/91c26376cd524810aa312f573611d8792fb4fdc8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/f43d15c7747ce0a879fa39931b4888fd61f06669 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/1cecb16127d13a09871e49a804ea62d03e17cf8d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/f076fc3226ebb726406302c3ff4afefd67cecbc2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/d96fc8423430631030d34b15daaab0d565f8775 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/8c9b9e4325bbbf953439cb465d05503d0c68aeef .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/3b3dc980a1ab34dda5fca4c6f3f587924d148b4f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/e87e8875362ab1a3028842f6e4011d9b268739ff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/e0c4de2264689c4108858df264a58102009c0a28 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/52b5a14ee5d508e3a99d723dcae22167c4b55012 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/30e146ffae661161a92e2f852bc558e0da7bd437 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/speed_pi.o: ../motor/speed_pi.c  .generated_files/flags/default/abbba052359ef92512914ed065d6a35e3a086531 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/speed_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/speed_pi.c  -o ${OBJECTDIR}/_ext/2109951130/speed_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/speed_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/4fcc09f809aca0ba5dfb31478d1ea3a361fb001d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/5554106c4035dfce652f2de8c4e1d7953830aa50 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/8138ac7527d6731089953d9871746a6007009f38 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/600fe3779fda234de1623d40ada7debd5bb8574e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/a885a40d5120491d267089111aa817f0052666c8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
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

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
