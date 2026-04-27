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
FINAL_IMAGE=${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../foc/mxlemming_obs.c ../foc/foc_v2_pi.c ../foc/foc_v2_observer.c ../foc/foc_v2_control.c ../foc/foc_v2_detect.c ../foc/foc_v3_smo.c ../foc/foc_v3_control.c ../foc/an1078_smc.c ../foc/an1078_motor.c ../gsp/gsp.c ../gsp/gsp_commands.c ../gsp/gsp_params.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_ata6847.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/hal_zcd.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../hal/hal_input_capture.c ../input/rx_decode.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../motor/bemf_zc.c ../motor/hwzc.c ../scope/scope_burst.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o ${OBJECTDIR}/_ext/2113678661/rx_decode.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o ${OBJECTDIR}/_ext/2109951130/hwzc.o ${OBJECTDIR}/_ext/2104772283/scope_burst.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360924651/clarke.o.d ${OBJECTDIR}/_ext/1360924651/park.o.d ${OBJECTDIR}/_ext/1360924651/svpwm.o.d ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d ${OBJECTDIR}/_ext/1360925749/gsp.o.d ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d ${OBJECTDIR}/_ext/1360926148/board_service.o.d ${OBJECTDIR}/_ext/1360926148/clock.o.d ${OBJECTDIR}/_ext/1360926148/device_config.o.d ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d ${OBJECTDIR}/_ext/1360926148/port_config.o.d ${OBJECTDIR}/_ext/1360926148/timer1.o.d ${OBJECTDIR}/_ext/1360926148/uart1.o.d ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d ${OBJECTDIR}/_ext/2111190731/quality.o.d ${OBJECTDIR}/_ext/2111190731/health.o.d ${OBJECTDIR}/_ext/2111190731/adaptation.o.d ${OBJECTDIR}/_ext/2111190731/commission.o.d ${OBJECTDIR}/_ext/2111190731/learn_service.o.d ${OBJECTDIR}/_ext/2109951130/commutation.o.d ${OBJECTDIR}/_ext/2109951130/pi.o.d ${OBJECTDIR}/_ext/2109951130/startup.o.d ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d ${OBJECTDIR}/_ext/2109951130/hwzc.o.d ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d ${OBJECTDIR}/_ext/1360926148/eeprom.o.d ${OBJECTDIR}/_ext/1472/garuda_service.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o ${OBJECTDIR}/_ext/2113678661/rx_decode.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o ${OBJECTDIR}/_ext/2109951130/hwzc.o ${OBJECTDIR}/_ext/2104772283/scope_burst.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../foc/mxlemming_obs.c ../foc/foc_v2_pi.c ../foc/foc_v2_observer.c ../foc/foc_v2_control.c ../foc/foc_v2_detect.c ../foc/foc_v3_smo.c ../foc/foc_v3_control.c ../foc/an1078_smc.c ../foc/an1078_motor.c ../gsp/gsp.c ../gsp/gsp_commands.c ../gsp/gsp_params.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_ata6847.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/hal_zcd.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../hal/hal_input_capture.c ../input/rx_decode.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../motor/bemf_zc.c ../motor/hwzc.c ../scope/scope_burst.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33AK128MC106
MP_LINKER_FILE_OPTION=,--script=p33AK128MC106.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/fe6ed72d2a02f0266ca54df8c5bc4538527acc3a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/51cdaff6d0403f629b92bf9c00541ab627a91941 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/d1cb2c899139b266dfca8e9e9c736e21d6114944 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/5e8e8695207ae42abe515205871aa3cf3878d5d9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/c3e1ce8a4bb6dfb84b1190f3001759415156e8fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/8f8335c5679e09ad36608e488da18ca145f11bea .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/b274432c06b0cc818a73bae7c28dd75f7fd0d4f4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/1bb919067ec03dfc231c42397eedc810e03f576c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/43df280c08529fd9b534b0515aca9a4a3ae290d8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/c35be7c595397d74e7c099236ef1c5caf692676 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/65dabe6a569efca870a94fcbc73b4345a9de8e6e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/b00307aadd77e62b55ea302dd8faf91e7320e303 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/741ef490b1f9a3e2407adae109c1dc84c666787 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/4afd07066f958a86418d8015fe53e62b3202b033 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/9d9eab89b94e4bff15ef2f1925364d096999a5fc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/f07ce920cfde97b4ff5b2fcbd08e3011fe9729a5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/6ba4b19f27be04fdefd58859f55df131e3ec3308 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/2368b752e5b0bc3f2d558abc096321417101e15d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/4939eedba3bb07e7413bd57907b9a68e54f8ad96 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/c37c25ec43518c92aa93a7ee584a320800bc0283 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/616805819ccf1c1a3d8c51147be4bf7e83eec91a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/698be70c2baf32df8cec313048d19672231b9fc9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/167902f2d751f05f30b1e597e2474d525e1853f4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/6d6374dd92470b859d85d62f31d72946eb33dbde .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/fd1c285006434815642139a936a5fde4ea51e50d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/d58f12d912398e191079d4e0fea4cb63a54143fe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/203f4d310a47de51883a87600727c1634196af8e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/735f5aa60f2e79bf3bfa3939499804469a1f4396 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/121fe558923d0f33c28b3f29e2a16e18ae1cac0e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/46bd2cbdf6fee82831b669fdfac4aac3de8f26e9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/b3e0ae37ac611de7a75b86ba6dfa434bc30aaa9a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/756af54fffa1af0c6d31345c6c3804a0ec7484a4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/9a879ce3fd88f35131115b0519e8d23019929883 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/1478e45fd3a0fb5cec9a223726db619a1c0e2f51 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/befd30530ebb4813cb323cd08cb04500a6b67d7e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/957222ae0eeafee4e950960601b1056c6a8c5199 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/7d1ce535af2c276d015046dea43d7363dc7bcf07 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/88283af70414b19778a2971d186961d8d4faa39 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/42da19796b3f445dfd22da0da28c90054f4722a2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/70c7c8e4ab0a474c1e05a03df93984b5e1db7575 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/c2c94401b8816db047436c72d741d399a059fd24 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/6f7786aceef419d9fcbe61a73e864bad0a359d22 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/cad1b6cac31d0fda604575958ed01481a8fb96df .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/5aa1c56b84896c9b26769075e5bd2875ba702a3e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/6a009879c34a3b402b988b8c373b91d1de29806f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/24b59011a9b3d83fe59c3870ef67137591f4cb7b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/a11311272bb3e6940e7aea930fbbe8ce03ad8e3e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/b26427e5979e8a20b9b575ed3c1a13ef99bed49c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/8be9e385f65b1ce2ca2c4cb354f5380286ba2cd3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/e619b03af68c41ea04573f7bb18609a351c90342 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/113b73f105cafa88ea3c764318726cdb729e9131 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/6ec77c0809af12bab532014c56c22142d18a4113 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/9099915e49a2783f1c3fdd9ba7685d83e9fd47f2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/ee4e6b8a4af8c59a4d7798a834a490918ed4cfc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/b17051d5d063cbe8e17a722cca466cb470e48857 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/1ff3e3172c0bf1c2e6a48b2136a1204f7de5604b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/461b03c27e500a80aa73039170fec603a389ff4a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o: ../foc/mxlemming_obs.c  .generated_files/flags/default/4b4f439e9d999f25790198db33897fb423ee82ef .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/mxlemming_obs.c  -o ${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/mxlemming_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o: ../foc/foc_v2_pi.c  .generated_files/flags/default/8e7d2e13326688bfbb876672f8222b5cc7e94487 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_pi.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o: ../foc/foc_v2_observer.c  .generated_files/flags/default/725139a4fb67a99ba6e7b859df5b55a65c13b12 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_observer.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_control.o: ../foc/foc_v2_control.c  .generated_files/flags/default/1358f52ab7e7d14137f961a9ccd261de652c9e85 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o: ../foc/foc_v2_detect.c  .generated_files/flags/default/bab8c8c35a86b28ccede519a0c477d899309472e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v2_detect.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v2_detect.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o: ../foc/foc_v3_smo.c  .generated_files/flags/default/7e4fcdf7cea450fa9c177c9ba4e8c0e6d16b96b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_smo.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_smo.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/foc_v3_control.o: ../foc/foc_v3_control.c  .generated_files/flags/default/716584435a2dc889edc84f9580a9369f926aa567 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/foc_v3_control.c  -o ${OBJECTDIR}/_ext/1360924651/foc_v3_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/foc_v3_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360924651/an1078_smc.o: ../foc/an1078_smc.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_smc.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_smc.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_smc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360924651/an1078_motor.o: ../foc/an1078_motor.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651"
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d
	@${RM} ${OBJECTDIR}/_ext/1360924651/an1078_motor.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/an1078_motor.c  -o ${OBJECTDIR}/_ext/1360924651/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/an1078_motor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/ecdfa950fc6b2615660074be87d521e408e07328 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/a556066cf90e44e9a5b7e8d463c0e659d0f5ded .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/a9381d4d4bc5456ecbd8721f36698a7dd007a99c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/af73cfd34ebca1c8dd3dae68622bee9dbc26c17b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/e5d3db6c61fc76a3a0efa8ada7c54073869ebc9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/dee1feb8a5600de5099e209c1cbaaf981ea0cccc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/30d51d1a70c29d005095a67c6cef329515574beb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/178b9c7d6f0dcb926b8bfd3eefed7d656966b7c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_ata6847.o: ../hal/hal_ata6847.c  .generated_files/flags/default/a802f3f33ba53c9f0db1208b25f1762d01c78d79 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_ata6847.c  -o ${OBJECTDIR}/_ext/1360926148/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_ata6847.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/e9fcae730aab1583f57578f0da37374576b6170f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/f2f5c2c50ddbef1d5ab9931c1ac44693dbd780c7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_zcd.o: ../hal/hal_zcd.c  .generated_files/flags/default/629bb269c37f273600bae1955abe107b4d1af628 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_zcd.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_zcd.c  -o ${OBJECTDIR}/_ext/1360926148/hal_zcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_zcd.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/68d73e9796f53e5d55fbea023856945c6c78e97a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/b3f96cd1140201f86ebf1b95af262f23d3e8426c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/679684d1e9deb7c3fe211dba3fc71217b14cd353 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/7fe3d184db33a4172374c7c85b21734ca5de1bfe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_input_capture.o: ../hal/hal_input_capture.c  .generated_files/flags/default/dec4fb6717553496c1f7da24c5d3b15d5d98779c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_input_capture.c  -o ${OBJECTDIR}/_ext/1360926148/hal_input_capture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_input_capture.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2113678661/rx_decode.o: ../input/rx_decode.c  .generated_files/flags/default/d9d95b831045ce24d7a9381c1abf1356611c7341 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2113678661" 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o.d 
	@${RM} ${OBJECTDIR}/_ext/2113678661/rx_decode.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../input/rx_decode.c  -o ${OBJECTDIR}/_ext/2113678661/rx_decode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2113678661/rx_decode.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/381f5f5ec1f028f2ad1967b9cf3ff8fbf5d716da .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/4f3e8fde0327c3aec8a75e661cab2d3725c3f52c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/ff20d3eea5a19aa21542c36ce3f481d77a6a200c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/a6a3f707a3fa153406345aa09cb1dee963236e12 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/f1224dc184e8daea48d12e5ee97b66717837d465 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/13b2a0214c38c43403f0ee29cc309e086f9bef22 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/ea130cafeeb6f3cd099452e55834d5e5694c824a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/284c53dbb74600bfd554eb9952b625a2eb4e671 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/f1a94821c403b98e3a39662e342bd304175c3078 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/bemf_zc.o: ../motor/bemf_zc.c  .generated_files/flags/default/75177d6b6b21bfcbb3923baca6572c29a6c85a22 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/bemf_zc.c  -o ${OBJECTDIR}/_ext/2109951130/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/bemf_zc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/hwzc.o: ../motor/hwzc.c  .generated_files/flags/default/db8b732d93bcf46af22d3e72806a64ca4e49fb48 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/hwzc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/hwzc.c  -o ${OBJECTDIR}/_ext/2109951130/hwzc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/hwzc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2104772283/scope_burst.o: ../scope/scope_burst.c  .generated_files/flags/default/4272348b2d4635e59a931c59735aee07284c8707 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2104772283" 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o.d 
	@${RM} ${OBJECTDIR}/_ext/2104772283/scope_burst.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../scope/scope_burst.c  -o ${OBJECTDIR}/_ext/2104772283/scope_burst.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2104772283/scope_burst.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/2f04cd76d415ef0b27867b322c3cd3b6f1b08bec .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/74355e752713f8067bf0da14d703136b062eb2be .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/ddfd6caca32e160dc149d0b2cc1e955bd4eb2666 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I"../gsp" -I"../input" -I"../foc" -I"../scope" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/56b9776abcbe6e967aaae15932a8b992b0bc4d3a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
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
${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  ../x2cscope/X2CScopeLib.X.a     -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PKOB4=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--library-path="../x2cscope",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  ../x2cscope/X2CScopeLib.X.a -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--library-path="../x2cscope",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}/xc-dsc-bin2hex ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
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
