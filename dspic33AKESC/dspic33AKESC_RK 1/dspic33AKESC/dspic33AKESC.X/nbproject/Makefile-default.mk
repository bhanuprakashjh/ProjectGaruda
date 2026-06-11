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
SOURCEFILES_QUOTED_IF_SPACED=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../gsp/gsp.c ../gsp/gsp_params.c ../gsp/gsp_commands.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360924651/clarke.o.d ${OBJECTDIR}/_ext/1360924651/park.o.d ${OBJECTDIR}/_ext/1360924651/svpwm.o.d ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d ${OBJECTDIR}/_ext/1360925749/gsp.o.d ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d ${OBJECTDIR}/_ext/1360926148/board_service.o.d ${OBJECTDIR}/_ext/1360926148/clock.o.d ${OBJECTDIR}/_ext/1360926148/device_config.o.d ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d ${OBJECTDIR}/_ext/1360926148/port_config.o.d ${OBJECTDIR}/_ext/1360926148/timer1.o.d ${OBJECTDIR}/_ext/1360926148/uart1.o.d ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d ${OBJECTDIR}/_ext/2111190731/quality.o.d ${OBJECTDIR}/_ext/2111190731/health.o.d ${OBJECTDIR}/_ext/2111190731/adaptation.o.d ${OBJECTDIR}/_ext/2111190731/commission.o.d ${OBJECTDIR}/_ext/2111190731/learn_service.o.d ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d ${OBJECTDIR}/_ext/1360926148/eeprom.o.d ${OBJECTDIR}/_ext/1472/garuda_service.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360924651/clarke.o ${OBJECTDIR}/_ext/1360924651/park.o ${OBJECTDIR}/_ext/1360924651/svpwm.o ${OBJECTDIR}/_ext/1360924651/pi_controller.o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o ${OBJECTDIR}/_ext/1360924651/smo_observer.o ${OBJECTDIR}/_ext/1360925749/gsp.o ${OBJECTDIR}/_ext/1360925749/gsp_params.o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o ${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/1360926148/hal_timer.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/1798147462/diagnostics.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../foc/clarke.c ../foc/park.c ../foc/svpwm.c ../foc/pi_controller.c ../foc/back_emf_obs.c ../foc/pll_estimator.c ../foc/flux_estimator.c ../foc/smo_observer.c ../gsp/gsp.c ../gsp/gsp_params.c ../gsp/gsp_commands.c ../gsp/gsp_snapshot.c ../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../hal/hal_timer.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../x2cscope/diagnostics.c ../hal/eeprom.c ../garuda_service.c ../main.c



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
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/d51fa974f652b469157d97efd26af7c375c5d6c7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/e151255ce6f6aaabf59811f9ff89d74151ad54a9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/3264d7b803f347f267e4f00d3b0929c8138ccdfb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/89a9b3fb9160afceae2e68bb7ca44f05695f2af0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/f2d426fe88c893365d05748546aa64c2de3ac985 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/d556124daea9c1ae1d7f1254e1396f454ca80fb2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/175d116196fd13cb34b21c4b83a0ddb7f8fd84e9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/97ac1587daf0183ba82d9e9e0c7d89e890b5d5f5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/a61571b0030a620c24144ccc444d335dd5960a55 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/2f0ca5b622b03947db02b46f4061d72e9af78ce4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/1866a8eaa36151bdd7005be0d57a7bef89e1590d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/8ff4f92470a92131f20053f73f8fc71d95e62c8b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/acae1929771cce8f4f68e94d66fff49dd7250e19 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/38c5e5b0c72f29bc276a3872594609316b19fb31 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/707a0ae20ffeea802af6bb55569d0b10a336e432 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/88307796b0acd7cd9de7c251f551421acf6bf43b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/31e4cf0fba22e112859dd758dec8f952d5b4388e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/992e6f5586231660e948af22bd248214f7206f3a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/c27ab8525bfaa6a42f6a5e8c450f812d76bc3d89 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/cffc00d096f0c649233783eb62ed0b9a89521aaf .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/8326888a17e2e2ca3f2d6f66f763577dea966107 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/46dc3999064d5a729bb0183571e3a4aafa17ca98 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/4683d61b7b8455ad851708a63ad4cbb5239b4989 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/402c0419add6eed50fe6191fcbfbd51ca82e72fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/615105c32fa1315d63e501e47f9869f0bc9a8af2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/d7995f5267eee2c63a531f77d539140753dbb690 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/19a0bd3e54b246439fa2deeaeeabfe5d1163dd03 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/a83bcb5f6a83791cdaec0a1e4bc44bdcb03ed028 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/15d72ad09f92dfdab2338406260ee039693dad8b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/185d3862ffc7bfda77c01aafb4b97096b6109a67 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/d41fd3e8f005daa169438299be16d201912b86d7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/b7e159594abd8b68ee8c746432dca29dd5a731fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/_ext/1360924651/clarke.o: ../foc/clarke.c  .generated_files/flags/default/48423236a8cc1d1a371743fe97a8d73a57e5e23d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/clarke.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/clarke.c  -o ${OBJECTDIR}/_ext/1360924651/clarke.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/clarke.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/park.o: ../foc/park.c  .generated_files/flags/default/4c32254adbd87be4eabcc26cc4ea1d92f17fc49 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/park.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/park.c  -o ${OBJECTDIR}/_ext/1360924651/park.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/park.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/svpwm.o: ../foc/svpwm.c  .generated_files/flags/default/2933426c25ea42c0ea2d6d2e258e4fb88961441c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/svpwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/svpwm.c  -o ${OBJECTDIR}/_ext/1360924651/svpwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/svpwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pi_controller.o: ../foc/pi_controller.c  .generated_files/flags/default/ea3ec6414b4d81fabd24bdaf0b158e2d24c32919 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pi_controller.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pi_controller.c  -o ${OBJECTDIR}/_ext/1360924651/pi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pi_controller.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/back_emf_obs.o: ../foc/back_emf_obs.c  .generated_files/flags/default/dc33194b1285665c3f71ab0d6d6613b5b2e7a5d5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/back_emf_obs.c  -o ${OBJECTDIR}/_ext/1360924651/back_emf_obs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/back_emf_obs.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/pll_estimator.o: ../foc/pll_estimator.c  .generated_files/flags/default/d4bdbb422e24905f5d51d92ba6f90e7d747bf850 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/pll_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/pll_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/pll_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/flux_estimator.o: ../foc/flux_estimator.c  .generated_files/flags/default/2e5efcd105db2191e2b68f9a30fdf8dbe25597be .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/flux_estimator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/flux_estimator.c  -o ${OBJECTDIR}/_ext/1360924651/flux_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/flux_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360924651/smo_observer.o: ../foc/smo_observer.c  .generated_files/flags/default/ae6ae30cb579dd186bae25db9d27a2dee7523386 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360924651" 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360924651/smo_observer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../foc/smo_observer.c  -o ${OBJECTDIR}/_ext/1360924651/smo_observer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360924651/smo_observer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp.o: ../gsp/gsp.c  .generated_files/flags/default/3bbbafa82312116c520b6fdb26d8f43ee6b4db79 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp.c  -o ${OBJECTDIR}/_ext/1360925749/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_params.o: ../gsp/gsp_params.c  .generated_files/flags/default/d891f025046902459b32ed7a7ea74514e28234ec .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_params.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_commands.o: ../gsp/gsp_commands.c  .generated_files/flags/default/830d80c4b1d565234d6c1909c49844844fc140e4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_commands.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_commands.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o: ../gsp/gsp_snapshot.c  .generated_files/flags/default/87037d3246871de7f8856d33adc6a160427146c5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360925749" 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../gsp/gsp_snapshot.c  -o ${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360925749/gsp_snapshot.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/b5d0b7980d69998ee8986422baff9cb4bdb10bfe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/11ec1f6ebc22c4d7f791f45d0227f203eaf802ed .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/c96d1ef871f701e25bb8cc029c6ff0e34c54bc41 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/9a2c59b77b61fa17223401c84d86b9f73f541260 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/95090643be945101b1ef7931d11861027f962fe8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/4c3088eb73d6c57009d8cbf23a9209921c3790e5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/fa5552f3f8685dadd6c51ad2e730e17482913220 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/c96107ebaad6ea48636b832ba19663c506d41621 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/6d2f7600badecf89e4e526c4ff7fb330120406d4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_timer.o: ../hal/hal_timer.c  .generated_files/flags/default/8e60e50512c23d67e5c550456ffb4fc226c0793b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_timer.c  -o ${OBJECTDIR}/_ext/1360926148/hal_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_timer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/2949335e7e115ebdf9b7d88a8da1a3866d461dd9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/6f3967906106aa8e6be2a18da0873aab26f8ac8e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/60a27eaed364ea404aa767b02d312bb1bdb1672e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/b1759796d181d992ddc0fae8636cadfcaa5179e1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/6eab96a92ecad1c21e86ce3aabe859cb7125edbc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/f4d474f9090aaa194df1f4b64f6f7b725dfe45a1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1798147462/diagnostics.o: ../x2cscope/diagnostics.c  .generated_files/flags/default/70bb2f3200e1af143bfceff4502482e57c211fa7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1798147462" 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o.d 
	@${RM} ${OBJECTDIR}/_ext/1798147462/diagnostics.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../x2cscope/diagnostics.c  -o ${OBJECTDIR}/_ext/1798147462/diagnostics.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1798147462/diagnostics.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/a03b3945b3f0b1d62ab4db47f3ab00ff2505e70d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/96e604489e122991df2c90de2b5140cb6dc5d3d7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/c05f71a77564bee717eeedf2f2296a99cce49a9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../foc" -I"../learn" -I"../gsp" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
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
