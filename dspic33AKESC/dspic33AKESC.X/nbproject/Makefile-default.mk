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
SOURCEFILES_QUOTED_IF_SPACED=../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../hal/eeprom.c ../garuda_service.c ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360926148/board_service.o.d ${OBJECTDIR}/_ext/1360926148/clock.o.d ${OBJECTDIR}/_ext/1360926148/device_config.o.d ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d ${OBJECTDIR}/_ext/1360926148/port_config.o.d ${OBJECTDIR}/_ext/1360926148/timer1.o.d ${OBJECTDIR}/_ext/1360926148/uart1.o.d ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d ${OBJECTDIR}/_ext/2111190731/quality.o.d ${OBJECTDIR}/_ext/2111190731/health.o.d ${OBJECTDIR}/_ext/2111190731/adaptation.o.d ${OBJECTDIR}/_ext/2111190731/commission.o.d ${OBJECTDIR}/_ext/2111190731/learn_service.o.d ${OBJECTDIR}/_ext/2109951130/commutation.o.d ${OBJECTDIR}/_ext/2109951130/pi.o.d ${OBJECTDIR}/_ext/2109951130/startup.o.d ${OBJECTDIR}/_ext/1360926148/eeprom.o.d ${OBJECTDIR}/_ext/1472/garuda_service.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360926148/board_service.o ${OBJECTDIR}/_ext/1360926148/clock.o ${OBJECTDIR}/_ext/1360926148/device_config.o ${OBJECTDIR}/_ext/1360926148/hal_adc.o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o ${OBJECTDIR}/_ext/1360926148/port_config.o ${OBJECTDIR}/_ext/1360926148/timer1.o ${OBJECTDIR}/_ext/1360926148/uart1.o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o ${OBJECTDIR}/_ext/2111190731/quality.o ${OBJECTDIR}/_ext/2111190731/health.o ${OBJECTDIR}/_ext/2111190731/adaptation.o ${OBJECTDIR}/_ext/2111190731/commission.o ${OBJECTDIR}/_ext/2111190731/learn_service.o ${OBJECTDIR}/_ext/2109951130/commutation.o ${OBJECTDIR}/_ext/2109951130/pi.o ${OBJECTDIR}/_ext/2109951130/startup.o ${OBJECTDIR}/_ext/1360926148/eeprom.o ${OBJECTDIR}/_ext/1472/garuda_service.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../hal/board_service.c ../hal/clock.c ../hal/device_config.c ../hal/hal_adc.c ../hal/hal_comparator.c ../hal/hal_pwm.c ../hal/port_config.c ../hal/timer1.c ../hal/uart1.c ../learn/ring_buffer.c ../learn/quality.c ../learn/health.c ../learn/adaptation.c ../learn/commission.c ../learn/learn_service.c ../motor/commutation.c ../motor/pi.c ../motor/startup.c ../hal/eeprom.c ../garuda_service.c ../main.c



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
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/f5f74e3901d90b6f02794514bc1e3ae88b55323f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/4ecd19fa933a12c9cec30dd58d5d7ce15db3d21c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/cb07567967ba59279cf5096a6d982e344edf0b73 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/79eae89f29dab0a8e5a67babddce13e9ad7fc985 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/96b9760ec582d8be74345ea603d25da5f351ff94 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/1d7a8340c7aa0202ccb64f3ecd23f712814ff0c5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/14382ed6fdbc992effb0024dfaae8e0207b55442 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/6a043a0219a99ac005cf3a719f145f0c86ff91aa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/3144972005f937c2eba4e56fa24c84b7f753134 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/e34f87a35f80a8c4c2e657277861a9baf0342ff1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/86bc372cdb536a5d9f6a2347c5ec6c6107451482 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/5e2ec8598f0db979ee3113c17ce7a18c72f8bb7f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/8eb90050a18fe6e1670aa408e507a55fc4ff8920 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/d80349408c033a3c9536b75eba22a72ff112d76c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/bad50364a5703f54f5c57414ce3597aacb67c9d3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/dc63719365480a7b229efd689ccaf77a84c2aefb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/695f29fe7d44a3ef957ee023a6dd28907c25079c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/61140a1a576e79d87c7a3cba7548608eebd78186 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/e5e1db0d131b79f7356bf91181beba3dd367cffe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/24e144443936bc87cbf0966cee9a79952f6cb262 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/b2e9b05ca8c8e248f0392cc138122cdb9cc57663 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/_ext/1360926148/board_service.o: ../hal/board_service.c  .generated_files/flags/default/dd5d374395ba0e8c2cafe44a797d5a767d3ffa8c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/board_service.c  -o ${OBJECTDIR}/_ext/1360926148/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/clock.o: ../hal/clock.c  .generated_files/flags/default/ce551cc37ef514f9b33ff4c20d5b178c69c2eaf9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/clock.c  -o ${OBJECTDIR}/_ext/1360926148/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/device_config.o: ../hal/device_config.c  .generated_files/flags/default/95ec127c593d1321d1d2935ab4c5d74ff804e616 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/device_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/device_config.c  -o ${OBJECTDIR}/_ext/1360926148/device_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/device_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_adc.o: ../hal/hal_adc.c  .generated_files/flags/default/50c66303abb55ad79f2fe06ddccc463941fa3634 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_adc.c  -o ${OBJECTDIR}/_ext/1360926148/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_comparator.o: ../hal/hal_comparator.c  .generated_files/flags/default/8d92da27a445ea04825c8aeeab0f081aae40e8a8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_comparator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_comparator.c  -o ${OBJECTDIR}/_ext/1360926148/hal_comparator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_comparator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/hal_pwm.o: ../hal/hal_pwm.c  .generated_files/flags/default/6cca4ed3685b2f23835d0474b24004035c6c2500 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/hal_pwm.c  -o ${OBJECTDIR}/_ext/1360926148/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/port_config.o: ../hal/port_config.c  .generated_files/flags/default/f46988d99309dce243f5c84cb092ecf728a661a5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/port_config.c  -o ${OBJECTDIR}/_ext/1360926148/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/timer1.o: ../hal/timer1.c  .generated_files/flags/default/c89f14b14fe7e251e31cd977b493e914747d385b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/timer1.c  -o ${OBJECTDIR}/_ext/1360926148/timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/uart1.o: ../hal/uart1.c  .generated_files/flags/default/60f226325bc49085457e65ab4bd3506a91421a16 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/uart1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/uart1.c  -o ${OBJECTDIR}/_ext/1360926148/uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/uart1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/ring_buffer.o: ../learn/ring_buffer.c  .generated_files/flags/default/700ac9be56c634f4ee20474b116c4bd3c321d436 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/ring_buffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/ring_buffer.c  -o ${OBJECTDIR}/_ext/2111190731/ring_buffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/ring_buffer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/quality.o: ../learn/quality.c  .generated_files/flags/default/90fd0673946aba02de0dd0e745cbd671950348fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/quality.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/quality.c  -o ${OBJECTDIR}/_ext/2111190731/quality.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/quality.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/health.o: ../learn/health.c  .generated_files/flags/default/e449768107600b8924331775f5496a0704b33de7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/health.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/health.c  -o ${OBJECTDIR}/_ext/2111190731/health.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/health.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/adaptation.o: ../learn/adaptation.c  .generated_files/flags/default/a2c69aeb9d0940333ccc4a3ad54190371d3e0b2d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/adaptation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/adaptation.c  -o ${OBJECTDIR}/_ext/2111190731/adaptation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/adaptation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/commission.o: ../learn/commission.c  .generated_files/flags/default/891b9edc63080d0a966443f54c46c4c0791ca63a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/commission.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/commission.c  -o ${OBJECTDIR}/_ext/2111190731/commission.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/commission.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2111190731/learn_service.o: ../learn/learn_service.c  .generated_files/flags/default/8118cf12a573cb800e6547d94255b8e11973b44a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2111190731" 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/2111190731/learn_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../learn/learn_service.c  -o ${OBJECTDIR}/_ext/2111190731/learn_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2111190731/learn_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/commutation.o: ../motor/commutation.c  .generated_files/flags/default/b87c7d73a099e8b40574847af09a1fd2846e8b96 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/commutation.c  -o ${OBJECTDIR}/_ext/2109951130/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/commutation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/pi.o: ../motor/pi.c  .generated_files/flags/default/4a5d0971864c75818cbb97dcd707b0eba32ab389 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/pi.c  -o ${OBJECTDIR}/_ext/2109951130/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/2109951130/startup.o: ../motor/startup.c  .generated_files/flags/default/5526ca44577823b54968833eae23c9a3588ba6f7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/2109951130" 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o.d 
	@${RM} ${OBJECTDIR}/_ext/2109951130/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../motor/startup.c  -o ${OBJECTDIR}/_ext/2109951130/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/2109951130/startup.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1360926148/eeprom.o: ../hal/eeprom.c  .generated_files/flags/default/10a9db2964e639079d90426315b0a6c487d6ba13 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360926148" 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360926148/eeprom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../hal/eeprom.c  -o ${OBJECTDIR}/_ext/1360926148/eeprom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1360926148/eeprom.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/garuda_service.o: ../garuda_service.c  .generated_files/flags/default/4588c322bb6881c26a8349b11a06b72e81c1d9f1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../garuda_service.c  -o ${OBJECTDIR}/_ext/1472/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/default/831976d609beb88c385f63e3dbe8535a44ae3765 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"../hal" -I"../motor" -I"../learn" -I".." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PKOB4=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)      -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PKOB4=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/dspic33AKESC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--stackguard=16,--ivt,--isr,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
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
