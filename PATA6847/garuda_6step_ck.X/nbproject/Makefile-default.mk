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
FINAL_IMAGE=${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=main.c garuda_service.c hal/clock.c hal/port_config.c hal/hal_uart.c hal/hal_diag.c hal/hal_spi.c hal/hal_ata6847.c hal/hal_adc.c hal/hal_opa.c hal/hal_pwm.c hal/hal_timer1.c hal/hal_ic.c hal/hal_com_timer.c hal/hal_clc.c hal/hal_trap.c hal/board_service.c motor/commutation.c motor/startup.c motor/bemf_zc.c gsp/gsp.c gsp/gsp_commands.c gsp/gsp_snapshot.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/garuda_service.o ${OBJECTDIR}/hal/clock.o ${OBJECTDIR}/hal/port_config.o ${OBJECTDIR}/hal/hal_uart.o ${OBJECTDIR}/hal/hal_diag.o ${OBJECTDIR}/hal/hal_spi.o ${OBJECTDIR}/hal/hal_ata6847.o ${OBJECTDIR}/hal/hal_adc.o ${OBJECTDIR}/hal/hal_opa.o ${OBJECTDIR}/hal/hal_pwm.o ${OBJECTDIR}/hal/hal_timer1.o ${OBJECTDIR}/hal/hal_ic.o ${OBJECTDIR}/hal/hal_com_timer.o ${OBJECTDIR}/hal/hal_clc.o ${OBJECTDIR}/hal/hal_trap.o ${OBJECTDIR}/hal/board_service.o ${OBJECTDIR}/motor/commutation.o ${OBJECTDIR}/motor/startup.o ${OBJECTDIR}/motor/bemf_zc.o ${OBJECTDIR}/gsp/gsp.o ${OBJECTDIR}/gsp/gsp_commands.o ${OBJECTDIR}/gsp/gsp_snapshot.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/garuda_service.o.d ${OBJECTDIR}/hal/clock.o.d ${OBJECTDIR}/hal/port_config.o.d ${OBJECTDIR}/hal/hal_uart.o.d ${OBJECTDIR}/hal/hal_diag.o.d ${OBJECTDIR}/hal/hal_spi.o.d ${OBJECTDIR}/hal/hal_ata6847.o.d ${OBJECTDIR}/hal/hal_adc.o.d ${OBJECTDIR}/hal/hal_opa.o.d ${OBJECTDIR}/hal/hal_pwm.o.d ${OBJECTDIR}/hal/hal_timer1.o.d ${OBJECTDIR}/hal/hal_ic.o.d ${OBJECTDIR}/hal/hal_com_timer.o.d ${OBJECTDIR}/hal/hal_clc.o.d ${OBJECTDIR}/hal/hal_trap.o.d ${OBJECTDIR}/hal/board_service.o.d ${OBJECTDIR}/motor/commutation.o.d ${OBJECTDIR}/motor/startup.o.d ${OBJECTDIR}/motor/bemf_zc.o.d ${OBJECTDIR}/gsp/gsp.o.d ${OBJECTDIR}/gsp/gsp_commands.o.d ${OBJECTDIR}/gsp/gsp_snapshot.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/garuda_service.o ${OBJECTDIR}/hal/clock.o ${OBJECTDIR}/hal/port_config.o ${OBJECTDIR}/hal/hal_uart.o ${OBJECTDIR}/hal/hal_diag.o ${OBJECTDIR}/hal/hal_spi.o ${OBJECTDIR}/hal/hal_ata6847.o ${OBJECTDIR}/hal/hal_adc.o ${OBJECTDIR}/hal/hal_opa.o ${OBJECTDIR}/hal/hal_pwm.o ${OBJECTDIR}/hal/hal_timer1.o ${OBJECTDIR}/hal/hal_ic.o ${OBJECTDIR}/hal/hal_com_timer.o ${OBJECTDIR}/hal/hal_clc.o ${OBJECTDIR}/hal/hal_trap.o ${OBJECTDIR}/hal/board_service.o ${OBJECTDIR}/motor/commutation.o ${OBJECTDIR}/motor/startup.o ${OBJECTDIR}/motor/bemf_zc.o ${OBJECTDIR}/gsp/gsp.o ${OBJECTDIR}/gsp/gsp_commands.o ${OBJECTDIR}/gsp/gsp_snapshot.o

# Source Files
SOURCEFILES=main.c garuda_service.c hal/clock.c hal/port_config.c hal/hal_uart.c hal/hal_diag.c hal/hal_spi.c hal/hal_ata6847.c hal/hal_adc.c hal/hal_opa.c hal/hal_pwm.c hal/hal_timer1.c hal/hal_ic.c hal/hal_com_timer.c hal/hal_clc.c hal/hal_trap.c hal/board_service.c motor/commutation.c motor/startup.c motor/bemf_zc.c gsp/gsp.c gsp/gsp_commands.c gsp/gsp_snapshot.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33CK64MP205
MP_LINKER_FILE_OPTION=,--script=p33CK64MP205.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/931eeb968be4d3622a1d3fa40ec75f8f72facc46 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/garuda_service.o: garuda_service.c  .generated_files/flags/default/cc84fccffe4a8c20d8bb2e2c197afcc805fafeea .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  garuda_service.c  -o ${OBJECTDIR}/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/clock.o: hal/clock.c  .generated_files/flags/default/f863a9860d480d1fb1e0914f7fb85332c92aaeaa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/clock.o.d 
	@${RM} ${OBJECTDIR}/hal/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/clock.c  -o ${OBJECTDIR}/hal/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/port_config.o: hal/port_config.c  .generated_files/flags/default/cc333e0c08aa018466a1f12853c6045bd4d43d8b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/port_config.o.d 
	@${RM} ${OBJECTDIR}/hal/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/port_config.c  -o ${OBJECTDIR}/hal/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_uart.o: hal/hal_uart.c  .generated_files/flags/default/cb6f626253fb004d3929a55bc2b8f86a430f4405 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_uart.c  -o ${OBJECTDIR}/hal/hal_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_uart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_diag.o: hal/hal_diag.c  .generated_files/flags/default/aabacaeecd0c3a493c9d588baee28817221e8cff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_diag.c  -o ${OBJECTDIR}/hal/hal_diag.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_diag.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_spi.o: hal/hal_spi.c  .generated_files/flags/default/ea3382adfb12a3bcad8d30b3f952de4823fddbce .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_spi.c  -o ${OBJECTDIR}/hal/hal_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_spi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_ata6847.o: hal/hal_ata6847.c  .generated_files/flags/default/464dfb621a19068b6efc20ba35b0c7a093d79946 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ata6847.c  -o ${OBJECTDIR}/hal/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ata6847.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_adc.o: hal/hal_adc.c  .generated_files/flags/default/8d254ee6c5ab56f6c1b6196b3c5f3d3141f0a12b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_adc.c  -o ${OBJECTDIR}/hal/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_opa.o: hal/hal_opa.c  .generated_files/flags/default/a78fce22893de01d15c6a2190e1818614bd8e346 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_opa.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_opa.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_opa.c  -o ${OBJECTDIR}/hal/hal_opa.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_opa.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_pwm.o: hal/hal_pwm.c  .generated_files/flags/default/d3aa2816e74b08176d4a1dddef96263385850501 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_pwm.c  -o ${OBJECTDIR}/hal/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_timer1.o: hal/hal_timer1.c  .generated_files/flags/default/ed4276465aad8ae34c5838b582aa3ef264daa022 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_timer1.c  -o ${OBJECTDIR}/hal/hal_timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_ic.o: hal/hal_ic.c  .generated_files/flags/default/8ed5a6cfacca5a60bc8ae781474a5dc101d48036 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ic.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ic.c  -o ${OBJECTDIR}/hal/hal_ic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ic.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_com_timer.o: hal/hal_com_timer.c  .generated_files/flags/default/8f20192fe3f369109234829d341f97b12263791a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_com_timer.c  -o ${OBJECTDIR}/hal/hal_com_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_com_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_clc.o: hal/hal_clc.c  .generated_files/flags/default/eaae6cc33a66d36a9e96d3fd3ea480b11a69d7b8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_clc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_clc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_clc.c  -o ${OBJECTDIR}/hal/hal_clc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_clc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_trap.o: hal/hal_trap.c  .generated_files/flags/default/26e19f1f453b3030df74bd9d645dc933ad90a6ba .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_trap.c  -o ${OBJECTDIR}/hal/hal_trap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_trap.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/board_service.o: hal/board_service.c  .generated_files/flags/default/a39f9f484e3953cad5b10676917237e47e32f838 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/board_service.o.d 
	@${RM} ${OBJECTDIR}/hal/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/board_service.c  -o ${OBJECTDIR}/hal/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/commutation.o: motor/commutation.c  .generated_files/flags/default/3ac4ba22cbe4417db48bce7caedf08eae9bcb788 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/commutation.o.d 
	@${RM} ${OBJECTDIR}/motor/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/commutation.c  -o ${OBJECTDIR}/motor/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/startup.o: motor/startup.c  .generated_files/flags/default/e7434da18e03a2c00d92aaaf408aca66a964dc3b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/startup.o.d 
	@${RM} ${OBJECTDIR}/motor/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/startup.c  -o ${OBJECTDIR}/motor/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/startup.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/bemf_zc.o: motor/bemf_zc.c  .generated_files/flags/default/5807239308dbd252cd227a8657cd693168b13808 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/motor/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/bemf_zc.c  -o ${OBJECTDIR}/motor/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/bemf_zc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp.o: gsp/gsp.c  .generated_files/flags/default/9cb06544e652b42a65aad322085616fe6068096 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp.c  -o ${OBJECTDIR}/gsp/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp_commands.o: gsp/gsp_commands.c  .generated_files/flags/default/faf01d72cdfd2b81b45b6385e0c1470b09ccff75 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_commands.c  -o ${OBJECTDIR}/gsp/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp_snapshot.o: gsp/gsp_snapshot.c  .generated_files/flags/default/549b82b16e6f1345a8f2215895beea2b189d694f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_snapshot.c  -o ${OBJECTDIR}/gsp/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_snapshot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
else
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/c91184e68b13327efde5182ea7d96a52d3a2ff2e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/garuda_service.o: garuda_service.c  .generated_files/flags/default/62c3b36c23c14578fc3834d61eff56d742580443 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  garuda_service.c  -o ${OBJECTDIR}/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/garuda_service.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/clock.o: hal/clock.c  .generated_files/flags/default/780eb7d58cfbb4e390aa044908d780f2a2692648 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/clock.o.d 
	@${RM} ${OBJECTDIR}/hal/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/clock.c  -o ${OBJECTDIR}/hal/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/clock.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/port_config.o: hal/port_config.c  .generated_files/flags/default/b5a2ae2a7b2a09dfdb3f846a1959b2c1678c5701 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/port_config.o.d 
	@${RM} ${OBJECTDIR}/hal/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/port_config.c  -o ${OBJECTDIR}/hal/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/port_config.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_uart.o: hal/hal_uart.c  .generated_files/flags/default/6c288dd319e320dd9183081e8a6a7c8864fe8168 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_uart.c  -o ${OBJECTDIR}/hal/hal_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_uart.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_diag.o: hal/hal_diag.c  .generated_files/flags/default/a4c9071e5e85d7c0e5cd557a1d8fa6d533322926 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_diag.c  -o ${OBJECTDIR}/hal/hal_diag.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_diag.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_spi.o: hal/hal_spi.c  .generated_files/flags/default/1e9a9717e9898ae062ba6e3a5feeaf944f784ca9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_spi.c  -o ${OBJECTDIR}/hal/hal_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_spi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_ata6847.o: hal/hal_ata6847.c  .generated_files/flags/default/d02023b95c374c5862586ecb531aa5230a522a6b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ata6847.c  -o ${OBJECTDIR}/hal/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ata6847.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_adc.o: hal/hal_adc.c  .generated_files/flags/default/8a72d190cce17370f0079799dc48e3d717f6f9e3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_adc.c  -o ${OBJECTDIR}/hal/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_adc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_opa.o: hal/hal_opa.c  .generated_files/flags/default/2a239946261e6bd4f575940537389d19bd58a5dd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_opa.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_opa.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_opa.c  -o ${OBJECTDIR}/hal/hal_opa.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_opa.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_pwm.o: hal/hal_pwm.c  .generated_files/flags/default/8db83c69fedb1d55ebb8b75acdfd8921452d0a12 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_pwm.c  -o ${OBJECTDIR}/hal/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_pwm.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_timer1.o: hal/hal_timer1.c  .generated_files/flags/default/2f309a149c6ec5cc2d767d713e2b9aca152f4642 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_timer1.c  -o ${OBJECTDIR}/hal/hal_timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_timer1.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_ic.o: hal/hal_ic.c  .generated_files/flags/default/64c29179ae74d27cbc3529b9793307af4df6d284 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ic.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ic.c  -o ${OBJECTDIR}/hal/hal_ic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ic.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_com_timer.o: hal/hal_com_timer.c  .generated_files/flags/default/b7e0dfca89aa06964fbc2329095dffef7eb5672a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_com_timer.c  -o ${OBJECTDIR}/hal/hal_com_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_com_timer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_clc.o: hal/hal_clc.c  .generated_files/flags/default/49356239ceb293dad69d40566470e3764c0c4d92 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_clc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_clc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_clc.c  -o ${OBJECTDIR}/hal/hal_clc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_clc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/hal_trap.o: hal/hal_trap.c  .generated_files/flags/default/307f253d8f130f3272e1f7b1b691c3bff43a6a03 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_trap.c  -o ${OBJECTDIR}/hal/hal_trap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_trap.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/hal/board_service.o: hal/board_service.c  .generated_files/flags/default/25cfcf9e9a578dd4d840f1e585db325fa99f1c40 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/board_service.o.d 
	@${RM} ${OBJECTDIR}/hal/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/board_service.c  -o ${OBJECTDIR}/hal/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/board_service.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/commutation.o: motor/commutation.c  .generated_files/flags/default/3d971bf75289d6b13d1290237b339c7dffb3d922 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/commutation.o.d 
	@${RM} ${OBJECTDIR}/motor/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/commutation.c  -o ${OBJECTDIR}/motor/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/commutation.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/startup.o: motor/startup.c  .generated_files/flags/default/54de1ef5c3d3d22fc4798697e1eb90b00dbd8089 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/startup.o.d 
	@${RM} ${OBJECTDIR}/motor/startup.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/startup.c  -o ${OBJECTDIR}/motor/startup.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/startup.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/motor/bemf_zc.o: motor/bemf_zc.c  .generated_files/flags/default/38cf65dd51504445f12e5df0be4ea49084b5c3ff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/bemf_zc.o.d 
	@${RM} ${OBJECTDIR}/motor/bemf_zc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/bemf_zc.c  -o ${OBJECTDIR}/motor/bemf_zc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/bemf_zc.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp.o: gsp/gsp.c  .generated_files/flags/default/5c17156fc78f48d1f1f19f4485875d2394797e82 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp.c  -o ${OBJECTDIR}/gsp/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp_commands.o: gsp/gsp_commands.c  .generated_files/flags/default/39e0c5d1dd2539cf94df2257c13dc6aef930098b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_commands.c  -o ${OBJECTDIR}/gsp/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_commands.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/gsp/gsp_snapshot.o: gsp/gsp_snapshot.c  .generated_files/flags/default/54c2954b168ef39814faa9eaedbdf6156b74cd67 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_snapshot.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_snapshot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_snapshot.c  -o ${OBJECTDIR}/gsp/gsp_snapshot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_snapshot.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -msmart-io=1 -Wall -msfr-warn=off   
	
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
${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK5=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	
else
${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	${MP_CC_DIR}/xc16-bin2hex ${DISTDIR}/garuda_6step_ck.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   
	
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
