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
FINAL_IMAGE=${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=main.c garuda_service.c hal/clock.c hal/port_config.c hal/hal_uart.c hal/hal_diag.c hal/hal_spi.c hal/hal_ata6847.c hal/hal_adc.c hal/hal_pwm.c hal/hal_timer1.c hal/hal_com_timer.c hal/hal_ptg.c hal/hal_trap.c hal/board_service.c motor/commutation.c motor/sector_pi.c motor/motor_params.c gsp/gsp.c gsp/gsp_commands.c gsp/gsp_params.c foc/an1078_motor.c foc/an1078_smc.c foc/pll_estimator.c foc/foc_runtime.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/garuda_service.o ${OBJECTDIR}/hal/clock.o ${OBJECTDIR}/hal/port_config.o ${OBJECTDIR}/hal/hal_uart.o ${OBJECTDIR}/hal/hal_diag.o ${OBJECTDIR}/hal/hal_spi.o ${OBJECTDIR}/hal/hal_ata6847.o ${OBJECTDIR}/hal/hal_adc.o ${OBJECTDIR}/hal/hal_pwm.o ${OBJECTDIR}/hal/hal_timer1.o ${OBJECTDIR}/hal/hal_com_timer.o ${OBJECTDIR}/hal/hal_ptg.o ${OBJECTDIR}/hal/hal_trap.o ${OBJECTDIR}/hal/board_service.o ${OBJECTDIR}/motor/commutation.o ${OBJECTDIR}/motor/sector_pi.o ${OBJECTDIR}/motor/motor_params.o ${OBJECTDIR}/gsp/gsp.o ${OBJECTDIR}/gsp/gsp_commands.o ${OBJECTDIR}/gsp/gsp_params.o ${OBJECTDIR}/foc/an1078_motor.o ${OBJECTDIR}/foc/an1078_smc.o ${OBJECTDIR}/foc/pll_estimator.o ${OBJECTDIR}/foc/foc_runtime.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/garuda_service.o.d ${OBJECTDIR}/hal/clock.o.d ${OBJECTDIR}/hal/port_config.o.d ${OBJECTDIR}/hal/hal_uart.o.d ${OBJECTDIR}/hal/hal_diag.o.d ${OBJECTDIR}/hal/hal_spi.o.d ${OBJECTDIR}/hal/hal_ata6847.o.d ${OBJECTDIR}/hal/hal_adc.o.d ${OBJECTDIR}/hal/hal_pwm.o.d ${OBJECTDIR}/hal/hal_timer1.o.d ${OBJECTDIR}/hal/hal_com_timer.o.d ${OBJECTDIR}/hal/hal_ptg.o.d ${OBJECTDIR}/hal/hal_trap.o.d ${OBJECTDIR}/hal/board_service.o.d ${OBJECTDIR}/motor/commutation.o.d ${OBJECTDIR}/motor/sector_pi.o.d ${OBJECTDIR}/motor/motor_params.o.d ${OBJECTDIR}/gsp/gsp.o.d ${OBJECTDIR}/gsp/gsp_commands.o.d ${OBJECTDIR}/gsp/gsp_params.o.d ${OBJECTDIR}/foc/an1078_motor.o.d ${OBJECTDIR}/foc/an1078_smc.o.d ${OBJECTDIR}/foc/pll_estimator.o.d ${OBJECTDIR}/foc/foc_runtime.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/garuda_service.o ${OBJECTDIR}/hal/clock.o ${OBJECTDIR}/hal/port_config.o ${OBJECTDIR}/hal/hal_uart.o ${OBJECTDIR}/hal/hal_diag.o ${OBJECTDIR}/hal/hal_spi.o ${OBJECTDIR}/hal/hal_ata6847.o ${OBJECTDIR}/hal/hal_adc.o ${OBJECTDIR}/hal/hal_pwm.o ${OBJECTDIR}/hal/hal_timer1.o ${OBJECTDIR}/hal/hal_com_timer.o ${OBJECTDIR}/hal/hal_ptg.o ${OBJECTDIR}/hal/hal_trap.o ${OBJECTDIR}/hal/board_service.o ${OBJECTDIR}/motor/commutation.o ${OBJECTDIR}/motor/sector_pi.o ${OBJECTDIR}/motor/motor_params.o ${OBJECTDIR}/gsp/gsp.o ${OBJECTDIR}/gsp/gsp_commands.o ${OBJECTDIR}/gsp/gsp_params.o ${OBJECTDIR}/foc/an1078_motor.o ${OBJECTDIR}/foc/an1078_smc.o ${OBJECTDIR}/foc/pll_estimator.o ${OBJECTDIR}/foc/foc_runtime.o

# Source Files
SOURCEFILES=main.c garuda_service.c hal/clock.c hal/port_config.c hal/hal_uart.c hal/hal_diag.c hal/hal_spi.c hal/hal_ata6847.c hal/hal_adc.c hal/hal_pwm.c hal/hal_timer1.c hal/hal_com_timer.c hal/hal_ptg.c hal/hal_trap.c hal/board_service.c motor/commutation.c motor/sector_pi.c motor/motor_params.c gsp/gsp.c gsp/gsp_commands.c gsp/gsp_params.c foc/an1078_motor.c foc/an1078_smc.c foc/pll_estimator.c foc/foc_runtime.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33AK128MC106
MP_LINKER_FILE_OPTION=,--script=p33AK128MC106.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/48ec7ae9d9067118750a626e1cabc3ccf62ec016 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/garuda_service.o: garuda_service.c  .generated_files/flags/default/6327edee0811894b6a3c4476d50876fd94e9d6c3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  garuda_service.c  -o ${OBJECTDIR}/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/garuda_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/clock.o: hal/clock.c  .generated_files/flags/default/d06c6a5fe9d283bc34c9c8d8ba0260f429b5c840 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/clock.o.d 
	@${RM} ${OBJECTDIR}/hal/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/clock.c  -o ${OBJECTDIR}/hal/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/clock.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/port_config.o: hal/port_config.c  .generated_files/flags/default/30f86770ba0f14c1e841a57ae67337955ada1f63 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/port_config.o.d 
	@${RM} ${OBJECTDIR}/hal/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/port_config.c  -o ${OBJECTDIR}/hal/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/port_config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_uart.o: hal/hal_uart.c  .generated_files/flags/default/51fad82b334c95ffb45235c1321887d453fc45d0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_uart.c  -o ${OBJECTDIR}/hal/hal_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_uart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_diag.o: hal/hal_diag.c  .generated_files/flags/default/1d231ab3b686d97fbe7a5ecf058d748043bca78e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_diag.c  -o ${OBJECTDIR}/hal/hal_diag.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_diag.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_spi.o: hal/hal_spi.c  .generated_files/flags/default/470dcc7fccb8ffbfe167590973d37516d276e85b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_spi.c  -o ${OBJECTDIR}/hal/hal_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_spi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_ata6847.o: hal/hal_ata6847.c  .generated_files/flags/default/d03d7b8d716b5d20d1fb9159c753e51c13f7adff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ata6847.c  -o ${OBJECTDIR}/hal/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ata6847.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_adc.o: hal/hal_adc.c  .generated_files/flags/default/e7efb26d81a8ceaee9df1af58e5f4ef161f7f053 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_adc.c  -o ${OBJECTDIR}/hal/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_pwm.o: hal/hal_pwm.c  .generated_files/flags/default/4584e3593956c88b2761511cefdc7ef5033b4eff .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_pwm.c  -o ${OBJECTDIR}/hal/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_pwm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_timer1.o: hal/hal_timer1.c  .generated_files/flags/default/7005df56f2d61bfc09f6ba4d3c2b5e8e438e2d5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_timer1.c  -o ${OBJECTDIR}/hal/hal_timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_timer1.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_com_timer.o: hal/hal_com_timer.c  .generated_files/flags/default/f92c09f6fae8e35ebb0492f5570430563424707 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_com_timer.c  -o ${OBJECTDIR}/hal/hal_com_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_com_timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_ptg.o: hal/hal_ptg.c  .generated_files/flags/default/e963a770d087acab2e28359642dbedf4a566c8c9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ptg.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ptg.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ptg.c  -o ${OBJECTDIR}/hal/hal_ptg.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ptg.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_trap.o: hal/hal_trap.c  .generated_files/flags/default/1b283b77fff7b7758a222adeed92d58e3af3da9a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_trap.c  -o ${OBJECTDIR}/hal/hal_trap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_trap.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/board_service.o: hal/board_service.c  .generated_files/flags/default/7109c1db708761c6679d007ffcc9dad6142ae7c9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/board_service.o.d 
	@${RM} ${OBJECTDIR}/hal/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/board_service.c  -o ${OBJECTDIR}/hal/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/board_service.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/commutation.o: motor/commutation.c  .generated_files/flags/default/a6ba5479f927393f46f73e226a534ab05b886617 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/commutation.o.d 
	@${RM} ${OBJECTDIR}/motor/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/commutation.c  -o ${OBJECTDIR}/motor/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/commutation.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/sector_pi.o: motor/sector_pi.c  .generated_files/flags/default/44cf981b35508724faaafd8b4842ade2fc4dddb3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/sector_pi.o.d 
	@${RM} ${OBJECTDIR}/motor/sector_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/sector_pi.c  -o ${OBJECTDIR}/motor/sector_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/sector_pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/motor_params.o: motor/motor_params.c  .generated_files/flags/default/7a6053ae3b515aca3a9a3ca24a902709ab04586f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/motor_params.o.d 
	@${RM} ${OBJECTDIR}/motor/motor_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/motor_params.c  -o ${OBJECTDIR}/motor/motor_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/motor_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp.o: gsp/gsp.c  .generated_files/flags/default/b6caf0c3cefaa7e2ab74c92011d78cd6e9191463 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp.c  -o ${OBJECTDIR}/gsp/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp_commands.o: gsp/gsp_commands.c  .generated_files/flags/default/34294e6cb3162e24d87903d9448a2730d8685fc4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_commands.c  -o ${OBJECTDIR}/gsp/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_commands.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp_params.o: gsp/gsp_params.c  .generated_files/flags/default/8f1fb9c008395a5624876f34eb319e79c9ff1094 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp"
	@${RM} ${OBJECTDIR}/gsp/gsp_params.o.d
	@${RM} ${OBJECTDIR}/gsp/gsp_params.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_params.c  -o ${OBJECTDIR}/gsp/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_params.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/an1078_motor.o: foc/an1078_motor.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/an1078_motor.o.d
	@${RM} ${OBJECTDIR}/foc/an1078_motor.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/an1078_motor.c  -o ${OBJECTDIR}/foc/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/an1078_motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/an1078_smc.o: foc/an1078_smc.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/an1078_smc.o.d
	@${RM} ${OBJECTDIR}/foc/an1078_smc.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/an1078_smc.c  -o ${OBJECTDIR}/foc/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/an1078_smc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/pll_estimator.o: foc/pll_estimator.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/pll_estimator.o.d
	@${RM} ${OBJECTDIR}/foc/pll_estimator.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/pll_estimator.c  -o ${OBJECTDIR}/foc/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/pll_estimator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/foc_runtime.o: foc/foc_runtime.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/foc_runtime.o.d
	@${RM} ${OBJECTDIR}/foc/foc_runtime.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/foc_runtime.c  -o ${OBJECTDIR}/foc/foc_runtime.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/foc_runtime.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK5=1    -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

else
${OBJECTDIR}/main.o: main.c  .generated_files/flags/default/601304e9828dfdc1e7ef925510f69c790c68a12c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/main.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/garuda_service.o: garuda_service.c  .generated_files/flags/default/d8e652ff260e7f3c3803783c48f157f343a8edc3 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/garuda_service.o.d 
	@${RM} ${OBJECTDIR}/garuda_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  garuda_service.c  -o ${OBJECTDIR}/garuda_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/garuda_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/clock.o: hal/clock.c  .generated_files/flags/default/4c508b10beb556d1d08cbe892cd21be7b09d3fad .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/clock.o.d 
	@${RM} ${OBJECTDIR}/hal/clock.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/clock.c  -o ${OBJECTDIR}/hal/clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/clock.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/port_config.o: hal/port_config.c  .generated_files/flags/default/3eaae39990a1ab45f6349432a809eb3a5a2221d1 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/port_config.o.d 
	@${RM} ${OBJECTDIR}/hal/port_config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/port_config.c  -o ${OBJECTDIR}/hal/port_config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/port_config.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_uart.o: hal/hal_uart.c  .generated_files/flags/default/2c7686d56c41a256bc89255c2c1a863d6f7b9912 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_uart.c  -o ${OBJECTDIR}/hal/hal_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_uart.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_diag.o: hal/hal_diag.c  .generated_files/flags/default/145b99e42aa4ff8646d98130c89a31f429a6c89e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_diag.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_diag.c  -o ${OBJECTDIR}/hal/hal_diag.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_diag.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_spi.o: hal/hal_spi.c  .generated_files/flags/default/9386d925c92d7317931f3ad9739d5c69548f4329 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_spi.c  -o ${OBJECTDIR}/hal/hal_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_spi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_ata6847.o: hal/hal_ata6847.c  .generated_files/flags/default/f1d6e1b9b6e7d740acdb922222fcf3c48cb2f8a0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ata6847.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ata6847.c  -o ${OBJECTDIR}/hal/hal_ata6847.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ata6847.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_adc.o: hal/hal_adc.c  .generated_files/flags/default/94be9a75435526cedbdc23fdfbeb338bfa26f7b8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_adc.c  -o ${OBJECTDIR}/hal/hal_adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_pwm.o: hal/hal_pwm.c  .generated_files/flags/default/5543427db6a3f82b740ecebd3c4f90bdf0c266d0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_pwm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_pwm.c  -o ${OBJECTDIR}/hal/hal_pwm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_pwm.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_timer1.o: hal/hal_timer1.c  .generated_files/flags/default/2c69dcd059a5d72d6c3c3816bb3d3ca9ca9aabb8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_timer1.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_timer1.c  -o ${OBJECTDIR}/hal/hal_timer1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_timer1.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_com_timer.o: hal/hal_com_timer.c  .generated_files/flags/default/cff5c0cc78fc7d5a0f69319a17a14d40cadf6541 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_com_timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_com_timer.c  -o ${OBJECTDIR}/hal/hal_com_timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_com_timer.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_ptg.o: hal/hal_ptg.c  .generated_files/flags/default/f26c19b4913ff1175b77c6d2e88f70d8f38e55ef .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_ptg.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_ptg.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_ptg.c  -o ${OBJECTDIR}/hal/hal_ptg.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_ptg.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/hal_trap.o: hal/hal_trap.c  .generated_files/flags/default/f09d882f0365307ac70b9a328a4d399af8de27c6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o.d 
	@${RM} ${OBJECTDIR}/hal/hal_trap.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/hal_trap.c  -o ${OBJECTDIR}/hal/hal_trap.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/hal_trap.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/hal/board_service.o: hal/board_service.c  .generated_files/flags/default/c4dc1afc4c60a5b938009c18f58005e6d6b76423 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/hal" 
	@${RM} ${OBJECTDIR}/hal/board_service.o.d 
	@${RM} ${OBJECTDIR}/hal/board_service.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hal/board_service.c  -o ${OBJECTDIR}/hal/board_service.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/hal/board_service.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/commutation.o: motor/commutation.c  .generated_files/flags/default/1bf47a7fe28a44b083f0e79a6ab236677bb64aee .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/commutation.o.d 
	@${RM} ${OBJECTDIR}/motor/commutation.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/commutation.c  -o ${OBJECTDIR}/motor/commutation.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/commutation.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/sector_pi.o: motor/sector_pi.c  .generated_files/flags/default/dc57ee1f35ec76dbfdf751d6dba1268491421864 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/sector_pi.o.d 
	@${RM} ${OBJECTDIR}/motor/sector_pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/sector_pi.c  -o ${OBJECTDIR}/motor/sector_pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/sector_pi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/motor/motor_params.o: motor/motor_params.c  .generated_files/flags/default/fe28263096587d4920f614b6df33b70bd317d0b9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/motor" 
	@${RM} ${OBJECTDIR}/motor/motor_params.o.d 
	@${RM} ${OBJECTDIR}/motor/motor_params.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  motor/motor_params.c  -o ${OBJECTDIR}/motor/motor_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/motor/motor_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp.o: gsp/gsp.c  .generated_files/flags/default/2efd250a0f3a90ecac8e459910b00ce30eb77c23 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp.c  -o ${OBJECTDIR}/gsp/gsp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp_commands.o: gsp/gsp_commands.c  .generated_files/flags/default/20b939bd1b4804d17fa92feabd18695508a92bbe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp" 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o.d 
	@${RM} ${OBJECTDIR}/gsp/gsp_commands.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_commands.c  -o ${OBJECTDIR}/gsp/gsp_commands.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_commands.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/gsp/gsp_params.o: gsp/gsp_params.c  .generated_files/flags/default/b277370d995be85386a5e3f0af1b0d894f16ef7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/gsp"
	@${RM} ${OBJECTDIR}/gsp/gsp_params.o.d
	@${RM} ${OBJECTDIR}/gsp/gsp_params.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  gsp/gsp_params.c  -o ${OBJECTDIR}/gsp/gsp_params.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/gsp/gsp_params.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/an1078_motor.o: foc/an1078_motor.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/an1078_motor.o.d
	@${RM} ${OBJECTDIR}/foc/an1078_motor.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/an1078_motor.c  -o ${OBJECTDIR}/foc/an1078_motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/an1078_motor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/an1078_smc.o: foc/an1078_smc.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/an1078_smc.o.d
	@${RM} ${OBJECTDIR}/foc/an1078_smc.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/an1078_smc.c  -o ${OBJECTDIR}/foc/an1078_smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/an1078_smc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/pll_estimator.o: foc/pll_estimator.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/pll_estimator.o.d
	@${RM} ${OBJECTDIR}/foc/pll_estimator.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/pll_estimator.c  -o ${OBJECTDIR}/foc/pll_estimator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/pll_estimator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

${OBJECTDIR}/foc/foc_runtime.o: foc/foc_runtime.c  .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/foc"
	@${RM} ${OBJECTDIR}/foc/foc_runtime.o.d
	@${RM} ${OBJECTDIR}/foc/foc_runtime.o
	${MP_CC} $(MP_EXTRA_CC_PRE)  foc/foc_runtime.c  -o ${OBJECTDIR}/foc/foc_runtime.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/foc/foc_runtime.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O1 -I"hal" -I"motor" -I"gsp" -I"foc" -I"." -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"

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
${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_PK5=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)      -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_PK5=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--fill-upper=0,--stackguard=16,--ivt,--isr,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-gc-sections,--fill-upper=0,--stackguard=16,--ivt,--isr,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}/xc-dsc-bin2hex ${DISTDIR}/garuda_ata6847_ak.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
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
