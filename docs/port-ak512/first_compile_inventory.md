# AK512MC510 first-compile error inventory (2026-06-12)

90 errors, by file:
```
     45 hal_pwm.c
     15 hal_adc.c
      9 board_service.c
      8 hal_comparator.c
      4 hwzc.c
      4 hal_pwm.h
      3 port_config.c
      2 garuda_service.c
```

Full list:
```
../hal/board_service.c:194:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/board_service.c:196:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/board_service.c:198:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/board_service.c:217:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/board_service.c:218:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/board_service.c:219:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/board_service.c:234:5: error: 'PG1FPCIbits' undeclared (first use in this function); did you mean 'PG1SPCI1bits'?
../hal/board_service.c:235:5: error: 'PG2FPCIbits' undeclared (first use in this function); did you mean 'PG2SPCI1bits'?
../hal/board_service.c:236:5: error: 'PG3FPCIbits' undeclared (first use in this function); did you mean 'PG3SPCI1bits'?
../hal/hal_adc.c:49:5: error: 'AD1CH0CONbits' undeclared (first use in this function); did you mean 'AD1CH0CON2bits'?
../hal/hal_adc.c:55:5: error: 'AD2CH0CONbits' undeclared (first use in this function); did you mean 'AD2CH0CON2bits'?
../hal/hal_adc.c:62:5: error: 'AD1CH1CONbits' undeclared (first use in this function); did you mean 'AD1CH1CON2bits'?
../hal/hal_adc.c:68:5: error: 'AD1CH4CONbits' undeclared (first use in this function); did you mean 'AD1CH4CON2bits'?
../hal/hal_adc.c:75:5: error: 'AD1CH2CONbits' undeclared (first use in this function); did you mean 'AD1CH2CON2bits'?
../hal/hal_adc.c:96:5: error: 'AD1CH5CONbits' undeclared (first use in this function); did you mean 'AD1CH5CON2bits'?
../hal/hal_adc.c:109:5: error: 'AD2CH1CONbits' undeclared (first use in this function); did you mean 'AD2CH1CON2bits'?
../hal/hal_adc.c:152:5: error: 'AD1CH3CONbits' undeclared (first use in this function); did you mean 'AD1CH3CON2bits'?
../hal/hal_adc.c:160:5: error: 'AD2CH2CONbits' undeclared (first use in this function); did you mean 'AD2CH2CON2bits'?
../hal/hal_adc.c:209:17: error: 'AD2CH0CONbits' undeclared (first use in this function); did you mean 'AD2CH0CON2bits'?
../hal/hal_adc.c:261:9: error: 'AD1CH5CONbits' undeclared (first use in this function); did you mean 'AD1CH5CON2bits'?
../hal/hal_adc.c:266:9: error: 'AD2CH1CONbits' undeclared (first use in this function); did you mean 'AD2CH1CON2bits'?
../hal/hal_adc.c:330:24: error: 'AD1CMPSTATBITS' {aka 'volatile struct tagAD1CMPSTATBITS'} has no member named 'CH5CMP'; did you mean 'CH5FLG'?
../hal/hal_adc.c:335:24: error: 'AD2CMPSTATBITS' {aka 'volatile struct tagAD2CMPSTATBITS'} has no member named 'CH1CMP'; did you mean 'CH1FLG'?
../hal/hal_adc.c:346:5: error: 'AD2CH1CONbits' undeclared (first use in this function); did you mean 'AD2CH1CON2bits'?
../hal/hal_comparator.c:196:16: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'FLTREN'
../hal/hal_comparator.c:197:16: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'CMPPOL'
../hal/hal_comparator.c:198:16: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'INSEL'
../hal/hal_comparator.c:199:16: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'HYSPOL'
../hal/hal_comparator.c:200:16: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'HYSSEL'
../hal/hal_comparator.c:296:35: error: 'DAC1CONBITS' {aka 'volatile struct tagDAC1CONBITS'} has no member named 'CMPSTAT'
../hal/hal_comparator.c:297:35: error: 'DAC2CONBITS' {aka 'volatile struct tagDAC2CONBITS'} has no member named 'CMPSTAT'
../hal/hal_comparator.c:298:35: error: 'DAC3CONBITS' {aka 'volatile struct tagDAC3CONBITS'} has no member named 'CMPSTAT'
../hal/hal_pwm.c:98:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/hal_pwm.c:99:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:100:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:125:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/hal_pwm.c:126:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:127:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:197:5: error: 'PG1IOCON' undeclared (first use in this function); did you mean 'PG1IOCON2'?
../hal/hal_pwm.c:199:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:219:5: error: 'PG1EVT' undeclared (first use in this function); did you mean 'PG1EVT2'?
../hal/hal_pwm.c:220:5: error: 'PG1EVTbits' undeclared (first use in this function); did you mean 'PG1EVT2bits'?
../hal/hal_pwm.c:248:5: error: 'PG1FPCI' undeclared (first use in this function); did you mean 'PG1SPCI2'?
../hal/hal_pwm.c:249:5: error: 'PG1FPCIbits' undeclared (first use in this function); did you mean 'PG1SPCI1bits'?
../hal/hal_pwm.c:294:5: error: 'PG1CLPCI' undeclared (first use in this function); did you mean 'PG1CLPCI2'?
../hal/hal_pwm.c:296:5: error: 'PG1FFPCI' undeclared (first use in this function); did you mean 'PG1FFPCI2'?
../hal/hal_pwm.c:297:5: error: 'PG1SPCI' undeclared (first use in this function); did you mean 'PG1SPCI2'?
../hal/hal_pwm.c:344:5: error: 'PG2IOCON' undeclared (first use in this function); did you mean 'PG2IOCON2'?
../hal/hal_pwm.c:346:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:365:5: error: 'PG2EVT' undeclared (first use in this function); did you mean 'PG2EVT2'?
../hal/hal_pwm.c:366:5: error: 'PG2EVTbits' undeclared (first use in this function); did you mean 'PG2EVT2bits'?
../hal/hal_pwm.c:384:5: error: 'PG2FPCI' undeclared (first use in this function); did you mean 'PG2SPCI2'?
../hal/hal_pwm.c:385:5: error: 'PG2FPCIbits' undeclared (first use in this function); did you mean 'PG2SPCI1bits'?
../hal/hal_pwm.c:422:5: error: 'PG2CLPCI' undeclared (first use in this function); did you mean 'PG2CLPCI2'?
../hal/hal_pwm.c:424:5: error: 'PG2FFPCI' undeclared (first use in this function); did you mean 'PG2FFPCI2'?
../hal/hal_pwm.c:425:5: error: 'PG2SPCI' undeclared (first use in this function); did you mean 'PG2SPCI2'?
../hal/hal_pwm.c:469:5: error: 'PG3IOCON' undeclared (first use in this function); did you mean 'PG3IOCON2'?
../hal/hal_pwm.c:471:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/hal_pwm.c:490:5: error: 'PG3EVT' undeclared (first use in this function); did you mean 'PG3EVT2'?
../hal/hal_pwm.c:491:5: error: 'PG3EVTbits' undeclared (first use in this function); did you mean 'PG3EVT2bits'?
../hal/hal_pwm.c:509:5: error: 'PG3FPCI' undeclared (first use in this function); did you mean 'PG3SPCI2'?
../hal/hal_pwm.c:510:5: error: 'PG3FPCIbits' undeclared (first use in this function); did you mean 'PG3SPCI1bits'?
../hal/hal_pwm.c:547:5: error: 'PG3CLPCI' undeclared (first use in this function); did you mean 'PG3CLPCI2'?
../hal/hal_pwm.c:549:5: error: 'PG3FFPCI' undeclared (first use in this function); did you mean 'PG3FFPCI2'?
../hal/hal_pwm.c:550:5: error: 'PG3SPCI' undeclared (first use in this function); did you mean 'PG3SPCI2'?
../hal/hal_pwm.c:639:5: error: 'PG1IOCON' undeclared (first use in this function); did you mean 'PG1IOCON2'?
../hal/hal_pwm.c:640:5: error: 'PG2IOCON' undeclared (first use in this function); did you mean 'PG2IOCON2'?
../hal/hal_pwm.c:641:5: error: 'PG3IOCON' undeclared (first use in this function); did you mean 'PG3IOCON2'?
../hal/hal_pwm.c:717:5: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/hal_pwm.c:719:5: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:721:5: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:729:17: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:730:17: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:731:17: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/hal_pwm.c:740:13: error: 'PG1IOCONbits' undeclared (first use in this function); did you mean 'PG1IOCON2bits'?
../hal/hal_pwm.c:743:13: error: 'PG2IOCONbits' undeclared (first use in this function); did you mean 'PG2IOCON2bits'?
../hal/hal_pwm.c:746:13: error: 'PG3IOCONbits' undeclared (first use in this function); did you mean 'PG3IOCON2bits'?
../hal/port_config.c:216:18: error: 'AMP1CON1BITS' {aka 'volatile struct tagAMP1CON1BITS'} has no member named 'OMONEN'; did you mean 'AMPEN'?
../hal/port_config.c:226:18: error: 'AMP2CON1BITS' {aka 'volatile struct tagAMP2CON1BITS'} has no member named 'OMONEN'; did you mean 'AMPEN'?
../hal/port_config.c:245:18: error: 'AMP3CON1BITS' {aka 'volatile struct tagAMP3CON1BITS'} has no member named 'OMONEN'; did you mean 'AMPEN'?
../motor/hwzc.c:1050:40: error: 'AD1CH5CONbits' undeclared (first use in this function); did you mean 'AD1CH5CON2bits'?
../motor/hwzc.c:1051:56: error: 'AD1CMPSTATBITS' {aka 'volatile struct tagAD1CMPSTATBITS'} has no member named 'CH5CMP'; did you mean 'CH5FLG'?
../motor/hwzc.c:1057:40: error: 'AD2CH1CONbits' undeclared (first use in this function); did you mean 'AD2CH1CON2bits'?
../motor/hwzc.c:1058:56: error: 'AD2CMPSTATBITS' {aka 'volatile struct tagAD2CMPSTATBITS'} has no member named 'CH1CMP'; did you mean 'CH1FLG'?
../hal/hal_pwm.h:72:43: error: 'PG1STATBITS' {aka 'volatile struct tagPG1STATBITS'} has no member named 'FLTEVT'; did you mean 'FLT1EVT'?
../hal/hal_pwm.h:73:43: error: 'PG2STATBITS' {aka 'volatile struct tagPG2STATBITS'} has no member named 'FLTEVT'; did you mean 'FLT1EVT'?
../hal/hal_pwm.h:74:43: error: 'PG3STATBITS' {aka 'volatile struct tagPG3STATBITS'} has no member named 'FLTEVT'; did you mean 'FLT1EVT'?
../hal/hal_pwm.h:50:49: error: 'PG1STATBITS' {aka 'volatile struct tagPG1STATBITS'} has no member named 'FLTACT'; did you mean 'FLT1ACT'?
../garuda_service.c:4597:20: error: 'AD1CMPSTATBITS' {aka 'volatile struct tagAD1CMPSTATBITS'} has no member named 'CH5CMP'; did you mean 'CH5FLG'?
../garuda_service.c:4609:20: error: 'AD2CMPSTATBITS' {aka 'volatile struct tagAD2CMPSTATBITS'} has no member named 'CH1CMP'; did you mean 'CH1FLG'?
```
