
    
.include "xc.inc" ; required "boiler-plate" (BP)
;the next two lines set up the actual chip for operation - required
;config __CONFIG2, POSCMOD_EC & I2C1SEL_SEC & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECME & FNOSC_FRCPLL & SOSCSEL_LPSOSC & WUTSEL_FST & IESO_OFF
;config __CONFIG1, WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & BKBUG_ON & GWRP_ON & GCP_ON & JTAGEN_OFF
   .text ; BP (put the following data in ROM(program memory))
   ;because we are using the C compiler to assemble our code, we need a "_main" label
   ;somewhere. (There's a link step that looks for it.)
.global _delay_ms, _delay_us
   
_delay_ms:
	    ; 2 for the function call
    repeat #15993 ;1
    nop     ; 15993 + 1
    return  ; 3
_delay_us:
	; 2 for the function call
    repeat #9 ;; 1
    nop ;; 12+1
    return ; 3



