	  PRESERVE8
    THUMB
    
;    AREA APPRESET, CODE, READONLY
;
;Reset_Handler PROC             ; Special case: Do BOTH stack and application reset handler
;    EXPORT  Reset_Handler      ; Standard version in startup_nRF5.s is marked as [WEAK]; this one overrides
;    ; IMPORT  SystemInit                
;    IMPORT  __main
;    ; LDR     R0, =SystemInit  ; Do NOT call SystemInit! Stack code already did.
;    ; BLX     R0
;    LDR     R0, =__main
;    BX      R0
;    ENDP


    AREA SVC_Area, CODE, READONLY
    EXPORT SVC_Handler         ; goes to interrupt vector [11]
    EXTERN C_SVC_Handler       ; C code SVC demultiplexer

SVC_Handler
    MRS   R1, MSP              ; Address Main Stack.
    LDR   R0,[R1,#24]          ; lr stacked by SVC interrupt = address of instruction following SVC 
    SUBS  R0,#2                ; Address of SVC instruction (Thumb format)
    LDRB  R0,[R0]              ; SVC instruction low octet: SVC number
    LDR   R2, =C_SVC_Handler
    BX    R2                   ; C code: C_SVC_Handler(unsigned SvcNumber, unsigned *regs)

    ALIGN
    END
