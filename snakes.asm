            processor 6502

            include "vcs.h"

            include "macro.h"

            include "xmacro.h"

      SEG.U vars

      ORG $80

MotionDelay = 10

DisplayPos    ds 1 ; 1 byte frame counter

DisplayNum    ds 1
Score = DisplayNum

StickState    ds 1

SpritePtr0    ds 2
SpritePtr1    ds 2

DisplayLeft  ds 17
DisplayRight ds 17

Direction   ds 1

MotionCount ds 1

HeadLoc     ds 1
FreeLoc     ds 1
TailLoc = Body

PF2a_Current  ds 1
PF0b_Current  ds 1
PF1b_Current  ds 1
Mask_Current  ds 1

PF2a_Temp    ds 1
PF0b_Temp    ds 1
PF1b_Temp    ds 1
Mask_Temp    ds 1

Temp = PF2a_Current
STACK_POS = Mask_Temp

Body        ds #$FF - STACK_POS

            SEG

            ORG $F000


Reset      CLEAN_START

        ldx #STACK_POS
        txs

        lda #>Char0
        sta SpritePtr0+1
        sta SpritePtr1+1
        lda SWCHA
        sta StickState
        lda #69
        sta DisplayPos

        lda #$98
        sta COLUPF

        lda #$10
        sta NUSIZ0

        lda #1
        sta VDELP1

        lda #4
        sta CTRLPF

        lda #$FF
        sta DisplayLeft+16
        sta DisplayRight+16

        lda #$12
        sta Body
        lda #$22
        sta Body+1
        lda #$32
        sta Body+2

        lda #2
        sta HeadLoc
        lda #3
        sta FreeLoc

        ldx #1
        ldy #2
        jsr SetPlayField
        inx
        jsr SetPlayField
        inx
        jsr SetPlayField

        lda #MotionDelay
        sta MotionCount

StartOfFrame

   ; Start of vertical blank processing

        VERTICAL_SYNC

        ; 36 scanlines of vertical blank...

        ldx #31
        jsr WaitForLines

        lda #$FF
        sta PF2a_Current
        sta PF0b_Current
        lda #$F8
        sta PF1b_Current
        sta WSYNC

        ; 3 scan line to position sprite
        cld
        lda DisplayPos
        ldx #1
        jsr SetHorizPos
        sta WSYNC
        lda DisplayPos
        clc
        adc #6
        ldx #0
        jsr SetHorizPos

        ; 1 scaline to move sprite
        sta WSYNC
        sta HMOVE

        ; 1 scan line to choose sprite
        lda DisplayNum
        and #$0F
        asl
        asl
        asl
        sta SpritePtr0
        lda DisplayNum
        and #$F0
        lsr
        sta SpritePtr1
        sta WSYNC

        lda #0
        sta VBLANK

        ; 192 scanlines of picture...

        ; 8 scan lines to draw sprite
              ldy #7
              ldx #$C2
DrawSprite    lda (SpritePtr1),y
              sta GRP1
              sta WSYNC
              lda (SpritePtr0),y
              sta GRP0
              stx COLUP1
              stx COLUP0
              inx
              inx
              dey
              bpl DrawSprite

              ; 3 scan lines to position fruit
              sta WSYNC
              ldx FreeLoc
              lda Body,x
              and #$F0
              lsr
              lsr
              clc
              adc #48
              ldx #0
              jsr SetHorizPos
              sta WSYNC
              sta HMOVE

         ; 17 scan lines of nothing
              ldx #17
              jsr WaitForLines

              ldx #0
              lda #$F0
              sta Mask_Current
Line1         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors          ; 4
              sta COLUP0          ; 3
              lda fruit           ; 4
              and Mask_Current    ; 3
              sta GRP0            ; 3 = 17
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda #0
              sta PF2
              lda DisplayLeft,x   ; 4
              sta PF2a_Temp       ; 3
              lda DisplayRight,x  ; 4
              asl                 ; 2
              asl                 ; 2
              asl                 ; 2
              asl                 ; 2
              sta PF0b_Temp       ; 3 = 22

Line2         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+1        ; 4
              sta COLUP0          ; 3
              lda fruit+1         ; 4
              and Mask_Current    ; 3
              sta GRP0            ; 3 = 17
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda 0
              sta PF2
              lda DisplayRight,x  ; 4
              lsr                 ; 2
              lsr                 ; 2
              lsr                 ; 2
              lsr                 ; 2
              tay                 ; 2 = 14

Line3         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+2        ; 4
              sta COLUP0          ; 3
              lda fruit+2         ; 4
              and Mask_Current    ; 3
              sta GRP0            ; 3 = 17
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda 0
              sta PF2
              lda PFData,y        ; 4
              sta PF1b_Temp       ; 3
              ldy FreeLoc         ; 3
              lda Body,y            ; 4
              and #$0F            ; 2
              sta Mask_Temp       ; 3 = 19

Line4         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+3        ; 4
              sta COLUP0          ; 3
              lda fruit+3         ; 4
              and Mask_Current    ; 3
              sta GRP0            ; 3 = 17
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda 0
              sta PF2
              cpx Mask_Temp       ; 3
              bne SetMask
              lda #$FF
SetMask       sta Mask_Temp
              ldy #4

Lines5to7     sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors,y        ; 4
              sta COLUP0          ; 3
              lda fruit,y         ; 4
              and Mask_Current    ; 3
              sta GRP0            ; 3 = 17
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda 0
              sta PF2
              iny
              cpy #7
              bne Lines5to7
              inx
              sta GRP0

Line8         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda PF2a_Current
              sta PF2
              lda PF2a_Temp       ; 3
              sta PF2a_Current    ; 3
              lda Mask_Temp       ; 3
              sta Mask_Current    ; 3
              SLEEP 2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda 0
              sta PF2
              lda PF0b_Temp       ; 3
              sta PF0b_Current    ; 3
              lda PF1b_Temp       ; 3
              sta PF1b_Current    ; 3

              cpx #18             ; 2
              beq Done            ; 2
              jmp Line1           ; 3 = 25

Done          sta WSYNC
              ldx #0
              stx PF0
              stx PF1
              stx PF2
              stx PF0
              stx PF1
              stx PF2
              ; 20 scan lines of nothing
              ldx #19
              jsr WaitForLines

              lda #%01000010

              sta VBLANK                     ; end of screen - enter blanking



              ; 30 scanlines of overscan...
              TIMER_SETUP 30

CheckStick    lda SWCHA
              eor StickState
              bne StateChange

              dec MotionCount
              bne WaitOver
              lda #MotionDelay
              sta MotionCount
              lda Direction
              beq WaitOver
              jsr Move
              bvc WaitOver
              jmp Reset

StateChange   lda SWCHA
              sta StickState
              lda Direction
              bne CheckLeft     ; not the first joystick push
              lda MotionCount   ; use MotionCount as seed
              jsr GetRandom     ; get random location of fruit
              ldx FreeLoc
              sta Body,x

CheckLeft     bit StickState
              bvs CheckRight
              lda #1
              sta Direction
              jmp WaitOver

CheckRight    bit StickState
              bmi CheckUP
              lda #2
              sta Direction
              jmp WaitOver

CheckUP       lda #$10
              bit StickState
              bne CheckDOWN
              lda #3
              sta Direction
              jmp WaitOver

CheckDOWN     lda #$20
              bit StickState
              bne WaitOver
              lda #4
              sta Direction

WaitOver      TIMER_WAIT
              jmp StartOfFrame

; wait for X scanlines
WaitForLines SUBROUTINE
              dex
              sta WSYNC
              bne WaitForLines
              rts

; SetHorizPos routine
; A = X coordinate
; X = player number (0 or 1)
SetHorizPos SUBROUTINE
  cmp #3
  bcc FindPos
  sec
  sbc #3
FindPos
  sta WSYNC  ; start a new line
  sec    ; set carry flag
DivideLoop
  sbc #15    ; subtract 15
  bcs DivideLoop  ; branch until negative
  eor #7    ; calculate fine offset
  asl
  asl
  asl
  asl
  sta HMP0,x  ; set fine offset
  sta RESP0,x  ; fix coarse position
  rts    ; return to caller

; Move snake
Move SUBROUTINE move
            ldx HeadLoc             ; Find location of head
            lda Body,x
            jsr UnpackA
            lda Direction
.left       cmp #1
            bne .right
            dex
            bpl .continue
            bmi .collision
.right      cmp #2
            bne .up
            inx
            cpx #16
            bne .continue
            beq .collision
.up         cmp #3
            bne .down
            dey
            bpl .continue
            bmi .collision
.down       iny
            cpy #16
            beq .collision
.continue   jsr PackXY
            jsr SetPlayField
            bvs .collision
            ldx FreeLoc
            cmp Body,x
            beq .grow
            pha
            lda TailLoc
            jsr UnpackA
            jsr ClearPlayField     ; clear PF at tail
            ldx HeadLoc
            pla
.reorder    ldy Body,x
            sta Body,x
            tya
            dex
            bpl .reorder
            clv
            rts
.grow       inc HeadLoc
            inc FreeLoc
            lda Body,x
            jsr GetRandom
            inx
            sta Body,x
            sed
            clc
            lda Score
            adc #1
            cld
            sta Score
            clv
            rts
.collision  bit .return             ; set overflow flag
.return     rts


; UpdatePlayField routine
SetPlayField SUBROUTINE
            pha
            cpx #8
            bcc .continue
            ; add 17 to y to access right hand side of screen!
            tya
            clc
            adc #17
            tay
            ; reallign x relative to right hand side of screen
            txa
            sec
            sbc #8
            tax
.continue   lda BitMask,x
            and DisplayLeft,y   ; is bit already set?
            bne .collision
            lda BitMask,x
            eor DisplayLeft,y   ; set bit
            sta DisplayLeft,y
            pla
            clv
            rts
.collision  pla
            bit .return         ; set overflow flag
.return     rts

; UpdatePlayField routine
ClearPlayField SUBROUTINE
            pha
            cpx #8
            bcc .continue
            ; add 17 to y to access right hand side of screen!
            tya
            clc
            adc #17
            tay
            ; reallign x relative to right hand side of screen
            txa
            sec
            sbc #8
            tax
.continue   lda BitMask,x
            eor DisplayLeft,y
            sta DisplayLeft,y
            pla
            rts

; Pack X and Y into A
PackXY SUBROUTINE
            txa
            asl
            asl
            asl
            asl
            sta Temp
            tya
            ora Temp
            rts

; Unpack A into X and Y
UnpackA SUBROUTINE
            sta Temp
            lsr
            lsr
            lsr
            lsr
            tax
            lda #$0F
            and Temp
            tay
            lda Temp
            rts

; Get next random number
GetRandom SUBROUTINE
            lsr
            bcc .NoEor
            eor #$D4
.NoEor      rts

            ALIGN  256

SpriteData

Char0    .byte $00,$38,$44,$64,$54,$4C,$44,$38
Char1    .byte $00,$38,$10,$10,$10,$10,$30,$10
Char2    .byte $00,$7C,$40,$20,$18,$04,$44,$38
Char3    .byte $00,$38,$44,$04,$18,$08,$04,$7C
Char4    .byte $00,$08,$08,$7C,$48,$28,$18,$08
Char5    .byte $00,$38,$44,$04,$04,$78,$40,$7C
Char6    .byte $00,$38,$44,$44,$78,$40,$20,$1C
Char7    .byte $00,$20,$20,$20,$10,$08,$04,$7C
Char8    .byte $00,$38,$44,$44,$38,$44,$44,$38
Char9    .byte $00,$70,$08,$04,$3C,$44,$44,$38

fruit    .byte %00000000,%01100000,%11110000,%11110000,%11110000,%11110000,%01100000

colors   .byte $1E,$2E,$3E,$4E,$5E,$6E,$7E,$8E

PFData   .byte %00001000,%10001000,%01001000,%11001000,%00101000,%10101000,%01101000,%11101000,%00011000,%10011000,%01011000,%11011000,%00111000,%10111000,%01111000,%11111000

BitMask  .byte %00000001, %00000010, %00000100, %00001000, %00010000, %00100000, %01000000, %10000000

            ORG $FFFA



            .word Reset          ; NMI

            .word Reset          ; RESET

            .word Reset          ; IRQ



      END
