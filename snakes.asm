; Copyright 2020 by Robin Sergeant. See license.txt distributed with this file.

            processor 6502

            include "vcs.h"

            include "macro.h"

            include "xmacro.h"

      SEG.U vars

      ORG $80

MOTION_DELAY = 10
MUNCH_DELAY = 20
DISPLAY_POS = [160 - 43] / 2

Score       ds 1
HiScore     ds 1

Sound       ds 1

RandomNum   ds 1

DisplayLeft  ds 17
DisplayRight ds 17

Direction   ds 1

MotionCount ds 1

HeadLoc     ds 1
FreeLoc     ds 1
TailLoc = Body

FaceSpritePtr   ds 2

PF2a_Current  ds 1
PF0b_Current  ds 1
PF1b_Current  ds 1
Mask_Current  ds 1
Fruit_Current ds 1

PF2a_Temp    ds 1
PF0b_Temp    ds 1
PF1b_Temp    ds 1
Mask_Temp    ds 1

SpritePtr0 = PF2a_Current
SpritePtr1 = PF1b_Current
SpritePtr2 = PF2a_Temp
SpritePtr3 = PF1b_Temp

Temp = PF2a_Current
STACK_POS = Mask_Temp

Body        ds #$FF - STACK_POS

; calculate the maximum score rounded down to a multiple of 5
; and convert to BCD
MAX_SCORE SET $FF - Body - 3
MAX_SCORE SET MAX_SCORE - MAX_SCORE % 5
MAX_SCORE SET MAX_SCORE / 10 << 4 + MAX_SCORE % 10

            SEG

            ORG $F000


Reset      CLEAN_START

        ldx #STACK_POS
        txs

        lda #$98
        sta COLUPF

        lda #1
        sta VDELP1

        lda #5
        sta AUDV0

        ;lda #4
        ;sta CTRLPF

        lda #$FF
        sta DisplayLeft+16
        sta DisplayRight+16

        lda #>Char0
        sta FaceSpritePtr+1
        lda #<Face
        sta FaceSpritePtr

Restart     lda #$12
            sta Body
            lda #$22
            sta Body+1
            lda #$32
            sta Body+2
            lda #0
            sta Body+3

            lda #2
            sta HeadLoc
            lda #3
            sta FreeLoc

            ldx #1
            ldy #2
            jsr UpdatePlayField
            inx
            jsr UpdatePlayField

            lda #MOTION_DELAY
            sta MotionCount

StartOfFrame

   ; Start of vertical blank processing

        VERTICAL_SYNC

        ; 36 scanlines of vertical blank...

        ldx #32
        jsr WaitForLines

        ; 2 scan lines to position sprite
        lda #DISPLAY_POS
        ldx #1
        jsr SetHorizPos
        sta WSYNC
        lda #DISPLAY_POS
        clc
        adc #6
        ldx #0
        jsr SetHorizPos

        ; 1 scaline to move sprite
        sta WSYNC
        sta HMOVE

        lda #0
        sta VBLANK

        ; 192 scanlines of picture...

         ; 14 scan lines of nothing
              ldx #14
              jsr WaitForLines

        ; 1 scan line to choose sprites
              lda #>Char0
              sta SpritePtr0+1
              sta SpritePtr1+1
              sta SpritePtr2+1
              sta SpritePtr3+1
              lda Score
              and #$0F
              asl
              asl
              asl
              sta SpritePtr0
              lda Score
              and #$F0
              lsr
              sta SpritePtr1
              lda HiScore
              and #$0F
              asl
              asl
              asl
              sta SpritePtr2
              lda HiScore
              and #$F0
              lsr
              sta SpritePtr3
              sta WSYNC

        ; 8 scan lines to draw sprite
              ldy #7
              ldx #$C2
              lda #2
              sta NUSIZ0
              sta NUSIZ1
DrawSprite    lda (SpritePtr3),y
              sta GRP1
              sta WSYNC
              lda (SpritePtr2),y
              sta GRP0
              stx COLUP1
              stx COLUP0
              lda (SpritePtr1),y
              sta GRP1
              lda (SpritePtr0),y
              inx
              inx
              dey
              SLEEP 10
              sta GRP0
              bpl DrawSprite

              ; 5 scan lines to position game sprites
              sta WSYNC
              ldx FreeLoc
              lda Body,x
              and #$F0
              lsr
              lsr
              clc
              adc #48
              ldx #1
              jsr SetHorizPos
              sta WSYNC
              ldx HeadLoc
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

              ldx #0
              ldy #0
              stx Mask_Current
              stx Fruit_Current
              stx NUSIZ0
              stx NUSIZ1
              lda #$FF
              sta PF2a_Current
              sta PF0b_Current
              lda #$F8
              sta PF1b_Current
              lda #$0E
              sta COLUP1
Line1         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors          ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              lda Fruit_Current   ; 3
              sta GRP1            ; 3 = 14
              ldy #2
              lda PF2a_Current
              sta PF2
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda #0
              sta PF2
              lda DisplayRight,x  ; 4
              lsr                 ; 2
              lsr                 ; 2
              lsr                 ; 2
              lsr                 ; 2
              sta PF1b_Temp

Line3         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+2        ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              ldy PF1b_Temp       ; 3
              lda PFData,y        ; 4
              sta PF1b_Temp       ; 3
              ldy HeadLoc         ; 3
              lda Body,y          ; 4
              and #$0F            ; 2
              sta Mask_Temp       ; 3 = 22
              ldy #3

Line4         ;sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+3        ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              cpx Mask_Temp       ; 3
              bne SetMask
              lda #$FF
SetMask       sta Mask_Temp
              ldy #4

Line5         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+4        ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              ldy FreeLoc         ; 3
              lda Body,y          ; 4
              bne ValidLoc
              lda #$FF
              bne Line5End
ValidLoc      and #$0F            ; 2
Line5End      sta Fruit_Current   ; 3
              ldy #5

Line6         sta WSYNC
              lda #0
              sta PF0
              lda #1
              sta PF1
              lda colors+5        ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              cpx Fruit_Current       ; 3
              bne SetFruit
              lda #%01100000
SetFruit      sta Fruit_Current
              lda #0
              sta GRP1

Line7         sta WSYNC
              ldy #6
              sta PF0
              lda #1
              sta PF1
              lda colors+6        ; 4
              sta COLUP0          ; 3
              lda (FaceSpritePtr),y
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
              inx
              ldy #0

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
              SLEEP 3
              lda PF0b_Current
              sta PF0
              lda PF1b_Current
              sta PF1
              lda #0
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
              ldx #20
              jsr WaitForLines

              lda #%01000010

              sta VBLANK            ; end of screen - enter blanking



              ; 30 scanlines of overscan...
              TIMER_SETUP 30

              ldx Sound
              lda Effects,x
              sta AUDC0
              beq CheckStick
              inx
              lda Effects,x
              sta AUDF0
              inc Sound
              inc Sound

CheckStick    jsr CheckJoystick
              cpx Direction
              beq Motion

              lda Direction
              bne SetDirection  ; not the first joystick push
              lda MotionCount   ; use MotionCount as seed
              sta RandomNum

SetDirection  stx Direction
              jsr GetRandom

Motion        dec MotionCount
              bne CheckFruit
Timeout       lda #MOTION_DELAY
              sta MotionCount
              lda Direction
              beq WaitOver
              lda #<Face
              sta FaceSpritePtr
              jsr Move
              bvc CheckLen
              jmp GameOver

CheckLen      lda Score
              cmp #MAX_SCORE
              bne CheckFruit
              lda #0
              sta Direction     ; Win!

CheckFruit    ldx FreeLoc       ; if FreeLoc contains zero then we need
              lda Body,x        ; to place the next fruit
              bne WaitOver
              lda RandomNum     ; check RandomNum corresponds to a free square
              jsr UnpackA
              jsr CheckPlayField
              bvs UpdateRandom
              ldx FreeLoc       ; if so place fruit here
              sta Body,x
UpdateRandom  jsr GetRandom

WaitOver      TIMER_WAIT
              sta WSYNC
              jmp StartOfFrame

; wait for X scanlines
WaitForLines SUBROUTINE
              dex
              sta WSYNC
              bne WaitForLines
              rts

CheckJoystick SUBROUTINE
            ldx Direction
            bit SWCHA
            bvs CheckRight
            cpx #2
            beq .return
            ldx #1
            rts
CheckRight  bit SWCHA
            bmi CheckUP
            cpx #1
            beq .return
            ldx #2
            rts
CheckUP     lda #$10
            bit SWCHA
            bne CheckDOWN
            cpx #4
            beq .return
            ldx #3
            rts
CheckDOWN   lda #$20
            bit SWCHA
            bne .return
            cpx #3
            beq .return
            ldx #4
.return     rts

; Move snake
Move SUBROUTINE move
            ldx HeadLoc             ; Find location of head
            lda Body,x
            jsr UnpackA
            jsr UpdatePlayField     ; Put body segment at old head location
            jsr UnpackA
            lda Direction           ; Use direction to calculate new head loc
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
            jsr CheckPlayField      ; check that new location is empty
            bvs .collision
            cmp #0
            beq .move               ; special case, do not check for fruit here
            ldx FreeLoc
            cmp Body,x              ; check if new location contains fruit
            beq .grow               ; if so grow rather than move
.move       pha
            lda TailLoc
            jsr UnpackA
            jsr UpdatePlayField     ; clear PF at tail
            ldx HeadLoc
            pla
.reorder    ldy Body,x              ; shift snake backwards overwriting tail
            sta Body,x
            tya
            dex
            bpl .reorder
            clv
            rts
.grow       sed                     ; increase score using BCD addition
            clc
            lda Score
            adc #1
            cld
            sta Score
            inc HeadLoc             ; increment head and free locations
            inc FreeLoc
            lda #0
            inx
            sta Body,x              ; clear free location coordinates
            lda #<FaceClose         ; set face sprite to a closed mouth
            sta FaceSpritePtr
            lda #MUNCH_DELAY         ; increase motion delay to make visible
            sta MotionCount
            lda #ChimeIndex         ; trigger sound effect
            sta Sound
            clv
            rts
.collision  bit .return             ; set overflow flag
.return     rts


GameOver SUBROUTINE
            lda CrashIndex
            sta Sound
            lda Score             ; Game Over!
            cmp HiScore
            bcc .NoHiScore
            sta HiScore
.NoHiScore  ldx #16
            lda #0
            sta Score
            sta Direction
            sta RandomNum
.ResetPF    dex
            sta DisplayLeft,x
            sta DisplayRight,x
            bne .ResetPF
            jmp Restart


; CheckPlayField routine
CheckPlayField SUBROUTINE
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
            pla
            clv
            rts
.collision  pla
            bit .return         ; set overflow flag
.return     rts

; UpdatePlayField routine
UpdatePlayField SUBROUTINE
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
            lda RandomNum
            lsr
            bcc .NoEor
            eor #$D4
.NoEor      sta RandomNum
            rts

; SetHorizPos routine
; A = X coordinate
; X = player number (0 or 1)
SetHorizPos SUBROUTINE
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
  SLEEP 3   ; waste 3 cycles for correct timing
  sta RESP0,x  ; fix coarse position
  sta HMP0,x  ; set fine offset
  rts    ; return to caller

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

Face        .byte %11110000,%11110000,%01100000,%01100000,%11110000,%10010000,%11110000
FaceClose   .byte %11110000,%11110000,%01100000,%01100000,%11110000,%11110000,%11110000

colors      .byte $1E,$2E,$3E,$4E,$5E,$6E,$7E,$8E

Effects
ChimeEffect .byte 6,4,6,3,6,2,6,1,6,0,0
CrashEffect .byte 8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,0

ChimeIndex = ChimeEffect - Effects
CrashIndex = CrashEffect - Effects

PFData   .byte %00001000,%10001000,%01001000,%11001000,%00101000,%10101000,%01101000,%11101000,%00011000,%10011000,%01011000,%11011000,%00111000,%10111000,%01111000,%11111000

BitMask  .byte %00000001, %00000010, %00000100, %00001000, %00010000, %00100000, %01000000, %10000000

            ORG $FFFA



            .word Reset          ; NMI

            .word Reset          ; RESET

            .word Reset          ; IRQ



      END
