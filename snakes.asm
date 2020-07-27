; Copyright 2020 by Robin Sergeant. See license.txt distributed with this file.

            processor 6502

            include "vcs.h"

            include "macro.h"

            include "xmacro.h"

      SEG.U vars

      ORG $80

MOTION_DELAY = 10
MUNCH_DELAY = 20
HISCORE_DELAY = 40
SCORE_POS = 108
HISCORE_POS = 28

PLAY_AREA_WIDTH = 24
PLAY_AREA_HEIGHT = 16

DIR_LEFT    = %10000000
DIR_RIGHT   = %11000000
DIR_UP      = %00100000
DIR_DOWN    = %00110000

MOV_LEFT    = %00000000
MOV_RIGHT   = %01000000
MOV_UP      = %10000000
MOV_DOWN    = %11000000

SHOW_HI_SCORE = %00000001

GameState   ds 1

Score       ds 2
HiScore     ds 2

Sound       ds 1

RandomNum   ds 1

PF0R_PF1L     ds PLAY_AREA_HEIGHT
PF2L          ds PLAY_AREA_HEIGHT
PF1R          ds PLAY_AREA_HEIGHT

Direction     ds 1
NewDirection  ds 1

MotionCount ds 1
EventCount  ds 1

HeadPosition_X  ds 1
HeadPosition_Y  ds 1
TailPosition_X  ds 1
TailPosition_Y  ds 1
FruitPosition_X ds 1
FruitPosition_Y ds 1

FreeIndex       ds 1
FreeOffset      ds 1
TailOffset      ds 1
TailLoc = Body

FaceSpritePtr   ds 2

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Temp data / stack space here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SpritePtr0  ds 2
SpritePtr1  ds 2
SpritePtr2  ds 2
SpritePtr3  ds 2

Mask_Current  = SpritePtr0
Fruit_Current = SpritePtr1
Mask_Temp     = SpritePtr2

Temp = SpritePtr0
STACK_POS = SpritePtr3+1

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; end of stack space
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Body        ds #$FF - STACK_POS

; calculate the maximum bodu length
MAX_BODY_LENGTH SET [$FF - Body + 1] * 4

            SEG

            ORG $F800   ; 2K Rom origin


Reset      CLEAN_START

        ldx #STACK_POS
        txs

        lda #$98
        sta COLUPF
        sta RandomNum ; use this color as random number seed!

        lda #1
        sta VDELP1

        lda #5
        sta AUDV0

        lda #>Char0
        sta FaceSpritePtr+1
        lda #<Face
        sta FaceSpritePtr

        lda #PLAY_AREA_HEIGHT
        sta FruitPosition_Y

Restart     lda #0
            sta TailLoc
            sta FreeIndex
            sta TailOffset
            sta FreeOffset
            sta EventCount

            ldx #1
            ldy #2
            sty HeadPosition_Y
            sty TailPosition_Y
            stx TailPosition_X
            jsr UpdatePlayField
            inx
            jsr UpdatePlayField
            inx
            stx HeadPosition_X

            lda #MOTION_DELAY
            sta MotionCount

            lda #SHOW_HI_SCORE
            sta GameState

            lda #DIR_RIGHT
            sta Direction

            jsr GrowBody
            jsr GrowBody

StartOfFrame

   ; Start of vertical blank processing

        VERTICAL_SYNC

        ; 36 scanlines of vertical blank...

        ldx #32
        jsr WaitForLines

        ; 2 scan lines to position score sprites
        ldy #SCORE_POS
        lda GameState
        and #SHOW_HI_SCORE
        beq .+4
        ldy #HISCORE_POS
        tya
        ldx #0
        jsr SetHorizPos
        sta WSYNC
        tya
        clc
        adc #8
        ldx #1
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

        ; 1 scan line to choose score sprites
              lda #>Char0
              sta SpritePtr0+1
              sta SpritePtr1+1
              sta SpritePtr2+1
              ldx #Score
              lda GameState
              and #SHOW_HI_SCORE
              beq .+4
              ldx #HiScore
              lda $0,x
              and #$0F
              asl
              asl
              asl
              sta SpritePtr0
              lda $0,x
              and #$F0
              lsr
              sta SpritePtr1
              inx
              lda $0,x
              and #$0F
              asl
              asl
              asl
              bne SetLastDigit
              lda #<Blank       ; blank out leading zeros
              ldx SpritePtr1
              bne SetLastDigit
              sta SpritePtr1
SetLastDigit  sta SpritePtr2
              ;sta WSYNC

        ; 8 scan lines to draw sprite
              ldy #7
              ldx #$C2
              lda #1
              sta NUSIZ0
DrawSprite    sta WSYNC
              lda (SpritePtr1),y
              sta GRP1
              lda (SpritePtr2),y
              lsr                   ; nudge MSD right 1 pixel
              sta GRP0
              stx COLUP1
              stx COLUP0
              lda (SpritePtr0),y
              asl                   ; nudge LSD left 1 pixel
              inx
              sta GRP0
              inx
              lda (SpritePtr2),y
              lsr                   ; nudge MSD right 1 pixel
              sta GRP0
              lda (SpritePtr0),y
              asl                   ; nudge LSD left 1 pixel
              dey
              nop
              sta GRP0
              bpl DrawSprite

              ; 5 scan lines to position game sprites
              sta WSYNC
              lda FruitPosition_X
              asl
              asl
              clc
              adc #32
              ldx #1
              jsr SetHorizPos
              sta WSYNC
              lda HeadPosition_X
              asl
              asl
              clc
              adc #32
              ldx #0
              jsr SetHorizPos
              sta WSYNC
              sta HMOVE

              ldx #0
              stx Mask_Current
              stx Fruit_Current
              stx NUSIZ0
              stx NUSIZ1
              lda #$0E
              sta COLUP1

              lda #0
              cmp HeadPosition_Y
              bne SetMask
              lda #$FF              ; snakes head is on first row
SetMask       sta Mask_Current

              lda #0
              cmp FruitPosition_Y
              bne SetFruit
              lda #%01100000        ; fruit is on first row
SetFruit      sta Fruit_Current


              ldy #7
TopWall       sta WSYNC
              lda #$1F              ; 2
              sta PF1               ; 3
              lda #$FF              ; 2
              sta PF2               ; 3
              lda #$F0              ; 2
              SLEEP 12
              sta PF0               ; 3 = 27
              lda #$FF              ; 2
              SLEEP 5
              sta PF1               ; 3 = 37
              lda #$01              ; 2
              SLEEP 6
              sta PF2               ; 3 = 48
              lda #0                ; 2
              sta PF0               ; 3 = 53
              dey
              bpl TopWall

RenderRow     ldy #7
              sec
              sta WSYNC
              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2 = 8
              sta PF1               ; 3
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda (FaceSpritePtr),y ; 5
              and Mask_Current      ; 3
              sta GRP0              ; 3 = 29
              lda PF2L,x            ; 4 = 33
              sta PF2               ; 3
              lda PF0R_PF1L,x       ; 4 = 40
              sta PF0               ; 3
              lda PF1R,x            ; 4 = 47
              sta PF1               ; 3
              lda #$01              ; 2 = 52
              sta PF2               ; 3
              lda #0                ; 2 = 57
              sta PF0               ; 3
              dey                   ; 2
              txa                   ; 2
              sbc HeadPosition_Y    ; 3
              cmp #$FF              ; 2
              beq SetTempMask       ; 2
              lda #0                ; 2
SetTempMask   sta Mask_Temp         ; 3 = 76

              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2 = 8
              sta PF1               ; 3
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda (FaceSpritePtr),y ; 5
              and Mask_Current      ; 3
              sta GRP0              ; 3 = 29
              lda PF2L,x            ; 4 = 33
              sta PF2               ; 3
              lda PF0R_PF1L,x       ; 4 = 40
              sta PF0               ; 3
              lda PF1R,x            ; 4 = 47
              sta PF1               ; 3
              lda #$01              ; 2 = 52
              sta PF2               ; 3
              lda #0                ; 2 = 57
              sta PF0               ; 3
              dey                   ; 2
              lda Fruit_Current
              sta GRP1
              sec
              sta WSYNC

              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2 = 8
              sta PF1               ; 3
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda (FaceSpritePtr),y ; 5
              and Mask_Current      ; 3
              sta GRP0              ; 3 = 29
              lda PF2L,x            ; 4 = 33
              sta PF2               ; 3
              lda PF0R_PF1L,x       ; 4 = 40
              sta PF0               ; 3
              lda PF1R,x            ; 4 = 47
              sta PF1               ; 3
              lda #$01              ; 2 = 52
              sta PF2               ; 3
              lda #0                ; 2 = 57
              sta PF0               ; 3
              dey                   ; 2
              txa
              sbc FruitPosition_Y
              sta Fruit_Current
              ;sta WSYNC

              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2 = 8
              sta PF1               ; 3
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda (FaceSpritePtr),y ; 5
              and Mask_Current      ; 3
              sta GRP0              ; 3 = 29
              lda PF2L,x            ; 4 = 33
              sta PF2               ; 3
              lda PF0R_PF1L,x       ; 4 = 40
              sta PF0               ; 3
              lda PF1R,x            ; 4 = 47
              sta PF1               ; 3
              lda #$01              ; 2 = 52
              sta PF2               ; 3
              lda #0                ; 2 = 57
              sta PF0               ; 3
              dey                   ; 2
              lda Fruit_Current
              cmp #$FF
              beq UpdateFruit
              lda #0
UpdateFruit   and #%01100000
              sta Fruit_Current

RenderLine    sta WSYNC
              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2 = 8
              sta PF1               ; 3
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda (FaceSpritePtr),y ; 5
              and Mask_Current      ; 3
              sta GRP0              ; 3 = 29
              lda PF2L,x            ; 4 = 33
              sta PF2               ; 3
              lda PF0R_PF1L,x       ; 4 = 40
              sta PF0               ; 3
              lda PF1R,x            ; 4 = 47
              sta PF1               ; 3
              lda #$01              ; 2 = 52
              sta PF2               ; 3
              lda #0                ; 2 = 57
              sta PF0               ; 3
              cpy #2
              bne KeepFruit
              sta GRP1
KeepFruit     dey
              bne RenderLine

              sta WSYNC
              lda PF0R_PF1L,x       ; 4
              and #$0F              ; 2
              ora #$10              ; 2
              sta PF1               ; 3 = 11
              lda colors,y          ; 4
              sta COLUP0            ; 3
              lda Mask_Temp         ; 3
              sta Mask_Current      ; 3
              lda PF2L,x            ; 4
              sta PF2               ; 3 = 31
              lda PF0R_PF1L,x       ; 4
              sta PF0               ; 3 = 38
              lda PF1R,x            ; 4
              inx
              sta PF1               ; 3
              lda #$01              ; 2
              sta PF2               ; 3
              lda #0                ; 2
              sta PF0               ; 3
              cpx #PLAY_AREA_HEIGHT
              bcs Bottom
              jmp RenderRow

Bottom        ldy #7
BottomWall    sta WSYNC
              sta GRP0
              lda #$1F              ; 2
              sta PF1               ; 3
              lda #$FF              ; 2
              sta PF2               ; 3
              lda #$F0              ; 2
              SLEEP 9
              sta PF0               ; 3 = 27
              lda #$FF              ; 2
              SLEEP 5
              sta PF1               ; 3 = 37
              lda #$01              ; 2
              SLEEP 6
              sta PF2               ; 3 = 48
              lda #0                ; 2
              sta PF0               ; 3 = 53
              dey
              bpl BottomWall

              sta WSYNC
              ldx #0
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

              lda EventCount
              beq HandleSound
              dec EventCount
              lda GameState
              eor #SHOW_HI_SCORE
              sta GameState

HandleSound   ldx Sound
              lda Effects,x
              sta AUDC0
              beq CheckStick
              inx
              lda Effects,x
              sta AUDF0
              inc Sound
              inc Sound

CheckStick    jsr CheckJoystick
              stx NewDirection

Motion        dec MotionCount
              bne CheckFruit
Timeout       lda #MOTION_DELAY
              sta MotionCount
              lda NewDirection
              beq WaitOver
              jsr UpdateDirection
              lda #0
              sta GameState
              lda #<Face
              sta FaceSpritePtr
              jsr Move
              bvc CheckFruit
              jmp GameOver

CheckFruit    lda FruitPosition_Y   ; if invalid we need to place the next fruit
              cmp #PLAY_AREA_HEIGHT
              bne WaitOver
              jsr GetRandomPos
              txa
              jsr CheckPlayField    ; check pos corresponds to a free square
              bvs WaitOver
              cmp HeadPosition_X    ; also check pos is not equal to snakes head
              bne PlaceFruit
              cpy HeadPosition_Y
              beq WaitOver
PlaceFruit    sta FruitPosition_X   ; if so place fruit here
              sty FruitPosition_Y

WaitOver      jsr GetRandom         ; cycle through random number every frame
              sta WSYNC
              TIMER_WAIT
              sta WSYNC
              jmp StartOfFrame

; wait for X scanlines
WaitForLines SUBROUTINE
              dex
              sta WSYNC
              bne WaitForLines
              rts

CheckJoystick SUBROUTINE
            ldx NewDirection
            bit SWCHA
            bvs .CheckRight
            ldx #DIR_LEFT
            rts
.CheckRight bit SWCHA
            bmi .CheckUP
            ldx #DIR_RIGHT
            rts
.CheckUP    lda #$10
            bit SWCHA
            bne .CheckDOWN
            ldx #DIR_UP
            rts
.CheckDOWN  lda #$20
            bit SWCHA
            bne .return
            ldx #DIR_DOWN
.return     rts

UpdateDirection SUBROUTINE
            lda NewDirection
            bit Direction
            bne .return           ; overlap indicates invalid direction change
            sta Direction
.return     rts

; Move snake
Move SUBROUTINE move
            ldx HeadPosition_X
            ldy HeadPosition_Y
            jsr UpdatePlayField     ; Put body segment at old head location
            ldx HeadPosition_X
            lda Direction           ; Use direction to calculate new head loc
.left       cmp #DIR_LEFT
            bne .right
            dex
            bpl .continue
            bmi .collision
.right      cmp #DIR_RIGHT
            bne .up
            inx
            cpx #PLAY_AREA_WIDTH
            bne .continue
            beq .collision
.up         cmp #DIR_UP
            bne .down
            dey
            bpl .continue
            bmi .collision
.down       iny
            cpy #PLAY_AREA_HEIGHT
            beq .collision
.continue   txa
            jsr CheckPlayField      ; check that new location is empty
            bvs .collision
            sta HeadPosition_X
            sty HeadPosition_Y
            cmp FruitPosition_X
            bne .move
            cpy FruitPosition_Y
            beq .grow               ; if so grow rather than move
.move       jsr TrimTail
            jsr GrowBody
            clv
            rts
.grow       jsr GrowBody
            lda #PLAY_AREA_HEIGHT
            sta FruitPosition_Y
            jsr UpdateScore
            bcs .setsprite
            lda #HISCORE_DELAY      ; flash hi-score if below it
            sta EventCount
.setsprite  lda #<FaceClose         ; set face sprite to a closed mouth
            sta FaceSpritePtr
            lda #MUNCH_DELAY        ; increase motion delay to make visible
            sta MotionCount
            lda #ChimeIndex         ; trigger sound effect
            sta Sound
            clv
            rts
.collision  bit .return             ; set overflow flag
.return     rts

GrowBody SUBROUTINE
            lda Direction           ; convert direction to movement value
            bpl .vertical
            and #%01000000
            bpl .store
.vertical   asl
            asl

.store      ldx FreeOffset          ; store new movement value
            beq .done
.shift      lsr
            lsr
            dex
            bne .shift
.done       ldx FreeIndex
            ora Body,x
            sta Body,x

            inc FreeOffset
            lda FreeOffset
            cmp #4
            bne .return
            lda #0
            sta FreeOffset
            inc FreeIndex
            ldx FreeIndex
            sta Body,x
.return     rts

TrimTail SUBROUTINE
            lda TailLoc           ; find movement value at tail
            ldx TailOffset
            beq .done
.shift      asl
            asl
            dex
            bne .shift
.done       and #%11000000

            ldx TailPosition_X    ; clear tail position
            ldy TailPosition_Y
            jsr UpdatePlayField

            ldx TailPosition_X    ; update tail position
            cmp #MOV_LEFT
            bne .right
            dex
.right      cmp #MOV_RIGHT
            bne .up
            inx
.up         cmp #MOV_UP
            bne .down
            dey
.down       cmp #MOV_DOWN
            bne .update
            iny
.update     stx TailPosition_X
            sty TailPosition_Y

            inc TailOffset
            lda TailOffset
            cmp #4
            bne .return
            lda #0
            sta TailOffset

            ldx FreeIndex
            dec FreeIndex
            lda Body,x
            dex
.reorder    ldy Body,x              ; shift backwards overwriting tail byte
            sta Body,x
            tya
            dex
            bpl .reorder

.return     rts

; on return the C flag will be set if Score >= HiScore
UpdateScore SUBROUTINE
            sed             ; use BCD mode for score arithmetic
            lda Score
            clc
            adc #1          ; add one to low byte of score
            sta Score
            lda Score+1     ; add carry to high byte of score
            adc #0
            sta Score+1
            cmp HiScore+1   ; check for new hi-score
            bcc .return
            bne .newhiscore
            lda Score
            cmp HiScore
            bcc .return
.newhiscore lda Score
            sta HiScore
            lda Score+1
            sta HiScore+1
.return     cld
            rts

; CheckScore SUBROUTINE
;             lda Score+1
;             cmp HiScore+1
;             bcc .return
;             lda Score
;             cmp HiScore
; .return     rts

GameOver SUBROUTINE
            lda CrashIndex
            sta Sound
            ldx #PLAY_AREA_HEIGHT
            lda #0
            sta Score
            sta Score+1
            sta NewDirection
.ResetPF    dex
            sta PF0R_PF1L,x
            sta PF2L,x
            sta PF1R,x
            bne .ResetPF
            jmp Restart

; CheckPlayField routine
CheckPlayField SUBROUTINE
            pha
            cpx #4
            bcs .pf2left
            lda PF0R_PF1L,y
            and BitMask_PF1L,x
            bne .pixelset
            beq .pixelclear
.pf2left    txa
            cpx #12
            bcs .pf0right
            sec
            sbc #4
            tax
            lda PF2L,y
            and BitMask_PF2,x
            bne .pixelset
            beq .pixelclear
.pf0right   cpx #16
            bcs .pf1right
            sec
            sbc #12
            tax
            lda PF0R_PF1L,y
            and BitMask_PF0,x
            bne .pixelset
            beq .pixelclear
.pf1right   sec
            sbc #16
            tax
            lda PF1R,y
            and BitMask_PF1,x
            bne .pixelset
.pixelclear pla
            clv
            rts
.pixelset   pla
            bit .return         ; set overflow flag
.return     rts

; UpdatePlayField routine
UpdatePlayField SUBROUTINE
            pha
            cpx #4
            bcs .pf2left
            lda PF0R_PF1L,y
            eor BitMask_PF1L,x
            sta PF0R_PF1L,y
            pla
            rts
.pf2left    txa
            cpx #12
            bcs .pf0right
            sec
            sbc #4
            tax
            lda PF2L,y
            eor BitMask_PF2,x
            sta PF2L,y
            pla
            rts
.pf0right   cpx #16
            bcs .pf1right
            sec
            sbc #12
            tax
            lda PF0R_PF1L,y
            eor BitMask_PF0,x
            sta PF0R_PF1L,y
            pla
            rts
.pf1right   sec
            sbc #16
            tax
            lda PF1R,y
            eor BitMask_PF1,x
            sta PF1R,y
            pla
            rts

; Get random X Y position
GetRandomPos SUBROUTINE
            lda RandomNum
            lsr
            lsr
            lsr
            lsr
            sta Temp      ; store top half of RandomNum in Temp
            lda RandomNum
            and #$0F      ; use bottom half for Y (0-15)
            tay
            jsr GetRandom ; get a second random number to extend X
            and #$0F      ; keep bottom half
            lsr           ; shift lsb into carry
            adc Temp      ; to add random number between 0 and 8 to Temp
            tax           ; X will now be between 0 and 23
            rts

; Get next random number
GetRandom SUBROUTINE
            lda RandomNum
            lsr
            bcc .NoEor
            eor #$D4
.NoEor      sta RandomNum
            rts

            ALIGN  256

; SetHorizPos routine
; A = X coordinate
; RESP0+X = reset register to strobe
SetHorizPos SUBROUTINE
            cpx #2 ; carry flag will be set for balls and missiles
            adc #0 ; (adding 1 to account for different timings)
            sec
            sta WSYNC
.loop       sbc #15
            bcs .loop
            eor #7
            asl
            asl
            asl
            asl
            sta.a HMP0,X  ; force absolute addressing for timing!
            sta RESP0,X
            rts

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
Blank    .byte $00,$00,$00,$00,$00,$00,$00,$00

Face        .byte %11110000,%11110000,%10010000,%11110000,%01100000,%01100000,%11110000,%11110000
FaceClose   .byte %11110000,%11110000,%11110000,%11110000,%01100000,%01100000,%11110000,%11110000

colors      .byte $8E,$7E,$6E,$5E,$4E,$3E,$2E,$1E

Effects
ChimeEffect .byte 6,4,6,3,6,2,6,1,6,0,0
CrashEffect .byte 8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,8,31,0

ChimeIndex = ChimeEffect - Effects
CrashIndex = CrashEffect - Effects

BitMask_PF2   .byte %00000001, %00000010, %00000100, %00001000
BitMask_PF0   .byte %00010000, %00100000, %01000000, %10000000
BitMask_PF1   .byte %10000000, %01000000, %00100000, %00010000
BitMask_PF1L  .byte %00001000, %00000100, %00000010, %00000001

            ORG $FFFA



            .word Reset          ; NMI

            .word Reset          ; RESET

            .word Reset          ; IRQ



      END
