#########################################
#										#
#	OTTER Tests							#
#	(37 instructions)					#
#	CalPoly 233 - Callenes				#
#										#	
#########################################

.equ test, 0x0044 
.equ SWITCHES, 0x11000000
.equ VGA_READ, 0x11040000
.equ LEDS,0x11080000
.equ SSEG, 0x110C0000
#.equ VGA_ADDR, 0x11100000
#.equ VGA_COLOR, 0x11140000
.equ DELAY_CNT, 1000000


.text

.global main 
.type main, @function
main:
	li x16, SWITCHES
	li x11, SSEG
	li x12, LEDS
	li x17, DELAY_CNT
  li x14, -1
loop: 
  addi x14, x14, 1
  sw x14, 0(x11)

  slli x10,x10,1; #turn on one more LED
	addi x10,x10,1; 
	sw x10, 0(x12);

  call start_delay
	j loop

# BEGIN DELAY
start_delay:
	li x17, DELAY_CNT
delay_loop:
	addi x17, x17, -1
	bnez x17, delay_loop
# END DELAY

	ret
