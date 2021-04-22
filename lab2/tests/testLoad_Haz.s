#This file contains tests for R-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)
#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13

.data
	TEST: .word 50
	
.text
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

tests:	
	#Load test
	auipc	t0,0x0
	addi	t0,t0,56 # PC+64=0x50 <TEST>
	lw t1, 0(t0)
	bne t1, a0, fail
	nop
	nop
	
	j tests
	nop
	nop

fail:
	j fail
	nop
	nop
