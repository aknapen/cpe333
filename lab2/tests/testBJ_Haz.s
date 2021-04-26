#This file contains tests for R-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)
#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13


	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a2, zero, -50
	addi a1, zero, 2

tests:	
	addi a1, a1, -1
	
	#Branch test
	bne zero, a1, tests
	beq zero, a1, test2

test2:
	addi a1, zero, 2
	j tests
	addi a0, zero, 0x69 # If the jump doesn't work properly a0 <= 0x69

fail:
	j fail
