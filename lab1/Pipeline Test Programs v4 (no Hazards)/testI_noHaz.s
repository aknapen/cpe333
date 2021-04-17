#This file contains tests for I-type RISCV instructions WITH NO HAZARDS (by this I mean I added nops to prevent hazards)
#You can check to see if any of your tests fail using this link: https://piazza.com/class/kbprd1f9gqj5ro?cid=13
	
	#Setup stuff
	addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
	addi a1, zero, 1

tests:
	nop
	nop
	#Add Immediate Test
	#initialize t0 to 50
	#subtract 50 off of t0
	addi t0, zero, 50
	nop
	nop
	bne a0, t0, fail #t0 should equal 50, otherwise program jumps to fail
	nop
	nop
	addi a2, t0, -50
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Set Less Than Immediate Test
	#Check if 25 < 50
	addi t0, zero, 25
	nop
	nop
	slti a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise
	nop
	nop
	beq a2, zero, fail
	nop
	nop
	
	#Set Less Than Immediate Unsigned Test
	#Check if -75 < 50 (it shouldn't be, since its unsigned)
	addi t0, zero, -75
	nop
	nop
	sltiu a2, t0, 50 #slt stores 1 in a2 if t0 < 50, 0 otherwise
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Xor Immediate Test
	#Xor something with itself, should be 0
	#Xor something with 0, should be itself
	addi t0, zero, 25
	nop
	nop
	xori a2, t0, 25
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	xori a2, t0, 0
	nop
	nop
	bne a2, t0, fail
	nop
	nop
	
	#Or Immediate Test
	#Or something with itself, should be itself
	#Or something with 0, should be itself
	ori a2, a0, 50
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	ori a2, a0, 0
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#And Immediate Test
	#And something with itself, should be itself
	#And something with 0, should be 0
	andi a2, a0, 50
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	andi a2, a0, 0
	nop
	nop
	bne a2, zero, fail
	nop
	nop
	
	#Shift Left Immediate Test
	#Shift t0 (25) left once (multiply by 2) and store in a2
	slli a2, t0, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Right Logical Immediate Test
	#Set a2 to 100, shift right once, test to make sure result is 50
	addi a2, zero, 100
	nop
	nop
	srli a2, a2, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	
	#Shift Right Arithmetic Immediate Test
	#Shift 100 right, should be 50
	#Shift -100 right, should be -50
	addi t1, zero, -100
	addi t2, zero, -50
	addi a2, zero, 100
	nop
	nop
	srai a2, a2, 1
	nop
	nop
	bne a0, a2, fail
	nop
	nop
	srai a2, t1, 1
	nop
	nop
	bne a2, t2, fail
	nop
	nop
	j tests
	nop
	nop
	
fail:
	j fail