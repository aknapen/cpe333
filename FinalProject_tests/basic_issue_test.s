#This file contains A basic bitch program
.data
	TEST: .word 50


.text

	# Of note, dependent instructions must either be 1 after the other or have 2 instructions between. 
	li a0, 20
	addi a0, a0, 3

	li a1, 5
	li a2, 7
	li a3, 109 		
	li a4, 26
	nop
	nop
	add a2, a3, a4
	sub a3, a4, a2

	#Load test
	la t0, TEST 
	nop
	nop
	lw t1, 0(t0)
	lw t2, 0(t0)

	addi t2, t2, 3
	addi t1, t2, 3
