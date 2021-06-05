#This file contains A basic bitch program
.data
	TEST: .word 50


.text
	li a0, 20
	addi a0, a0, a1

	add a2, a3, a4
	sub a3, a4, a2

	#Load test
	auipc	t0,0x0
	addi	t0,t0,56 # PC+64=0x50 <TEST>
	lw t1, 0(t0)
	lw t2, 0(t0)

	addi t2, t2, 3
	addi t1, t2, 3

	# Fill the queue
	add a2, a3, a4
	sub a3, a4, a2
	add a2, a3, a4
	sub a3, a4, a2
	add a2, a3, a4
	sub a3, a4, a2
	add a2, a3, a4
	sub a3, a4, a2
	add a2, a3, a4
	sub a3, a4, a2

	
