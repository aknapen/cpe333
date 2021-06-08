#This file contains A basic bitch program
.data
	TEST: .word 50


.text

	# Of note, dependent instructions must either be 1 after the other or have 2 instructions between. 
	li a0, 20 # Result 20
	addi a0, a0, 3 # Result 23

	li a1, 5		# Result 5
	li a2, 7		# Result 7
	li a3, 109 		# Result 109
	li a4, 26		# Result 26
	nop				# Result 0
	nop				# Result 0
	add a2, a3, a4	# Result 135
	sub a3, a4, a2	# Result -109

	#Load test
	la t0, TEST 	# Result 50
	nop				# Result 0
	nop				# Result 0
	lw t1, 0(t0)	# Idk 
	lw t2, 0(t0)	# Idk

	addi t2, t2, 3	# Idk + 3
	addi t1, t2, 3	# Idk + 3
