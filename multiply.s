# Initialization Code
li a0 20
li a1 40
Call Mult
j end

# Multiply Function
# Input registers: a0 = A, a1= B
# Output: performs A*B and stores the result in a0
# Changed registers: a0

Mult:
	li t0, 0        	# store result in t0
	bgt a0, a1, MultLoop	# If a0 < a1 we swap values for optimization
	mv t1, a0     	# t1 = a0
	mv a0, a1    	# a0 = a1
	mv a1, t1     	# a1 = t1 = a0
MultLoop:
	Beqz a1 MultEnd
	add t0, t0, a0     	# a0 = a0 + t0
	addi a1, a1, -1    	# Decrement a1 as counter
	J MultLoop
MultEnd:
	mv a0, t0     	# move final result into a0
	ret


end: j end
