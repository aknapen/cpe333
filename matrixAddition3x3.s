.data
   A: .byte 0x0, 0x3, 0x2, 0x0, 0x3, 0x1, 0x0, 0x3, 0x2
   B: .byte 0x1, 0x1, 0x0, 0x3, 0x1, 0x2, 0x0, 0x0, 0x0
   res: .byte 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0

.text
# Initialize registers
la a0 A # store address into matrix A in a0
la a1 B # store address into matrix B in a1
li a2 3 # store dimension N in a2
li a3 3 # store dimension M in a3
la a4 res # store address into resulting matrix in a0
Call MatrixAddition
j end


# Multiply Function
# Input registers: a0 = A, a1= B
# Output: performs A*B and stores the result in a0
# Changed registers: a0
Mult:
   li t0, 0      	# store result in t0
   bgt a0, a1, MultLoop	# If a0 < a1 we swap values for optimization
   mv t1, a0 	# t1 = a0
   mv a0, a1  	# a0 = a1
   mv a1, t1 	# a1 = t1 = a0
MultLoop:
   Beqz a1 MultEnd
   add t0, t0, a0	# a0 = a0 + t0
   addi a1, a1, -1	# Decrement a1 as counter
   J MultLoop
MultEnd:
   mv a0, t0 	# move final result into a0
   ret


# Input Registers: a0 = &A, a1 = &B, a2 = M, a3 = N, a4 = &res
# Output: the address to the sum matrix is stored in a0
# Changed registers: a0, a4
MatrixAddition:
   addi sp sp -20 # make space on stack for a0, a1, t0, t1
   sw ra 0(sp) # store return address on stack
   sw a0 4(sp)	# store a0 on stack
   sw a1 8(sp)	# store a1 on stack
   sw t0 12(sp) # store t0 on stack
   sw t1 16(sp) # store t1 on stack
   mv a0 a2   # prep m and n for multiplication
   mv a1 a3
   Call Mult   # M*N now stored in a0
   lw ra 0(sp) # restore return address
   addi sp sp 4 # reset stack pointer
   mv s1 a0   # store m * n in s1. Use as counter
   lw a0 0(sp)	# load value of a0 off stack
   lw a1 4(sp)	# load value of a1 off stack
   addi sp sp 8   # reset stack pointer
Loop:
   beqz s1 endAdd # is counter = 0?
   lb t0 0(a0)	# t0 = current element of A
   lb t1 0 (a1)   # t1 = current element of B
   add t2 t0 t1   # t2 = t0 + t1
   sb t2 0(a4)	# store the resulting element into result matrix
   addi a0 a0 1   # increment to next element in A
   addi a1 a1 1   # increment to next element in B
   addi a4 a4 1   # increment to next element in result matrix
   addi s1 s1 -1  # decrement counter
   j Loop
endAdd:
   mv a0 a4 # store final result in a0
   ret
   
end: j end
