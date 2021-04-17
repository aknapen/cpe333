Naming convention:
	test<instruction type>_<hazards or not>.s
	
How to use these files:
	1. Compile the program
	2. Import the .mem file into vivado and set the memory module to read from it (or copy/paste into the .mem already in vivado)
	3. Configure the clock_tb.sv testbench, instructions here: https://piazza.com/class/kbprd1f9gqj5ro?cid=19
	4. Run the simulator
	5. That's it! If you want help interpretting the results, check out the piazza post here: https://piazza.com/class/kbprd1f9gqj5ro?cid=13