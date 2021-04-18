sim:
	cd build_sim; make && make sim

setup_environ:
	qsub-sim

setup: rtl/sources.txt
	afu_sim_setup --source rtl/sources.txt build_sim

fpga_setup: rtl/sources.txt
	afu_synth_setup -s rtl/sources.txt build_fpga

.PHONY:clean
clean:
	rm -rf build_sim