.PHONY: generate-results debug-run plot_debug_assertions help

help:
	@echo "make generate-results"
	@echo "    Generate the csv file of runs"
	@echo "make debug-run"
	@echo "    Run the test useful for re-creating a single csv row, for debugging"
	@echo "make plot_debug_assertions"
	@echo "    plot the results from a run with --features=\"debug_assertions\""

# For generation the csv file of runs
generate-results:
	cargo test --features="no_gravity" --features="early_quit" -- generate_and_run_setups_0 --nocapture

# Run the test useful for re-creating a single csv row, for debugging
debug-run:
	cargo test --features="no_gravity" --features="early_quit" --features="debug_assertions" -- debug_one_test --nocapture 2> run.log
	python3.10 analysis/STEP.py pre-run

# plot the results from a run with --features="debug_assertions"
plot_debug_assertions:
	python3.10 analysis/STEP.py pre-run
