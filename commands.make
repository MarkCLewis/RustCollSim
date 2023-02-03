.PHONY: count_time

# change make shell
SHELL := /bin/bash

count_time:
	grep -o "dt=[0-9.]*" "run.log" | grep -o "[0-9.]*" | awk '{sum+=$1} END {print sum}'
