#!/bin/bash

cargo run 4thOrder2 2> out.json
python3 graphing/graph4thOrder.py out.json 