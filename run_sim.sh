#!/bin/bash
echo -e "\x1b[32m[+] running AVR core tests...\x1b[33m"
iverilog -o cpu.vvp testbenches/cpu_tb.v avr.v
vvp cpu.vvp
echo -e "\x1b[32m[+] running instruction fetch tests...\x1b[33m"
iverilog -o fetch.vvp testbenches/fetch_tb.v avr.v
vvp fetch.vvp
echo -e "\x1b[32m[+] running integrated tests...\x1b[33m"
iverilog -o final.vvp testbenches/avr_all_tb.v avr.v
vvp final.vvp
rm *.vvp
echo -ne "\x1b[0m"
