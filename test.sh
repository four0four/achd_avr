#!/bin/sh
echo -e "\x1b[32m[+] running AVR core tests...\x1b[33m"
iverilog -o cpu.vvp cpu_tb.v avr.v
vvp cpu.vvp
echo -e "\x1b[32m[+] running instruction fetch tests...\x1b[33m"
iverilog -o fetch.vvp fetch_tb.v avr.v
vvp fetch.vvp
echo -ne "\x1b[0m"
