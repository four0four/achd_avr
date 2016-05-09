#!/bin/sh
iverilog -o cpu.vvp cpu_tb.v avr.v
iverilog -o fetch.vvp fetch_tb.v avr.v
vvp cpu.vvp
vvp fetch.vvp
