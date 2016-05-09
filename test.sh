#!/bin/sh
iverilog -o cpu.vvp cpu_tb.v avr.v
vvp cpu.vvp
