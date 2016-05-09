#!/bin/sh
iverilog -o fetch.vvp fetch_tb.v avr.v; vvp fetch.vvp;
