#!/bin/bash

make

if [ $? -eq 0 ]
then
  st-flash --reset write build/*.bin 0x8000000
  st-flash --reset write build/*.bin 0x8000000
  exit 0
else
  exit -1
fi




