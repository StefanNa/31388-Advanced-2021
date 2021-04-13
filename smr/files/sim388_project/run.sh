#!/bin/bash

simserver1 simconfig388_final.xml & sleep 1 && ulmsserver & sleep 1  && mrc -s8000 smrGuidemarks

