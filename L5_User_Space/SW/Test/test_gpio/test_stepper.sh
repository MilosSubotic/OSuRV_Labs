#!/bin/bash

TEST_GPIO=./build/test_gpio
STEP_DURATION=0.01 # seconds

while true
do
	# Step 1
	$TEST_GPIO w 11 1
	$TEST_GPIO w 12 0
	$TEST_GPIO w 13 0
	$TEST_GPIO w 14 0
	sleep $STEP_DURATION

	# Step 2
	$TEST_GPIO w 11 0
	$TEST_GPIO w 12 1
	$TEST_GPIO w 13 0
	$TEST_GPIO w 14 0
	sleep $STEP_DURATION

	# Step 3
	$TEST_GPIO w 11 0
	$TEST_GPIO w 12 0
	$TEST_GPIO w 13 1
	$TEST_GPIO w 14 0
	sleep $STEP_DURATION

	# Step 4
	$TEST_GPIO w 11 0
	$TEST_GPIO w 12 0
	$TEST_GPIO w 13 0
	$TEST_GPIO w 14 1
	sleep $STEP_DURATION
done
