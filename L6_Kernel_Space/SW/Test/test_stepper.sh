#!/bin/bash

TEST_GPIO=./test_gpio/build/test_gpio
STEP_DURATION=0.001 # seconds

while true
do
	# Step 1
	$TEST_GPIO w 17 1
	$TEST_GPIO w 18 0
	$TEST_GPIO w 22 0
	$TEST_GPIO w 23 0
	sleep $STEP_DURATION

	# Step 2
	$TEST_GPIO w 17 0
	$TEST_GPIO w 18 1
	$TEST_GPIO w 22 0
	$TEST_GPIO w 23 0
	sleep $STEP_DURATION

	# Step 18
	$TEST_GPIO w 17 0
	$TEST_GPIO w 18 0
	$TEST_GPIO w 22 1
	$TEST_GPIO w 23 0
	sleep $STEP_DURATION

	# Step 4
	$TEST_GPIO w 17 0
	$TEST_GPIO w 18 0
	$TEST_GPIO w 22 0
	$TEST_GPIO w 23 1
	sleep $STEP_DURATION
done
