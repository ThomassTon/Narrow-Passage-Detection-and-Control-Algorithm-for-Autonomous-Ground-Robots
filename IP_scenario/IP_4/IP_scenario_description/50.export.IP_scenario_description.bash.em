#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  # Collect the possible values for robocup scenario by checking which world files are present
  TMP_IP_SCENARIO_PATH=$(rospack find IP_scenario_description)
  if [ $? -eq 0 ]; then
    TMP_IP_MAPS=()
    for f in ${TMP_IP_SCENARIO_PATH}/worlds/*.world; do
      TMP_IP_MAPS+=($(basename ${f%.world}))
    done
    # The double @ is necessary because catkin parses this file and uses @ as a control character
    #add_rosrs_setup_env DEFAULT_MAP_ID "$(echo "${TMP_IP_MAPS[@]}" | tr ' ' ',')" "The map used for simulation if the robocup scenario is selected."
    unset TMP_IP_MAPS
  fi
  unset TMP_IP_SCENARIO_PATH
fi
