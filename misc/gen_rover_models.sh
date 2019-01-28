#!/bin/bash
# Regenerate the model SDF and config files for the 8 default rovers.
# This script should be executed any time the master model template is edited.

declare -A rovers
rovers=(
  ["achilles"]="FlatBlack"
  ["aeneas"]="Yellow"
  ["ajax"]="White"
  ["diomedes"]="Red"
  ["hector"]="Blue"
  ["paris"]="Orange"
  ["thor"]="Turquoise"
  ["zeus"]="Indigo"
)

model_path=$(catkin locate)/simulation/models
master_template_model=${model_path}/swarmie/model.sdf.erb
master_template_config=${model_path}/swarmie/model.config.erb

for rover in "${!rovers[@]}"; do
  erb -T - rovername=${rover} \
    -- ${master_template_config} > ${model_path}/${rover}/model.config
  erb -T - rovername=${rover} rovercolor=${rovers[$rover]} \
    -- ${master_template_model} > ${model_path}/${rover}/model.sdf
done

